#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <net/if.h>
#include <poll.h>
#include <errno.h>

#ifndef USE_CANOPEND
#define USE_CANOPEND 0
#else
#include <sys/un.h>
#define SOCKET_PATH "/tmp/CO_command_socket"
#endif

#ifndef CONFIG_EXAMPLES_DMKE_CONTROLLER_CAN_CMD
#define CONFIG_EXAMPLES_DMKE_CONTROLLER_CAN_CMD "can0"
#endif

#ifdef CONFIG_EXAMPLES_DMKE_CONTROLLER_USE_2_CAN
#ifndef CONFIG_EXAMPLES_DMKE_CONTROLLER_CAN_MOTOR
#define CONFIG_EXAMPLES_DMKE_CONTROLLER_CAN_MOTOR "can1"
#endif
#define MAXSOCK (2 + USE_CANOPEND)
#else
#define MAXSOCK (1 + USE_CANOPEND)
#endif

#ifdef __NuttX__
#include <nuttx/config.h>
#include <netpacket/can.h>
#include <nuttx/can.h>
#else
#include <linux/can.h>
#include <linux/can/raw.h>
#endif

#ifndef POLL_TIMEOUT_MS
#define POLL_TIMEOUT_MS 100
#endif

#ifndef QUERY_PERIOD_MS
#define QUERY_PERIOD_MS 100
#endif

#ifndef REPORT_DELAY_MS
#define REPORT_DELAY_MS (QUERY_PERIOD_MS / 2)
#endif

#ifdef DEBUG
#define debug(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

typedef struct {
  uint16_t speed;
  uint16_t position;
  uint32_t disp   : 1;
  uint32_t ctrl   : 1;
  uint32_t enable : 1;
  uint32_t unused : 29;
} cmd_t;

#define INVALID_VALUE 0xFF
#define CTRL_MODE_POSITION 0
#define CTRL_MODE_SPEED 1

typedef struct {
  uint16_t speed;
  uint16_t position;
  uint16_t current;
  uint16_t enable : 1;
  uint16_t unused : 15;
} status_t;

// Adjust the following values to match the actual CAN IDs
enum {
    MOTOR_Z = 0x06,
    MOTOR_X = 0x12,
    MOTOR_Y = 0x13,
    MOTOR_ROLL = 0x14,
    MOTOR_PITCH = 0x15,
    MOTOR_YAW = 0x16,
    MOTOR_COUNT = 6
};

enum {
    CMD_Z = 0x105,
    CMD_X = 0x107,
    CMD_Y = 0x108,
    CMD_ROLL = 0x10A,
    CMD_PITCH = 0x10B,
    CMD_YAW = 0x10C,
    CMD_COUNT = 6
};

enum {
    STATUS_Z = 0x205,
    STATUS_X = 0x209,
    STATUS_Y = 0x208,
    STATUS_ROLL = 0x20C,
    STATUS_PITCH = 0x20A,
    STATUS_YAW = 0x207,
    STATUS_COUNT = 6
};

static const uint8_t motor_ids[MOTOR_COUNT] = {MOTOR_Z, MOTOR_X, MOTOR_Y, MOTOR_ROLL, MOTOR_PITCH, MOTOR_YAW};
static const uint16_t status_ids[MOTOR_COUNT] = {STATUS_Z, STATUS_X, STATUS_Y, STATUS_ROLL, STATUS_PITCH, STATUS_YAW};

 // Linear correction for command to motor
typedef struct {
    int a;
    int b;
} linear_correction_t;

static linear_correction_t speed_ofs[MOTOR_COUNT] = {
    {1, 0}, // Z
    {1, 0}, // X
    {1, 0}, // Y
    {1, 0}, // Roll
    {1, 0}, // Pitch
    {1, 0}  // Yaw
};

static linear_correction_t pos_ofs[MOTOR_COUNT] = {
    {1, 0}, // Z
    {1, 0}, // X
    {1, 0}, // Y
    {1, 0}, // Roll
    {1, 0}, // Pitch
    {1, 0}  // Yaw
};

static status_t status[MOTOR_COUNT]; // Leaving 0th index unused

static pthread_mutex_t mutex;

static void init_sockets(int s[])
{
    struct sockaddr_can can[MAXSOCK - USE_CANOPEND];
    int i, j;
    struct ifreq ifr;
    struct can_filter rfilter[2 - USE_CANOPEND];
#if USE_CANOPEND
    struct sockaddr_un local;
    // Create Unix socket
    if ((s[0] = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    // Set up the local address struct for the Unix socket
    memset(&local, 0, sizeof(local));
    local.sun_family = AF_UNIX;
    strncpy(local.sun_path, SOCKET_PATH, sizeof(local.sun_path) - 1);

    // Connect to the Unix socket
    if (connect(s[0], (struct sockaddr *)&local, sizeof(local)) == -1) {
        perror("connect");
        close(s[0]);
        exit(EXIT_FAILURE);
    }

    // Debug print
    debug("Connected to Unix socket %s\n", SOCKET_PATH);
#endif
    // Create CAN sockets
    for (i = USE_CANOPEND; i < MAXSOCK; ++i) {
        if ((s[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("Socket creation failed");
            exit(EXIT_FAILURE);
        }

        // Set both sockets to non-blocking mode
        if (fcntl(s[i], F_SETFL, O_NONBLOCK) < 0) {
            perror("Setting socket to non-blocking mode failed");
            for (j = 0; j <= i; ++j) close(s[j]);
            exit(EXIT_FAILURE);
        }

        // Set up CAN interface for each socket
        if (i == USE_CANOPEND) strcpy(ifr.ifr_name, CONFIG_EXAMPLES_DMKE_CONTROLLER_CAN_CMD);
#ifdef CONFIG_EXAMPLES_DMKE_CONTROLLER_USE_2_CAN
        else strcpy(ifr.ifr_name, CONFIG_EXAMPLES_DMKE_CONTROLLER_CAN_MOTOR);
#endif
        if (ioctl(s[i], SIOCGIFINDEX, &ifr) < 0) {
            perror("SIOCGIFINDEX");
            for (j = 0; j <= i; ++j) close(s[j]);
            exit(EXIT_FAILURE);
        }

        can[i - USE_CANOPEND].can_family = AF_CAN;
        can[i - USE_CANOPEND].can_ifindex = ifr.ifr_ifindex;

        // Bind sockets to the CAN interface
        if (bind(s[i], (struct sockaddr *)&can[i - USE_CANOPEND], sizeof(can[i - USE_CANOPEND])) < 0) {
            perror("Bind failed");
            for (j = 0; j <= i; ++j) close(s[j]);
            exit(EXIT_FAILURE);
        }

        // Set up filter
        rfilter[0].can_id = 0x100;
        rfilter[0].can_mask = 0x7F0;
#if !USE_CANOPEND
        rfilter[1].can_id = 0x580;
        rfilter[1].can_mask = 0x780;
#endif
        if (setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
            perror("setsockopt failed");
            for (j = 0; j <= i; ++j) close(s[j]);
            exit(EXIT_FAILURE);
        }

        // Debug print
        debug("Initialized CAN interface %s at index %d\n", ifr.ifr_name, ifr.ifr_ifindex);
    }
}

static void send_command(int s[], uint16_t id, char rw, uint16_t index, uint8_t subindex, uint8_t len, uint32_t data)
{
#if USE_CANOPEND
    char buf[100];
#else
    struct can_frame frame;
#endif
    if (rw != 'r' && rw != 'w') {
        debug("Invalid rw\n");
        return;
    }
    if (len > 4) {
        debug("Invalid len\n");
        return;
    }
#if USE_CANOPEND
    if (rw == 'r')
        sprintf(buf, "[%d] %d %c 0x%X %d u%d\n", 0, id, rw, index, subindex, len << 3);
    else if (rw == 'w')
        sprintf(buf, "[%d] %d %c 0x%X %d u%d 0x%X\n", 0, id, rw, index, subindex, len << 3, data);
#else
    frame.can_id = 0x600 | id;
    frame.can_dlc = 8;
    frame.data[1] = index & 0xFF;
    frame.data[2] = index >> 8;
    frame.data[3] = subindex;
    if (rw == 'r') {
        frame.data[0] = 0x40;
        *(uint32_t*)(&frame.data[4]) = 0;
    } else {
        frame.data[0] = 0x23 | (4 - len) << 2;
        frame.data[4] = data & 0xFF;
        frame.data[5] = (data >> 8) & 0xFF;
        frame.data[6] = (data >> 16) & 0xFF;
        frame.data[7] = (data >> 24) & 0xFF;
    }
#endif
    //Acquire mutex
    pthread_mutex_lock(&mutex);
#if USE_CANOPEND
    send(s[0], buf, strlen(buf), 0);
#else
    write(s[0], &frame, sizeof(struct can_frame));
#endif
    //Release mutex
    pthread_mutex_unlock(&mutex);
#if USE_CANOPEND
    debug("Sent: %s\n", buf);
#else
    debug("Sent: %03x#", frame.can_id);
    for (int i = 0; i < frame.can_dlc; ++i) {
        debug("%02x", frame.data[i]);
    }
    debug("\n");
#endif
}

static void send_status(int s[], uint16_t i)
{
    struct can_frame frame;
    frame.can_id = status_ids[i];
    frame.can_dlc = 8;
    memcpy(frame.data, &status[i], sizeof(status_t));
    write(s[MAXSOCK-1], &frame, sizeof(struct can_frame));
}

static void start_all(int s[])
{
    struct can_frame frame;
    frame.can_id = 0;
    frame.can_dlc = 2;
    frame.data[0] = 1;
    frame.data[1] = 0;
    write(s[USE_CANOPEND], &frame, sizeof(struct can_frame));
}

static void *status_query_thread(void *arg)
{
    int *s = (int *)arg;
    int i;
    while (1) {
        // Prepare status query frame
        for (i = 0; i < MOTOR_COUNT; ++i) {
            send_command(s, motor_ids[i], 'r', 0x6064, 0, 4, 0);
            send_command(s, motor_ids[i], 'r', 0x6069, 0, 4, 0);
            send_command(s, motor_ids[i], 'r', 0x221C, 0, 2, 0);
            send_command(s, motor_ids[i], 'r', 0x6041, 0, 2, 0);
        }
        debug("Status query sent\n");
        // Sleep for a while before the next status query
        usleep(REPORT_DELAY_MS * 1000); // 100 milliseconds
        for (i = 0; i < MOTOR_COUNT; ++i) {
            send_status(s, i);
        }
        // Sleep for a while before the next status query
        usleep((QUERY_PERIOD_MS - REPORT_DELAY_MS) * 1000); // 100 milliseconds
    }

    return NULL;
}

static int can_id_to_motor_id_index(uint32_t can_id)
{
    switch (can_id) {
        case CMD_Z:
        case 0x580 | MOTOR_Z:
            return 0;
        case CMD_X:
        case 0x580 | MOTOR_X:
            return 1;
        case CMD_Y:
        case 0x580 | MOTOR_Y:
            return 2;
        case CMD_ROLL:
        case 0x580 | MOTOR_ROLL:
            return 3;
        case CMD_PITCH:
        case 0x580 | MOTOR_PITCH:
            return 4;
        case CMD_YAW:
        case 0x580 | MOTOR_YAW:
            return 5;
        default:
            return -1;
    }
}

static void enable_motor(int s[], uint16_t id, uint8_t enable, uint8_t disp)
{
    // Send enable command to motor
    send_command(s, id, 'w', 0x6040, 0, 2, enable ? (disp ? 0x5F : 0x0F) : 0x00);
    // Debug print
    debug("Motor %d %s %s\n", id, enable ? "start" : "stop", disp ? "relatively" : "");
}

static void set_motor_speed(int s[], uint16_t id, uint16_t speed)
{
    // Set speed mode
    send_command(s, id, 'w', 0x6060, 0, 1, 3);
    // Send speed command to motor
    send_command(s, id, 'w', 0x60FF, 0, 4, speed);
    // Debug print
    debug("Motor %d speed set to %d\n", id, speed);
}

static void set_motor_position(int s[], uint16_t id, uint16_t position)
{
    // Set position mode
    send_command(s, id, 'w', 0x6060, 0, 1, 1);
    // Send position command to motor
    send_command(s, id, 'w', 0x607A, 0, 4, position);
    // Debug print
    debug("Motor %d position set to %d\n", id, position);
}

void process(int s[], struct can_frame *frame)
{
    int i = can_id_to_motor_id_index(frame->can_id);
    if (i < 0) {
        debug("Unknown can_id:0x%x\n", frame->can_id);
        return;
    }
    uint16_t id = motor_ids[i];
    if ((frame->can_id & 0x780) == 0x100) {
            // Calc speed and position
            cmd_t *cmd = (cmd_t *)frame->data;
            if (cmd->ctrl == CTRL_MODE_SPEED) {
                uint16_t speed = cmd->speed * speed_ofs[i].a + speed_ofs[i].b;
                set_motor_speed(s, id, speed);
            } else if (cmd->ctrl == CTRL_MODE_POSITION) {
                uint16_t position = cmd->position * pos_ofs[i].a + pos_ofs[i].b;
                set_motor_position(s, id, position);
            }
            enable_motor(s, id, cmd->enable, cmd->disp);
    }
#if !USE_CANOPEND
    else if ((frame->can_id & 0x780) == 0x580) {
        uint16_t reg = frame->data[1] | (frame->data[2] << 8);
        uint16_t len = 4 - (frame->data[0] >> 2 & 0x03);
        uint32_t value = frame->data[4];
        uint8_t *p = &frame->data[5];
        int j = 0;
        while (--len) value = value | *p++ << ++j*8;
        if (reg == 0x6064) {
            status[i].speed = (value - speed_ofs[i].b) / speed_ofs[i].a;
        } else if (reg == 0x6069) {
            status[i].position = (value - pos_ofs[i].b) / pos_ofs[i].a;
        } else if (reg == 0x221C) {
            status[i].current = value;
        } else if (reg == 0x6041) {
            status[i].enable = !!(value & 4);
        }
    }
#endif
    else {
        // Should not reach here
        printf("CAN error\n");
    }
}

int main(int argc, char *argv[])
{
    int s[MAXSOCK];
    struct pollfd fds[MAXSOCK];
    int ret, i, j;
    pthread_t thread_id;
    struct can_frame frame;

    // Initialize sockets
    init_sockets(s);

    //Initialize mutex
    if (pthread_mutex_init(&mutex, NULL) != 0) {
        perror("pthread_mutex_init");
        for (i = 0; i < MAXSOCK; ++i) {
            close(s[i]);
        }
        exit(EXIT_FAILURE);
    }

    // Create status query thread
    if (pthread_create(&thread_id, NULL, status_query_thread, s) != 0) {
        perror("pthread_create");
        close(s[0]);
        exit(EXIT_FAILURE);
    }

    start_all(s);

    // Setup pollfd
    for (i = 0; i < MAXSOCK; ++i) {
        fds[i].fd = s[i];
        fds[i].events = POLLIN;
    }

    // Receive messages using poll
    while (1) {
        ret = poll(fds, MAXSOCK, POLL_TIMEOUT_MS);
        if (ret == -1) {
            perror("poll");
            for (i = 0; i < MAXSOCK; ++i) {
                close(s[i]);
            }
            exit(EXIT_FAILURE);
        } else if (ret == 0) {
            // No data received within timeout
            debug(".");
            fflush(stdout);
            continue;
        }
        debug("!\n");

        // Check for activity on each socket
        for (i = 0; i < MAXSOCK; ++i) {
            if (fds[i].revents & POLLIN) {
#if USE_CANOPEND
                // Receive data from Unix socket
                if (i == 0) {
                    if (ret = recv(s[i], buf, sizeof(buf), 0) < 0) {
                        perror("Receive failed");
                        for (i = 0; i < MAXSOCK; ++i) {
                            close(s[i]);
                        }
                        return EXIT_FAILURE;
                    } else {
                        // print buf
                        debug("Received %d bytes: %s", ret, buf);
                    }
                } else
#endif
                {
                    debug("Received from CAN\n");
                    if (read(s[i], &frame, sizeof(struct can_frame)) < 0) {
                        perror("Read failed");
                        for (i = 0; i < MAXSOCK; ++i) {
                            close(s[i]);
                        }
                        return EXIT_FAILURE;
                    } else {
                        // Print frame
                        debug("Received: %03x#", frame.can_id);
                        for (j = 0; j < frame.can_dlc; ++j) {
                            debug("%02x", frame.data[j]);
                        }
                        debug("\n");
                        process(s, &frame);
                    }
                }
            }
        }
    }

    // Close sockets
    for (i = 0; i < MAXSOCK; ++i) {
        close(s[i]);
    }

    return EXIT_SUCCESS;
}
