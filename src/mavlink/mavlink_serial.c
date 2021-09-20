#include "mavlink_serial.h"
#include "mavlink_main.h"
#include "util/logger.h"
#include "util/debug.h"
#include "util/system/scheduler.h"

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>


int mavlink_init_serial(const char *dev) {
    // Initialize serial device.
    int fd;
    if ((fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY)) == -1) {
        LOG_ERROR("Failed to open device \"%s\".\n", dev);
        return -1;
    }

    LOG("Successfully opened device\"%s\".\n", dev);
    LOG("Initializing device.\n");

    struct termios t;

    tcgetattr(fd, &t);
    cfsetispeed(&t, B57600);
    cfsetospeed(&t, B57600);
    
    // 8 bit data
    t.c_cflag |= CS8; // set CS

    // number of stop bit 1
    t.c_cflag &= ~CSTOPB;

    // no parity
    t.c_cflag &= ~(PARENB | PARODD);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;

    // Attributes
    t.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
    t.c_oflag &= ~(OPOST);
    t.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);

    tcsetattr(fd, TCSANOW, &t);
    
    struct MAVLinkFile *connection = malloc(sizeof(struct MAVLinkFile));
    connection->fd = fd;
    strcpy(connection->name, dev);
    connection->on_begin = NULL;
    connection->on_end = NULL;
    connection->mav_channel = mavlink_ocupy_usable_channel();
    connection->exit_on_error = false;
    pthread_t thread;
    return pthread_create(&thread, NULL, mavlink_connection_handler, (void*)connection);
}
