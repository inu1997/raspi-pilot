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

void *serial_handler(void *arg);

int mavlink_init_serial(const char *dev) {
    char *_dev = malloc(32);
    strcpy(_dev, dev);
    pthread_t thread;
    return pthread_create(&thread, NULL, serial_handler, (void*)_dev);
}

/**
 * @brief Serial communication handler.
 * 
 * @param arg 
 *      Device path.
 * @return void* 
 */
void *serial_handler(void *arg) {
    pthread_detach(pthread_self());

    char dev[32]; // Path to device.
    strcpy(dev, arg);
    free(arg);
    // Initialize serial device.
    int fd;
    if ((fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY)) == -1) {
        LOG_ERROR("Failed to open device \"%s\".\n", dev);
        goto EXIT;
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
    t.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL | IXON | IXOFF);
    t.c_oflag &= ~(OPOST | ONLCR);
    t.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);
    t.c_cc[VMIN] = 0;
    tcsetattr(fd, TCSANOW, &t);

    mavlink_communication(fd, false, false, true);

    EXIT:
    pthread_exit(NULL);
}