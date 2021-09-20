#include "gpio.h"

#include "util/debug.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#define GPIO_PATH "/sys/class/gpio/gpio"
#define GPIO_EXPORT_PATH "/sys/class/gpio/export"
#define GPIO_UNEXPORT_PATH "/sys/class/gpio/unexport"

/**
 * @brief Allow /sys/class/gpio/gpip%d to be found.
 * 
 * @param index 
 *      Index of GPIO.
 * @return 0 if success else -1.
 */
int gpio_set_export(int index) {
    int fd = open(GPIO_EXPORT_PATH, O_WRONLY);
    
    int ret = 0;
    if (fd == -1) {
        printf("Failed to open export.\n");
        ret = -1;
        goto EXIT;
    }
    if (dprintf(fd, "%d", index) < 0) {
        printf("Failed to write.\n");
        ret = -1;
        goto EXIT;
    }
    EXIT:
    close(fd);
    return ret;
}

/**
 * @brief Disallow /sys/class/gpio/gpio%d.
 * 
 * @param index 
 *      Index of GPIO.
 * @return 0 if success else -1.
 */
int gpio_set_unexport(int index) {
    int fd = open(GPIO_UNEXPORT_PATH, O_WRONLY);
    
    int ret = 0;
    if (fd == -1) {
        printf("Failed to open unexport.\n");
        ret = -1;
        goto EXIT;
    }
    if (dprintf(fd, "%d", index) < 0) {
        printf("Failed to write.\n");
        ret = -1;
        goto EXIT;
    }
    EXIT:
    close(fd);
    return ret;
}

/**
 * @brief Set direction of GPIO pin.
 * 
 * @param index 
 *      GPIO index.
 * @param dir 
 *      "out" or "in"
 * @return 0 if success else -1.
 */
int gpio_set_direction(int index, const char *dir) {
    char path[64];
    sprintf(path, GPIO_PATH "%d/direction", index);
    DEBUG("Opening path \"%s\"\n", path);
    int fd = open(path, O_WRONLY);

    int ret = 0;
    if (fd == -1) {
        LOG_ERROR("Failed to open path \"%s\".\n", path);
        ret = -1;
        goto EXIT;
    }
    if (write(fd, dir, strlen(dir)) < 0) {
        LOG_ERROR("Failed to write.\n");
        ret = -1;
        goto EXIT;
    }
    EXIT:
    close(fd);
    return 0;
}

/**
 * @brief Get direction of GPIO pin.
 * 
 * @param index 
 *      GPIO index.
 * @param dest
 *      "out" or "in"
 * @return 0 if success else -1.
 */
int gpio_get_direction(int index, char *dest) {
    char path[64];
    sprintf(path, GPIO_PATH "%d/direction", index);
    DEBUG("Opening path \"%s\"\n", path);
    int fd = open(path, O_RDONLY);

    int ret = 0;
    if (fd == -1) {
        LOG_ERROR("Failed to open path \"%s\".\n", path);
        ret = -1;
        goto EXIT;
    }
    
    if (read(fd, dest, 3) < 0) {
        LOG_ERROR("Failed to read.\n");
        ret = -1;
        goto EXIT;
    }
    EXIT:
    close(fd);
    return 0;

}

/**
 * @brief Set value of GPIO pin.
 * 
 * @param index 
 *      GPIO index.
 * @param val
 *      0 or 1
 * @return 0 if success else -1.
 */
int gpio_write(int index, int val) {
    char path[64];
    sprintf(path, GPIO_PATH "%d/value", index);
    DEBUG("Opening path \"%s\"\n", path);
    int fd = open(path, O_WRONLY);

    int ret = 0;
    if (fd == -1) {
        LOG_ERROR("Failed to open path \"%s\".\n", path);
        ret = -1;
        goto EXIT;
    }
    char zero_one[2] = "01";
    if (write(fd, &zero_one[val], 1) < 0) {
        LOG_ERROR("Failed to write.\n");
        ret = -1;
        goto EXIT;
    }
    EXIT:
    close(fd);
    return 0;
}

/**
 * @brief Get value of GPIO pin.
 * 
 * @param index 
 *      GPIO index.
 * @param val
 *      0 or 1
 * @return 0 if success else -1.
 */
int gpio_read(int index, int *val) {
    char path[64];
    sprintf(path, GPIO_PATH "%d/value", index);
    DEBUG("Opening path \"%s\"\n", path);
    int fd = open(path, O_RDONLY);

    int ret = 0;
    if (fd == -1) {
        LOG_ERROR("Failed to open path \"%s\".\n", path);
        ret = -1;
        goto EXIT;
    }
    char value_str[3];
    if (read(fd, value_str, 3) < 0) {
        LOG_ERROR("Failed to read.\n");
        ret = -1;
        goto EXIT;
    }
    *val = atoi(value_str);
    EXIT:
    close(fd);
    return 0;
}