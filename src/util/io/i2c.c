#include "i2c.h"
#include "util/logger.h"
#include "util/debug.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define I2C_PATH "/dev/i2c-1"

/**
 * @brief Check if the device exists.
 * 
 * @param dev_addr
 *      I2C Address of device.
 * @return True if the device is found else false.
*/
bool i2c_device_exists(uint8_t dev_addr){
    bool ret = false;

    int fd = -1; // I2C file descriptor.
    int reg_addr = 0x01;    // for test
    
    fd = open(I2C_PATH, O_RDWR);
    if (fd < 0){
        LOG_ERROR("Failed to open i2c.\n");
        goto EXIT;
    }
    if (ioctl(fd, I2C_SLAVE, dev_addr) < 0){
        LOG_ERROR("Failed to open device.\n");
        goto EXIT;
    }
    if (write(fd, &reg_addr, 1) != 1) {
        LOG_ERROR("Failed to contact with device.\n");
        goto EXIT;
    }

    ret = true;
    EXIT:
    close(fd);
    return ret;
}

/**
 * @brief Write data to device.
 * 
 * @param dev_addr
 *      Address of device.
 * @param reg_addr
 *      Address of register to write.
 * @param buffer
 *      data to write.
 * @param n
 *      Length of buffer.
 * @return 0 if success, else -1.
*/
int i2c_write_array(uint8_t dev_addr, uint8_t reg_addr,uint8_t *buffer,  uint8_t n) {
    if (n > 127) {
        LOG_ERROR("Length %d is too long.\n", n);
        return -1;
    }
    
    int ret = -1;
    int fd; // I2C file descriptor.
    uint8_t _buf[128]; // Buffer to write through I2C.
    int w_cnt;  // Write count.
    
    // Open device.
    fd = open(I2C_PATH, O_RDWR);
    if (fd < 0){
        LOG_ERROR("Failed to open i2c.\n");
        goto EXIT;
    }

    // Open I2C device.
    if ((ret = ioctl(fd, I2C_SLAVE, dev_addr)) < 0){
        LOG_ERROR("Failed to open device. Address: %d\n", dev_addr);
        goto EXIT;
    }
    
    // Write to device
    // Prepare buffer.
    _buf[0] = reg_addr;
    memcpy(_buf + 1, buffer, n);
    // Write out.
    if ((w_cnt = write(fd, _buf, n + 1)) != n + 1){
        LOG_ERROR("Write count doesn't match.(Expect: %d, got: %d)\n", n + 1, w_cnt);
        goto EXIT;
    }

    ret = 0;
    EXIT:
    close(fd);
    return ret;
}

/**
 * @brief Read data from device.
 * 
 * @param dev_addr
 *      Address of device.
 * @param reg_addr
 *      Address of register to read.
 * @param buffer
 *      Buffer to hold read data.
 * @param n
 *      Length of buffer.
 * @return 0 if success, else -1.
*/
int i2c_read_array(uint8_t dev_addr, uint8_t reg_addr,uint8_t *buffer,  uint8_t n) {
    int ret = -1;

    // Open I2C device.
    int fd = open(I2C_PATH, O_RDWR);
    if (fd < 0) {
        LOG_ERROR("Failed to open i2c.\n");
        goto EXIT;
    }
    // Open I2C device.
    if ((ret = ioctl(fd, I2C_SLAVE, dev_addr)) < 0){
        LOG_ERROR("Failed to open device.(Address: %d)\n", dev_addr);
        goto EXIT;
    }
    // Write register address.
    if (write(fd, &reg_addr, 1) != 1) {
        LOG_ERROR("Failed to write register.(Address: %d)\n", reg_addr);
        goto EXIT;
    }
    // Read.
    int r_cnt;
    if ((r_cnt = read(fd, buffer, n)) != n) {
        LOG_ERROR("Read count doesn't match.(Expect: %d, got: %d).\n", n, r_cnt);
        goto EXIT;
    }
    
    ret = 0;
    EXIT:
    close (fd);
    return ret;
}

/**
 * @brief Write 1 byte to device.
 * 
 * @param dev_addr
 *      Address of device.
 * @param reg_addr
 *      Address of register to read/write.
 * @param data
 *      Data to write.
 * @return 0 if success, else -1.
*/
int i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    return i2c_write_array(dev_addr, reg_addr, &data, 1);
}

/**
 * @brief Read 1 byte from device.
 * 
 * @param dev_addr
 *      Address of device.
 * @param reg_addr
 *      Address of register to read.
 * @param data
 *      Catcher of read data.
 * @return 0 if success, else -1.
*/
int i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    return i2c_read_array(dev_addr, reg_addr, data, 1);
}

/**
 * @brief Write bit to register.
 * 
 * @param dev_addr
 *      Address of device.
 * @param reg_addr 
 *      Address of register.
 * @param data 
 *      Bits to write.
 * @param nbit
 *      Number of bits.
 * @param offset
 *      Offset of bits.
 * @return 0 if success, else -1.
 */
int i2c_write_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t data,  uint8_t nbit, uint8_t offset) {
    int ret;
    uint8_t orig_data;
    if ((ret = i2c_read(dev_addr, reg_addr, &orig_data)) != 0) {
        LOG_ERROR("Failed to read.\n");
        goto EXIT;
    };

    uint8_t mask = (1 << nbit) - 1; // Mask for nbit.
    orig_data &= ~(mask << offset); // Clear bits.
    orig_data |= (data & mask) << offset; // Set bits.
    
    if ((ret = i2c_write(dev_addr, reg_addr, orig_data)) != 0) {
        LOG_ERROR("Failed to write.\n");
        goto EXIT;
    }

    EXIT:
    return ret;
}

/**
 * @brief Read bit from register.
 * 
 * @param dev_addr
 *      Address of device.
 * @param reg_addr 
 *      Address of register.
 * @param data 
 *      Bits to read.
 * @param nbit
 *      Number of bits.
 * @param offset
 *      Offset of bits.
 * @return 0 if success, else -1.
 */
int i2c_read_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,  uint8_t nbit, uint8_t offset) {
    uint8_t orig_data;
    if (i2c_read(dev_addr, reg_addr, &orig_data) != 0) {
        LOG_ERROR("Failed to read.\n");
        return -1;
    }

    uint8_t mask = (1 << nbit) - 1;
    *data = (orig_data >> offset) & mask; // Crop out unnecessary bits.
    
    return 0;
}