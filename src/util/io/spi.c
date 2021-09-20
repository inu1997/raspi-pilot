#include "spi.h"

#include "util/logger.h"

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>

static int spi_transfer(const char *dev, uint8_t *tx, uint8_t *rx, uint8_t n);

bool spi_device_exists(const char *dev) {
    uint8_t tx_buf[1] = {0x01};
    uint8_t rx_buf[1] = {0};

    return spi_transfer(dev, tx_buf, rx_buf, 1);
}

int spi_write_array(const char *dev, uint8_t reg_addr,uint8_t *buffer,  uint8_t n) {
    unsigned char tx_buf[255];
    unsigned char rx_buf[255];
    
    tx_buf[0] = reg_addr;
    memcpy(tx_buf + 1, buffer, n);
    
    return spi_transfer(dev, tx_buf, rx_buf, n + 1);
}

int spi_read_array(const char *dev, uint8_t reg_addr,uint8_t *buffer,  uint8_t n) {
    unsigned char tx_buf[255];
    unsigned char rx_buf[255] = {0};

    tx_buf[0] = reg_addr | 0x80;
    int ret = -1;
    
    if (spi_transfer(dev, tx_buf, rx_buf, n + 1) < 0) {
        LOG_ERROR("Failed to transfer.\n");
        goto EXIT;
    }
    int i;
    for(i = 0; i < n; i++) {
        buffer[i] = rx_buf[i + 1];
    }
    EXIT:
    return ret;
}

/**
 * @brief Write 1 byte to device.
 * 
 * @param dev
 *      Path of device.
 * @param reg_addr
 *      Address of register to read/write.
 * @param data
 *      Data to write.
 * @return 0 if success, else -1.
*/
int spi_write(const char *dev, uint8_t reg_addr, uint8_t data) {
    return spi_write_array(dev, reg_addr, &data, 1);
}

/**
 * @brief Read 1 byte from device.
 * 
 * @param dev
 *      path of device.
 * @param reg_addr
 *      Address of register to read.
 * @param data
 *      Catcher of read data.
 * @return 0 if success, else -1.
*/
int spi_read(const char *dev, uint8_t reg_addr, uint8_t *data) {
    return spi_read_array(dev, reg_addr, data, 1);
}

/**
 * @brief Write bit to register.
 * 
 * @param dev
 *      Path of device.
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
int spi_write_bit(const char *dev, uint8_t reg_addr, uint8_t data,  uint8_t nbit, uint8_t offset) {
    uint8_t mask, data_mask;
    mask = (1 << nbit) - 1;
    data_mask = mask << offset;
    data = (data & mask) << offset;
    
    uint8_t orig_data;
    if (spi_read(dev, reg_addr, &orig_data) != 0) {
        LOG_ERROR("Failed to read.\n");
        return -1;
    }
    orig_data &= ~data_mask;
    orig_data |= data;
    if (spi_write(dev, reg_addr, orig_data) != 0) {
        LOG_ERROR("Failed to write.\n");
        return -1;
    }

    return 0;
}

/**
 * @brief Read bit from register.
 * 
 * @param dev
 *      Path of device.
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
int spi_read_bit(const char *dev, uint8_t reg_addr, uint8_t *data,  uint8_t nbit, uint8_t offset) {
    uint8_t orig_data;
    if (spi_read(dev, reg_addr, &orig_data) != 0) {
        LOG_ERROR("Failed to read.\n");
        return -1;
    }
    uint8_t data_mask = ((1 << nbit) - 1);
    *data = (orig_data >> offset) & data_mask;
    
    return 0;
}


static int spi_transfer(const char *dev, uint8_t *tx, uint8_t *rx, uint8_t n) {
    //----- Prepare spi_ioc_transfer struct.
    struct spi_ioc_transfer _spi;
    memset(&_spi, 0, sizeof(struct spi_ioc_transfer));
    _spi.bits_per_word = 8;
    _spi.speed_hz = 1000000;
    _spi.len = n;
    _spi.delay_usecs = 0;
    _spi.tx_buf = (unsigned long)tx;
    _spi.rx_buf = (unsigned long)rx;

    //----- Open fd.
    int spi_fd = open(dev, O_RDWR);
    int ret = -1;
    if (spi_fd < 0) {
        LOG_ERROR("Failed to open \"%s\".\n", dev);
        goto EXIT;
    }

    //----- Transfer.
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &_spi);

    EXIT:
    close(spi_fd);
    return ret;
}