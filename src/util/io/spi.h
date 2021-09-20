/**
 * @file spi.h
 * @author LIN 
 * @brief Linux SPI utilities.
 * WIP
 * 
 * 
 * @version 0.1
 * @date 2021-09-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _SPI_H_
#define _SPI_H_

#include <stdbool.h>
#include <stdint.h>

bool spi_device_exists(const char *dev);

int spi_write_array(const char *dev, uint8_t reg_addr,uint8_t *buffer,  uint8_t n);

int spi_read_array(const char *dev, uint8_t reg_addr,uint8_t *buffer,  uint8_t n);

int spi_write(const char *dev, uint8_t reg_addr, uint8_t data);

int spi_read(const char *dev, uint8_t reg_addr, uint8_t *data);

int spi_write_bit(const char *dev, uint8_t reg_addr, uint8_t data,  uint8_t nbit, uint8_t offset);

int spi_read_bit(const char *dev, uint8_t reg_addr, uint8_t *data,  uint8_t nbit, uint8_t offset);

#endif // _SPI_H_
