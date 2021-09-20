/**
 * @file i2c.h
 * @author LIN 
 * @brief Linux I2C Utilities.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _I2C_H_
#define _I2C_H_

#include <stdbool.h>
#include <stdint.h>

bool i2c_device_exists(uint8_t dev_addr);

int i2c_write_array(uint8_t dev_addr, uint8_t reg_addr,uint8_t *buffer,  uint8_t n);

int i2c_read_array(uint8_t dev_addr, uint8_t reg_addr,uint8_t *buffer,  uint8_t n);

int i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

int i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

int i2c_write_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t data,  uint8_t nbit, uint8_t offset);

int i2c_read_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,  uint8_t nbit, uint8_t offset);

#endif // _I2C_H_
