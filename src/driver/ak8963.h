/**
 * @file ak8963.h
 * @author LIN 
 * @brief Driver for AK8963 magnetometer.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _AK8963_H_
#define _AK8963_H_

#include <stdint.h>

enum AK_MODE {
    AK_MODE_POWER_DOWN = 0,
    AK_MODE_SINGLE_MEASUREMENT = 0x01,
    AK_MODE_CONTINUOUS_MEASUREMENT_8HZ = 0x02,
    AK_MODE_CONTINUOUS_MEASUREMENT_100HZ = 0x06,
    AK_MODE_SELF_TEST = 0x08,
    AK_MODE_FUSE_ROM_ACCESS = 0x0f
};

int ak_init();

int ak_reset();

int ak_read_single(int16_t *x, int16_t *y, int16_t *z);

int ak_read(int16_t *x, int16_t *y, int16_t *z);

int ak_read_sensitivy_adjustment_value(int8_t *x, int8_t *y, int8_t *z);

int ak_set_16_bit();

int ak_set_14_bit();

int ak_get_bit(uint8_t *bit);

int ak_set_mode(uint8_t mode);

int ak_get_mode(uint8_t *mode);

#endif // _AK8963_H_
