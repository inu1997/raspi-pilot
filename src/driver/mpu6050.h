/**
 * @file mpu6050.h
 * @author LIN 
 * @brief Driver for MPU IMU
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#define MPU_9AXIS   // This will enable compiled executable enable direct access to AK8963.

//-----Enums-----
// DPS: Degree per second.
enum MPU_GYRO_FS{
    MPU_GYRO_FS_250_DPS = 0,
    MPU_GYRO_FS_500_DPS = 1,
    MPU_GYRO_FS_1000_DPS = 2,
    MPU_GYRO_FS_2000_DPS = 3
}; 

extern const float GYRO_SCALE_TABLE[4];

enum MPU_ACCEL_FS{
    MPU_ACCEL_FS_2_G = 0,
    MPU_ACCEL_FS_4_G = 1,
    MPU_ACCEL_FS_8_G = 2,
    MPU_ACCEL_FS_16_G = 3
};

extern const float ACCEL_SCALE_TABLE[4];

//-----

int mpu_init();

int mpu_read_all(
    int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz,
    int16_t *mx, int16_t *my, int16_t *mz,
    bool *mag_ready);

int mpu_read_accel(int16_t *x, int16_t *y, int16_t *z);

int mpu_read_gyro(int16_t *x, int16_t *y, int16_t *z);

int mpu_set_gyro_offsets(int16_t x, int16_t y, int16_t z);

int mpu_read_gyro_offsets(int16_t *x, int16_t *y, int16_t *z);

int mpu_set_sample_rate_div(uint8_t sdiv);

int mpu_get_sample_rate_div(uint8_t *sdiv);

int mpu_set_dlpf(uint8_t dlpf);

int mpu_get_dlpf(uint8_t *dlpf);

int mpu_set_accel_fchoice(uint8_t fchoice);

int mpu_get_accel_fchoice(uint8_t *fchoice);

int mpu_set_accel_dlpf(uint8_t dlpf);

int mpu_get_accel_dlpf(uint8_t *dlpf);

int mpu_set_gyro_fchoice(uint8_t fchoice);

int mpu_get_gyro_fchoice(uint8_t *fchoice);

float mpu_set_accel_fullscale(uint8_t fs);

int mpu_get_accel_fullscale(uint8_t * fs);

float mpu_set_gyro_fullscale(uint8_t fs);

int mpu_get_gyro_fullscale(uint8_t * fs);

int mpu_enable_bypass();

int mpu_disable_bypass();

int mpu_enable_master_mode();

int mpu_reset();

bool mpu_is_using_spi();

bool mpu_is_using_i2c();

//----- SLAVE0 RW Function.
int mpu_slave_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

int mpu_slave_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

int mpu_slave_read_array(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, int len);

int mpu_slave_write_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t data, uint8_t n_bit, uint8_t offset);

int mpu_slave_read_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t n_bit, uint8_t offset);

#endif // _MPU6050_H_
