#include "mpu6050.h"
#include "util/io/i2c.h"
#include "util/logger.h"
#include "util/debug.h"

#include <unistd.h>


//-----
static uint8_t _dev_addr = 0x68; // Device address found.
#define MPU_ADDRESS _dev_addr
#define MPU_ADDRESS_AD0_HI 0x69
#define MPU_ADDRESS_AD0_LO 0x68

//----- For easy edit
#define MPU_WRITE(reg, data) i2c_write(MPU_ADDRESS, reg, data)
#define MPU_READ(reg, data) i2c_read(MPU_ADDRESS, reg, data)
#define MPU_WRITE_ARRAY(reg, data, n) i2c_write_array(MPU_ADDRESS, reg, data, n)
#define MPU_READ_ARRAY(reg, data, n) i2c_read_array(MPU_ADDRESS, reg, data, n)
#define MPU_WRITE_BIT(reg, data, n, offset) i2c_write_bit(MPU_ADDRESS, reg, data, n, offset)
#define MPU_READ_BIT(reg, data, n, offset) i2c_read_bit(MPU_ADDRESS, reg, data, n, offset)

//----- Register addresses-----

#define MPU_ADDR_SELF_TEST_X_GYRO 0x00
#define MPU_ADDR_SELF_TEST_Y_GYRO 0x01
#define MPU_ADDR_SELF_TEST_Z_GYRO 0x02
#define MPU_ADDR_SELF_TEST_X_ACCEL 0x0d
#define MPU_ADDR_SELF_TEST_Y_ACCEL 0x0e
#define MPU_ADDR_SELF_TEST_Z_ACCEL 0x0f
#define MPU_XG_OFFSET_H 0x13
#define MPU_XG_OFFSET_L 0x14
#define MPU_YG_OFFSET_H 0x15
#define MPU_YG_OFFSET_L 0x16
#define MPU_ZG_OFFSET_H 0x17
#define MPU_ZG_OFFSET_L 0x18
#define MPU_SMPLRT_DIV 0x19
#define MPU_CONFIG 0x1a
#define MPU_GYRO_CONFIG 0x1b
#define MPU_ACCEL_CONFIG 0x1c
#define MPU_ACCEL_CONFIG_2 0x1d
#define MPU_LP_ACCEL_ODR 0x1e
#define MPU_WOM_THR 0x1f
#define MPU_FIFO_EN 0x23
#define MPU_INT_PIN_CFG 0x37
#define MPU_INT_ENABLE 0x38
#define MPU_INT_STATUS 0x3a

#define MPU_ACCEL_XOUT_H 0x3b
#define MPU_ACCEL_XOUT_L 0x3c
#define MPU_ACCEL_YOUT_H 0x3d
#define MPU_ACCEL_YOUT_L 0x3e
#define MPU_ACCEL_ZOUT_H 0x3f
#define MPU_ACCEL_ZOUT_L 0x40

#define MPU_TEMP_OUT_H 41
#define MPU_TEMP_OUT_L 42

#define MPU_GYRO_XOUT_H 0x43
#define MPU_GYRO_XOUT_L 0x44
#define MPU_GYRO_YOUT_H 0x45
#define MPU_GYRO_YOUT_L 0x46
#define MPU_GYRO_ZOUT_H 0x47
#define MPU_GYRO_ZOUT_L 0x48

#define MPU_PWR_MGMT 0x6b
#define MPU_PWR_MGMT_2 0x6c

// Real velue = Reading / scale
const float GYRO_SCALE_TABLE[4] = {
    131.0f,
    65.5f,
    32.8f,
    16.4f
};

// Real velue = Reading / scale
const float ACCEL_SCALE_TABLE[4] = {
    16384.0f,
    8192.0f,
    4096.0f,
    2048.0f
};

//-----

/**
 * @brief Initiator of mpu module.
 * 
 * @return 0 if success else -1.
 */
int mpu_init(){
    LOG("Initiating MPU module.\n");

    if (i2c_device_exists(MPU_ADDRESS_AD0_HI)) {
        _dev_addr = MPU_ADDRESS_AD0_HI;
    } else if (i2c_device_exists(MPU_ADDRESS_AD0_LO)) {
        _dev_addr = MPU_ADDRESS_AD0_LO;
    } else {
        LOG_ERROR("Failed to find MPU.\n");
        return -1;
    }
    LOG("Find MPU Device at 0x%x.\n", _dev_addr);
    LOG("Resetting.\n");
    mpu_reset();

    LOG("Done.\n");
    return 0;
}

/**
 * @brief Get the sensor value in G.
 * 
 * @param x
 *      x data destination.
 * @param y
 *      y data destination.
 * @param z
 *      z data destination.
 * @return 0 if success else -1.
 */
int mpu_read_accel(int16_t *x, int16_t *y, int16_t *z){
    uint8_t data[6];
    
    if (MPU_READ_ARRAY(MPU_ACCEL_XOUT_H, data, 6) != 0){
        LOG_ERROR("Failed to read Accelerometer data.\n");
        return -1;
    }
    
    *x = ((int16_t)data[0] << 8) | data[1];
    *y = ((int16_t)data[2] << 8) | data[3];
    *z = ((int16_t)data[4] << 8) | data[5];

    return 0;
}

/**
 * @brief Get the sensor value in degree/s.
 * 
 * @param x
 *      x data destination.
 * @param y
 *      y data destination.
 * @param z
 *      z data destination.
 * @return 0 if success else -1.
 */
int mpu_read_gyro(int16_t *x, int16_t *y, int16_t *z){
    uint8_t data[6];
    
    if (MPU_READ_ARRAY(MPU_GYRO_XOUT_H, data, 6) != 0){
        LOG_ERROR("Failed to read Gyro data.\n");
        return -1;
    }
    
    *x = ((int16_t)data[0] << 8) | data[1];
    *y = ((int16_t)data[2] << 8) | data[3];
    *z = ((int16_t)data[4] << 8) | data[5];
    
    return 0;
}

/**
 * @brief Set the gyro offset in degree/s.
 * 
 * @param x
 *      x offset.
 * @param y
 *      y offset.
 * @param z
 *      z offset.
 * @return 0 if success else -1.
 */
int mpu_set_gyro_offsets(int16_t x, int16_t y, int16_t z){
    uint8_t buffer[6];
    union {
        int16_t d;
        struct {
            uint8_t high;
            uint8_t low;
        };
    } u;
    u.d = x;
    buffer[0] = u.high;
    buffer[1] = u.low;
    u.d = y;
    buffer[2] = u.high;
    buffer[3] = u.low;
    u.d = z;
    buffer[4] = u.high;
    buffer[5] = u.low;

    if (MPU_WRITE_ARRAY(MPU_XG_OFFSET_H, buffer, 6) != 0) {
        LOG_ERROR("Failed to write Gyro offsets.\n");
        return -1;
    }
    return 0;
}

/**
 * @brief Read the gyro offset in degree/s.
 *
 * @param x
 *      x data destination.
 * @param y
 *      y data destination.
 * @param z
 *      z data destination.
 * @return 0 if success else -1.
 */
int mpu_read_gyro_offsets(int16_t *x, int16_t *y, int16_t *z){
    uint8_t data[6];
    
    if (MPU_READ_ARRAY(MPU_XG_OFFSET_H, data, 6) != 0){
        LOG_ERROR("Failed to read Gyro data.\n");
        return -1;
    }
    
    *x = ((int16_t)data[0] << 8) | data[1];
    *y = ((int16_t)data[2] << 8) | data[3];
    *z = ((int16_t)data[4] << 8) | data[5];
    
    return 0;
}

/**
 * @brief Set the divider of sample rate.
 * 
 * @param sdiv
 *      Sample_rate = Interval_sample_rate / (1 + sdiv)
 * @return 0 if success, else -1.
 */
int mpu_set_sample_rate_div(uint8_t sdiv) {
    return MPU_WRITE(MPU_SMPLRT_DIV, sdiv);
}

/**
 * @brief Get the divider of sample rate.
 * 
 * @param sdiv
 *      Sample_rate = Interval_sample_rate / (1 + sdiv)
 * @return 0 if success, else -1.
 */
int mpu_get_sample_rate_div(uint8_t *sdiv) {
    return MPU_READ(MPU_SMPLRT_DIV, sdiv);
}

/**
 * @brief Set bits of DLPF in CONFIG register.
 * 
 * @param dlpf 
 * @return 0 if success, else -1.
 */
int mpu_set_dlpf(uint8_t dlpf) {
    return MPU_WRITE_BIT(MPU_CONFIG, dlpf, 3, 0);
}


/**
 * @brief Get bits of DLPF in CONFIG register.
 * 
 * @param dlpf 
 * @return 0 if success, else -1.
 */
int mpu_get_dlpf(uint8_t *dlpf) {
    return MPU_READ_BIT(MPU_CONFIG, dlpf, 3, 0);
}

/**
 * @brief Set fchoice of FCHOICE in ACCEL CONFIG 2 register.
 * 
 * @param fchoice 
 * @return 0 if success, else -1.
 */
int mpu_set_accel_fchoice(uint8_t fchoice) {
    return MPU_WRITE_BIT(MPU_ACCEL_CONFIG_2, fchoice, 1, 3);
}

/**
 * @brief Get fchoice of FCHOICE in ACCEL CONFIG 2 register.
 * 
 * @param fchoice 
 * @return 0 if success, else -1.
 */
int mpu_get_accel_fchoice(uint8_t *fchoice) {
    return MPU_READ_BIT(MPU_ACCEL_CONFIG_2, fchoice, 1, 3);
}

/**
 * @brief Set DLPF In ACCEL CONFIG 2 register.
 * 
 * @param dlpf 
 * @return 0 if success else -1.
 */
int mpu_set_accel_dlpf(uint8_t dlpf) {
    return MPU_WRITE_BIT(MPU_ACCEL_CONFIG_2, dlpf, 3, 0);
}

/**
 * @brief Get DLPF In ACCEL CONFIG 2 register.
 * 
 * @param dlpf 
 * @return 0 if success else -1.
 */
int mpu_get_accel_dlpf(uint8_t *dlpf) {
    return MPU_READ_BIT(MPU_ACCEL_CONFIG_2, dlpf, 3, 0);
}

/**
 * @brief Set FCHOICE In GYRO CONFIG register.
 * 
 * @param fchoice
 * @return 0 if success else -1.
 */
int mpu_set_gyro_fchoice(uint8_t fchoice) {
    return MPU_WRITE_BIT(MPU_GYRO_CONFIG, fchoice, 2, 0);
}

/**
 * @brief Get FCHOICE In GYRO CONFIG register.
 * 
 * @param fchoice
 * @return 0 if success else -1.
 */
int mpu_get_gyro_fchoice(uint8_t *fchoice) {
    return MPU_READ_BIT(MPU_GYRO_CONFIG, fchoice, 2, 0);
}

/**
 * @brief Set fullscale of accelerometer.
 * 
 * @param fs
 *      Defined in MPU datasheet and enum.
 * @return Scale of reading. Real_Value = Read / scale. 0.0f if fail.
 */
float mpu_set_accel_fullscale(uint8_t fs) {
    if (MPU_WRITE_BIT(MPU_ACCEL_CONFIG, fs, 2, 3) != 0) {
        return 0.0f;
    }
    
    return ACCEL_SCALE_TABLE[fs];
}

/**
 * @brief Get fullscale of accelerometer.
 * 
 * @param fs
 *      Defined in MPU datasheet and enum.
 * @return 0 if success, else -1.
 */
int mpu_get_accel_fullscale(uint8_t * fs) {
    return MPU_READ_BIT(MPU_ACCEL_CONFIG, fs, 2, 3);
}

/**
 * @brief Set fullscale of gyro.
 * 
 * @param fs
 *      Defined in MPU datasheet and enum.
 * @return Scale of reading. Real_Value = Read / scale. 0.0f if fail.
 */
float mpu_set_gyro_fullscale(uint8_t fs) {
    if (MPU_WRITE_BIT(MPU_GYRO_CONFIG, fs, 2, 3) != 0) {
        return 0.0f;
    }

    return GYRO_SCALE_TABLE[fs];
}
/**
 * @brief Get fullscale of gyro.
 * 
 * @param fs
 *      Defined in MPU datasheet and enum.
 * @return 0 if success, else -1.
 */
int mpu_get_gyro_fullscale(uint8_t * fs) {
    return MPU_READ_BIT(MPU_GYRO_CONFIG, fs, 2, 3);
}

/**
 * @brief Enable direct access to AK8963.
 * 
 * @return 0 if success, else -1.
 */
int mpu_enable_bypass() {
    return MPU_WRITE_BIT(0x37, 1, 1, 1);
}

/**
 * @brief Disable direct access to AK8963.
 * 
 * @return 0 if success, else -1.
 */
int mpu_disable_bypass() {
    return MPU_WRITE_BIT(0x37, 0, 1, 1);
}

/**
 * @brief Reset MPU module.
 * 
 * @return 0 if success, else -1.
 */
int mpu_reset() {
    if (MPU_WRITE_BIT(MPU_PWR_MGMT, 1, 1, 7) != 0) {
        LOG_ERROR("Failed to reset.\n");
        return -1;
    }
    usleep(1000);
    return 0;
}

//-----