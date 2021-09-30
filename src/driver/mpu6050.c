#include "mpu6050.h"
#include "util/io/i2c.h"
#include "util/io/spi.h"

#include "util/logger.h"
#include "util/debug.h"

#include <unistd.h>

//-----
static uint8_t _dev_addr = 0x68; // Device address found.
#define MPU_ADDRESS _dev_addr
#define MPU_ADDRESS_AD0_HI 0x69
#define MPU_ADDRESS_AD0_LO 0x68

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

#define MPU_EXT_SENS_DATA_00 0x49

#define MPU_PWR_MGMT 0x6b
#define MPU_PWR_MGMT_2 0x6c

#define MPU_USER_CTRL 0x6a
#define MPU_I2C_DELAY_CTRL 0x67
#define MPU_I2C_MST_CTRL 0x24

#define MPU_I2C_SLV0_ADDR 0x25
#define MPU_I2C_SLV0_REG  0x26
#define MPU_I2C_SLV0_CTRL 0x27

#define MPU_I2C_SLV1_ADDR 0x28
#define MPU_I2C_SLV1_REG  0x29
#define MPU_I2C_SLV1_CTRL 0x2a

#define MPU_I2C_SLV2_ADDR 0x2b
#define MPU_I2C_SLV2_REG  0x2c
#define MPU_I2C_SLV2_CTRL 0x2d

#define MPU_I2C_SLV3_ADDR 0x2e
#define MPU_I2C_SLV3_REG  0x2f
#define MPU_I2C_SLV3_CTRL 0x30

#define MPU_I2C_SLV4_ADDR 0x31
#define MPU_I2C_SLV4_REG  0x32
#define MPU_I2C_SLV4_DO   0x33
#define MPU_I2C_SLV4_CTRL 0x34
#define MPU_I2C_SLV4_DI   0x35

#define MPU_I2C_SLV0_DO 0x63
#define MPU_I2C_SLV1_DO 0x64
#define MPU_I2C_SLV2_DO 0x65
#define MPU_I2C_SLV3_DO 0x66


#define MPU_WHO_AM_I 0x75

#define MPU_USE_SPI

#ifdef MPU_USE_SPI
// Use SPI.
#define MPU_SPI_DEVICE "/dev/spidev0.0"

#define MPU_WRITE(reg, data) spi_write(MPU_SPI_DEVICE, reg, data)
#define MPU_READ(reg, data) spi_read(MPU_SPI_DEVICE, reg, data)
#define MPU_WRITE_ARRAY(reg, data, n) spi_write_array(MPU_SPI_DEVICE, reg, data, n)
#define MPU_READ_ARRAY(reg, data, n) spi_read_array(MPU_SPI_DEVICE, reg, data, n)
#define MPU_WRITE_BIT(reg, data, n, offset) spi_write_bit(MPU_SPI_DEVICE, reg, data, n, offset)
#define MPU_READ_BIT(reg, data, n, offset) spi_read_bit(MPU_SPI_DEVICE, reg, data, n, offset)

#else
// Use I2C.
#define MPU_WRITE(reg, data) i2c_write(MPU_ADDRESS, reg, data)
#define MPU_READ(reg, data) i2c_read(MPU_ADDRESS, reg, data)
#define MPU_WRITE_ARRAY(reg, data, n) i2c_write_array(MPU_ADDRESS, reg, data, n)
#define MPU_READ_ARRAY(reg, data, n) i2c_read_array(MPU_ADDRESS, reg, data, n)
#define MPU_WRITE_BIT(reg, data, n, offset) i2c_write_bit(MPU_ADDRESS, reg, data, n, offset)
#define MPU_READ_BIT(reg, data, n, offset) i2c_read_bit(MPU_ADDRESS, reg, data, n, offset)
#endif // MPU_USE_SPI

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
    // Check WHO AM I

    uint8_t who_am_i;
    if (MPU_READ(MPU_WHO_AM_I, &who_am_i) != 0) {
        LOG_ERROR("Failed to read.\n");
        return -1;
    }
    if (who_am_i != 0x71) {
        LOG_ERROR("WHO AM I doesn't match.(who_am_i: 0x%02x)\n", who_am_i);
        return -1;
    }
    usleep(1000);

    LOG("Resetting.\n");
    mpu_reset();

    MPU_WRITE(MPU_PWR_MGMT, 0x01);
    MPU_WRITE(MPU_INT_PIN_CFG, 0x30);

    LOG("Done.\n");
    return 0;
}

/**
 * @brief Read all data at once.
 * 
 * @param ax Accelerometer reading x. 
 * @param ay Accelerometer reading y. 
 * @param az Accelerometer reading z. 
 * @param gx Gyroscope reading x.
 * @param gy Gyroscope reading y.
 * @param gz Gyroscope reading z.
 * @param mx Magnetometer reading x.
 * @param my Magnetometer reading y.
 * @param mz Magnetometer reading z.
 * @param mag_ready 
 *      True if AK8963 get new reading else false.
 * @return 0 if success else -1.
 */
int mpu_read_all(
    int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz,
    int16_t *mx, int16_t *my, int16_t *mz,
    bool *mag_ready) {

    // 0x0c: Address of AK8963.
    if (MPU_WRITE(MPU_I2C_SLV0_ADDR, 0x0c | 0x80) != 0) {
        LOG_ERROR("Failed to write addr.\n");
        return -1;
    }
    // Start from AK_XOUTL
    if (MPU_WRITE(MPU_I2C_SLV0_REG, 0x02) != 0) {
        LOG_ERROR("Failed to write reg.\n");
        return -1;
    }
    if (MPU_WRITE(MPU_I2C_SLV0_CTRL, (0x88)) != 0) {
        LOG_ERROR("Failed to write ctrl.\n");
        return -1;
    }

    uint8_t buf[22]; // ACCEL: 6, TEMP: 2, GYRO:6, EXT MAG: 8.
    MPU_READ_ARRAY(MPU_ACCEL_XOUT_H, buf, sizeof(buf));


    *ax = ((int16_t)buf[0] << 8) | buf[1];
    *ay = ((int16_t)buf[2] << 8) | buf[3];
    *az = ((int16_t)buf[4] << 8) | buf[5];
    
    *gx = ((int16_t)buf[8] << 8) | buf[9];
    *gy = ((int16_t)buf[10] << 8) | buf[11];
    *gz = ((int16_t)buf[12] << 8) | buf[13];

    *mag_ready = false;

    if (buf[14] & 0x01) {
        // Data ready in ST1.
        if (!(buf[21] &0x08)) {
            // No overflow in ST2
            *mx = ((int16_t)buf[16]) << 8 | buf[15];
            *my = ((int16_t)buf[18]) << 8 | buf[17];
            *mz = ((int16_t)buf[20]) << 8 | buf[19];
            *mag_ready = true;
        } else {
            LOG_ERROR("Overflow!\n");
        }
    }
    
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
 * @brief Enable master mode of MPU.
 * 
 * @return 0 if success else -1.
 */
int mpu_enable_master_mode() {
    int ret = -1;
    
    if ((ret = mpu_disable_bypass()) != 0) {
        LOG_ERROR("Failed to disable bypass.(ret: %d)\n", ret);
        goto EXIT;
    }
    usleep(1000);
    
    // Reset I2C_MST
    if ((ret = MPU_WRITE(MPU_USER_CTRL, 0x02)) != 0) {
        LOG_ERROR("Failed to reset I2C master mode.\n");
        goto EXIT;
    }
    usleep(1000);

    if ((ret = MPU_WRITE(MPU_USER_CTRL, 0x20)) != 0) {
        LOG_ERROR("Failed to set USER_CTRL.\n");
        goto EXIT;
    }
    usleep(1000);
    
    // Set 0x02 for 320kHz I2C speed. 
    if ((ret = MPU_WRITE(MPU_I2C_MST_CTRL, 0x02)) != 0) {
        LOG_ERROR("Failed to set I2C_MST_CTRL.\n");
        goto EXIT;
    }
    usleep(1000);
    
    // if ((ret = MPU_WRITE(MPU_I2C_DELAY_CTRL, 0x80)) != 0) {
    //     LOG_ERROR("Failed to set I2C_DELAY_CTRL.\n");
    //     goto EXIT;
    // }
    // usleep(1000);
    
    // if ((ret = MPU_WRITE(MPU_I2C_SLV4_CTRL, 0x01)) != 0) {
    //     LOG_ERROR("Failed to set I2C_SLV4_CTRL.\n");
    //     goto EXIT;
    // }
    // usleep(1000);

    EXIT:
    return ret;
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

/**
 * @brief Check if MPU is using SPI interface.
 * 
 * @return True if using SPI else false.
 */
bool mpu_is_using_spi() {
#ifdef MPU_USE_SPI
    return true;
#else
    return false;
#endif // MPU_USE_SPI
}

/**
 * @brief The opposite of mpu_is_using_spi.
 * 
 * @return True if using I2C else false.
 */
bool mpu_is_using_i2c() {
    return !mpu_is_using_spi();
}

//----- SLAVE0 RW Function.
int mpu_slave_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    if (MPU_WRITE(MPU_I2C_SLV4_ADDR, dev_addr) != 0) {
        LOG_ERROR("Failed to write addr.\n");
        return -1;
    }
    usleep(1000);
    if (MPU_WRITE(MPU_I2C_SLV4_REG, reg_addr) != 0) {
        LOG_ERROR("Failed to write reg.\n");
        return -1;
    }
    usleep(1000);
    if (MPU_WRITE(MPU_I2C_SLV4_DO, data) != 0) {
        LOG_ERROR("Failed to write data.\n");
        return -1;
    }
    usleep(1000);
    if (MPU_WRITE(MPU_I2C_SLV4_CTRL, 0x80) != 0) {
        LOG_ERROR("Failed to write ctrl.\n");
        return -1;
    }
    usleep(1000);
    return 0;
}

int mpu_slave_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    if (MPU_WRITE(MPU_I2C_SLV4_ADDR, dev_addr | 0x80) != 0) {
        LOG_ERROR("Failed to write addr.\n");
        return -1;
    }
    usleep(1000);
    if (MPU_WRITE(MPU_I2C_SLV4_REG, reg_addr) != 0) {
        LOG_ERROR("Failed to write reg.\n");
        return -1;
    }
    usleep(1000);
    if (MPU_WRITE(MPU_I2C_SLV4_CTRL, 0x80) != 0) {
        LOG_ERROR("Failed to write ctrl.\n");
        return -1;
    }
    usleep(1000);
    if (MPU_READ(MPU_I2C_SLV4_DI, data) != 0) {
        LOG_ERROR("Failed to read.\n");
        return -1;
    }
    usleep(1000);
    return 0;
}

int mpu_slave_read_array(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, int len) {
    if (MPU_WRITE(MPU_I2C_SLV0_ADDR, dev_addr | 0x80) != 0) {
        LOG_ERROR("Failed to write addr.\n");
        return -1;
    }
    usleep(1000);
    if (MPU_WRITE(MPU_I2C_SLV0_REG, reg_addr) != 0) {
        LOG_ERROR("Failed to write reg.\n");
        return -1;
    }
    usleep(1000);
    if (MPU_WRITE(MPU_I2C_SLV0_CTRL, (0x80 | len)) != 0) {
        LOG_ERROR("Failed to write ctrl.\n");
        return -1;
    }
    usleep(1000);
    if (MPU_READ_ARRAY(MPU_EXT_SENS_DATA_00, buf, len) != 0) {
        LOG_ERROR("Failed to read.\n");
        return -1;
    }
    usleep(1000);
    return 0;
}

int mpu_slave_write_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t data, uint8_t n_bit, uint8_t offset) {
    uint8_t mask, data_mask;
    mask = (1 << n_bit) - 1;
    data_mask = mask << offset;
    data = (data & mask) << offset;
    
    uint8_t orig_data;
    if (mpu_slave_read(dev_addr, reg_addr, &orig_data) != 0) {
        printf("Failed to read.\n");
        return -1;
    }
    orig_data &= ~data_mask;
    orig_data |= data;
    if (mpu_slave_write(dev_addr, reg_addr, orig_data) != 0) {
        printf("Failed to write.\n");
        return -1;
    }

    return 0;
}

int mpu_slave_read_bit(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t n_bit, uint8_t offset) {
    uint8_t orig_data;
    if (mpu_slave_read(dev_addr, reg_addr, &orig_data) != 0) {
        printf("Failed to read.\n");
        return -1;
    }
    uint8_t data_mask = ((1 << n_bit) - 1);
    *data = (orig_data >> offset) & data_mask;
    
    return 0;
}

//-----