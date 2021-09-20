#include "ak8963.h"
#include "util/io/i2c.h"
#include "util/logger.h"
#include <unistd.h>

#define AK_ADDRESS 0x0c

#define AK_WRITE(reg, data) i2c_write(AK_ADDRESS, reg, data)
#define AK_READ(reg, data) i2c_read(AK_ADDRESS, reg, data)
#define AK_WRITE_ARRAY(reg, data, n) i2c_write_array(AK_ADDRESS, reg, data, n)
#define AK_READ_ARRAY(reg, data, n) i2c_read_array(AK_ADDRESS, reg, data, n)
#define AK_WRITE_BIT(reg, data, n, offset) i2c_write_bit(AK_ADDRESS, reg, data, n, offset)
#define AK_READ_BIT(reg, data, n, offset) i2c_read_bit(AK_ADDRESS, reg, data, n, offset)

#define AK_CNTL1 0x0a
#define AK_CNTL2 0x0b
#define AK_ST1 0x02
#define AK_H_XOUT_L 0x03
#define AK_H_XOUT_H 0x04
#define AK_H_YOUT_L 0x05
#define AK_H_YOUT_H 0x06
#define AK_H_ZOUT_L 0x07
#define AK_H_ZOUT_H 0x08
#define AK_ST2 0x09
#define AK_ASAX 0x10
#define AK_ASAY 0x11
#define AK_ASAZ 0x12

//-----

/**
 * @brief Initiator of AK module.
 * 
 * @return 0 if success else -1.
 */
int ak_init(){
    LOG("Initiating AK8963.\n");
    if (!i2c_device_exists(AK_ADDRESS)) {
        LOG_ERROR("Failed to find AK8963.\n");
        return -1;
    }
    LOG("Resetting...\n");
    ak_reset();

    LOG("Done.\n");
    return 0;
}

/**
 * @brief Read sensitivity adjustment value in AK Fuse ROM.
 * RealValue = Read * ((ASA - 128) * 0.5 / 128 + 1)
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return 0 if success else -1.
 */
int ak_read_sensitivy_adjustment_value(int8_t *x, int8_t *y, int8_t *z){
    uint8_t orig_cntl1;
    if (AK_READ(AK_CNTL1, &orig_cntl1) != 0) {
        return -1;
    }
    
    if (ak_set_mode(AK_MODE_FUSE_ROM_ACCESS) != 0) {
        LOG_ERROR("Failed to set FUSE ROM ACCESS MODE.\n");
        return -1;
    }

    int8_t d[3];
    if (AK_READ_ARRAY(AK_ASAX, d, 3) != 0){
        LOG("Failed to read sensitivity adjustment value.\n");
        return -1;
    }

    *x = d[0];
    *y = d[1];
    *z = d[2];
    
    if (AK_WRITE(AK_CNTL1, orig_cntl1) != 0) {
        return -1;
    }
    return 0;
}

/** 
 * @brief AK read function in single measurement mode.
 * 
 * @param x
 * @param y
 * @param z
 * @return 0 if success else -1.
 */
int ak_read_single(int16_t *x, int16_t *y, int16_t *z){
    uint8_t d[8];
    AK_READ_ARRAY(AK_ST1, d, 8);

    if (d[0] & 0x01) {
        // DATA READY
        if (!(d[7] & 0x08)) {
            // NO OVERFLOW
            *x = ((int16_t)d[2]) << 8 | d[1];
            *y = ((int16_t)d[4]) << 8 | d[3];
            *z = ((int16_t)d[6]) << 8 | d[5];
        } else {
            LOG_ERROR("Overflow!\n");
            return -1;
        }
    } else {
        // NOT READY
        return -1;
    }
    ak_set_mode(AK_MODE_SINGLE_MEASUREMENT);
    return 0;
}

/**
 * @brief AK read function for Continuous mode.
 * 
 * @param x
 * @param y
 * @param z
 * @return 0 if success else -1.
 */
int ak_read(int16_t *x, int16_t *y, int16_t *z){
    uint8_t d[7];
    if (AK_READ_ARRAY(AK_H_XOUT_L, d, 7) != 0) {
        LOG_ERROR("Failed to read AK.\n");
        return -1;
    }
    *x = ((int16_t)d[1] << 8) | d[0];
    *y = ((int16_t)d[3] << 8) | d[2];
    *z = ((int16_t)d[5] << 8) | d[4];

    if ((d[6] & 0x08) == 0x08) {
        LOG_ERROR("Overflow occur!\n");
        return -1;
    }
    return 0;
}

/**
 * @brief Set AK8963 to 16 bit output mode.
 * 
 * @return 0 if success else -1.
 */
int ak_set_16_bit() {
    return AK_WRITE_BIT(AK_CNTL1, 1, 1, 4);
}

/**
 * @brief Set AK8963 to 14 bit output mode.
 * 
 * @return 0 if success else -1.
 */
int ak_set_14_bit() {
    return AK_WRITE_BIT(AK_CNTL1, 0, 1, 4);
}

/**
 * @brief Get the bit in CNTL1 bit.
 * 
 * @param bit 
 * @return 0 if success else -1.
 */
int ak_get_bit(uint8_t *bit) {
    return AK_READ_BIT(AK_CNTL1, bit, 1, 4);
}

int ak_set_mode(uint8_t mode) {
    return AK_WRITE_BIT(AK_CNTL1, mode, 4, 0);
}

int ak_get_mode(uint8_t *mode) {
    return AK_READ_BIT(AK_CNTL1, mode, 4, 0);
}

int ak_reset() {
    return AK_WRITE_BIT(AK_CNTL2, 1, 1, 0);
}

//-----