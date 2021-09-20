#include "pca9685.h"

#include "util/io/i2c.h"
#include "util/logger.h"
#include "util/debug.h"

#include <unistd.h>

#define PCA_ADDRESS 0x40

#define PCA_WRITE(reg, data) i2c_write(PCA_ADDRESS, reg, data)
#define PCA_READ(reg, data) i2c_read(PCA_ADDRESS, reg, data)
#define PCA_WRITE_ARRAY(reg, data, n) i2c_write_array(PCA_ADDRESS, reg, data, n)
#define PCA_READ_ARRAY(reg, data, n) i2c_read_array(PCA_ADDRESS, reg, data, n)
#define PCA_WRITE_BIT(reg, data, n, offset) i2c_write_bit(PCA_ADDRESS, reg, data, n, offset)
#define PCA_READ_BIT(reg, data, n, offset) i2c_read_bit(PCA_ADDRESS, reg, data, n, offset)


#define PCA_MODE1 0x00
#define PCA_MODE2 0x01
#define PCA_LED0_ON_L 0x06
#define PCA_LED0_ON_H 0x07
#define PCA_LED0_OFF_L 0x08
#define PCA_LED0_OFF_H 0x09
#define PCA_LED_ON_L(index) (PCA_LED0_ON_L + 4 * (index))
#define PCA_LED_ON_H(index) (PCA_LED0_ON_H + 4 * (index))
#define PCA_LED_OFF_L(index) (PCA_LED0_OFF_L + 4 * (index))
#define PCA_LED_OFF_H(index) (PCA_LED0_OFF_H + 4 * (index))
#define PCA_PRE_SCALE 0xfe
#define PCA_CLOCK_FREQ 25000000.0f // 25MHz default clock

/**
 * @brief Initiator of PCA module.
 * 
 * @return 0 if success else -1.
 */
int pca_init() {
    LOG("Initiating PCA9685.\n");
    if (!i2c_device_exists(PCA_ADDRESS)) {
        LOG_ERROR("Failed to find PCA module.\n");
        return -1;
    }
    
    LOG("Resetting.\n");
    if (pca_reset() != 0) {
        LOG_ERROR("Failed to reset.\n");
        return -1;
    }
    
    LOG("Done.\n");
    return 0;
}

/**
 * @brief Set frequency of PCA.
 * 
 * @param freq
 *      Frequency.
 * @return 0 if success else -1.
 */
int pca_set_frequency(int freq) {
    uint8_t pre_scale = (PCA_CLOCK_FREQ / 4096 / freq) - 1;
    uint8_t old_mode;
    if (PCA_READ(PCA_MODE1, &old_mode) != 0) {
        LOG_ERROR("Failed to read mode.\n");
        return -1;
    }

    // Setup sleep mode (Bit 4) since prescale can only be set in sleep mode.
    if (PCA_WRITE(PCA_MODE1, old_mode | 0x10) != 0) {
        LOG_ERROR("Failed to set sleep mode.\n");
        return -1;
    }
    // Set frequency.
    if (PCA_WRITE(PCA_PRE_SCALE, pre_scale) != 0) {
        LOG_ERROR("Failed to set pre scale.\n");
        return -1;
    }
    // Restore mode.
    if(PCA_WRITE(PCA_MODE1, old_mode) != 0) {
        LOG_ERROR("Failed to restore mode.\n");
        return -1;
    }
    usleep(1000);
    // Restart
    if (PCA_WRITE(PCA_PRE_SCALE, old_mode | 0x80) != 0) {
        LOG_ERROR("Failed to restart.\n");
        return -1;
    }
    usleep(1000);
    DEBUG("Prescale: 0x%02x.\n", pre_scale);
    return 0;
}

/**
 * @brief Set PWM.
 * 
 * @param chan
 *      Channel of PCA.
 * @param width
 *      Width of PWM.
 * @return 0 if success else -1.
 */
int pca_set_pwm(int chan, int width) {
    if (PCA_WRITE(PCA_LED_OFF_L(chan), (width & 0xff)) != 0) {
        return -1;
    }
    if (PCA_WRITE(PCA_LED_OFF_H(chan), ((width >> 8) & 0xff)) != 0) {
        return -1;
    }
    DEBUG("Set PWM: %d.\n", width);

    return 0;
}

/**
 * @brief Reset & initialize PWM ON delay.
 * 
 * @return 0 if success else -1.
 */
int pca_reset() {
    if (PCA_WRITE(PCA_MODE1, 0) != 0) {
        return -1;
    }
    if (PCA_WRITE(PCA_MODE2, 0x04) != 0) {
        return -1;
    }
    
    usleep(1000);
    int i = 0;
    for (i = 0; i < 16; i++) {
        // 0 delay time for every channel
        if (PCA_WRITE(PCA_LED_ON_L(i), 0) != 0) {
            return -1;
        }
        if (PCA_WRITE(PCA_LED_ON_H(i), 0) != 0) {
            return -1;
        }
    }
    usleep(1000);
    return 0;
} 