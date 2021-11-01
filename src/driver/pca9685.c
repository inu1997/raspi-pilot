#include "pca9685.h"

#include "util/io/i2c.h"
#include "util/macro.h"
#include "util/logger.h"
#include "util/debug.h"

#include <stdlib.h>
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

static int _freq; // Frequency value cached.

void pca_atexit();

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

    LOG("Registering atexit function.\n");
    if (atexit(pca_atexit) != 0) {
        LOG_ERROR("Failed to register atexit function.\n");
        return -1;
    }

    _freq = 0;
    
    LOG("Done.\n");
    return 0;
}

/**
 * @brief Reset & initialize.
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
        if (PCA_WRITE(PCA_LED_OFF_L(i), 0) != 0) {
            return -1;
        }
        if (PCA_WRITE(PCA_LED_OFF_H(i), 0) != 0) {
            return -1;
        }
    }
    usleep(1000);
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
    usleep(1000);
    
    // Setup sleep mode (Bit 4) since prescale can only be set in sleep mode.
    if (PCA_WRITE(PCA_MODE1, old_mode | 0x10) != 0) {
        LOG_ERROR("Failed to set sleep mode.\n");
        return -1;
    }
    usleep(1000);
    
    // Set frequency.
    if (PCA_WRITE(PCA_PRE_SCALE, pre_scale) != 0) {
        LOG_ERROR("Failed to set pre scale.\n");
        return -1;
    }
    usleep(1000);
    
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

    // Update cach.
    _freq = freq;
    return 0;
}

/**
 * @brief Directly set PWM value.
 * 
 * @param chan
 *      Channel of PCA.
 * @param width
 *      Width of PWM from 0 to 4095.
 * @return 0 if success else -1.
 */
int pca_write_pwm(int chan, int width) {
    // Check channel.
    if (chan > 15 || chan < 0) {
        LOG_ERROR("Trying to set unexist channel.\n");
        return -1;
    }

    // Write value.
    if (PCA_WRITE(PCA_LED_OFF_L(chan), (width & 0xff)) != 0) {
        return -1;
    }
    if (PCA_WRITE(PCA_LED_OFF_H(chan), ((width >> 8) & 0xff)) != 0) {
        return -1;
    }

    return 0;
}

/**
 * @brief Set PCA9685 PWM value by throttle from 0 to 100.0.
 * 
 * @param chan 
 *      Channel of PCA.
 * @param throttle 
 *      Throttle value from 0.0 to 100.0.
 * @return 0 if success else -1.
 */
int pca_write_throttle(int chan, float throttle) {
    throttle = LIMIT_MAX_MIN(throttle, 100.0f, 0.0f);
    int pwm_value = (4095.0f / 100.0f) * throttle;

    return pca_write_pwm(chan, pwm_value);
}

/**
 * @brief Set PCA9685 PWM value to SERVO control signal.
 * 
 * @param chan 
 *      Channel of PCA.
 * @param micro 
 *      Microseconds from 1000 to 2000(1ms to 2ms).
 * @return 0 if success else -1.
 */
int pca_write_servo(int chan, int micro) {
    float one_ms_value = 4095.0f / (1000.0f / (float)_freq);
    micro = LIMIT_MAX_MIN(micro, 2000, 1000);
    int pwm_value = one_ms_value * ((float)(micro - 1000) / 1000.0f + 1.0f);
    
    return pca_write_pwm(chan, pwm_value);
}

/**
 * @brief ATEXIT function of PCA9685.
 * 
 */
void pca_atexit() {
    LOG("Disabling all PWM output.\n");
    int i;
    for (i = 0; i < 16; i++) {
        pca_write_pwm(i, 0);
    }
}