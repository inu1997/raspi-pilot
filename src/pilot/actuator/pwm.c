#include "pwm.h"
#include "driver/pca9685.h"

#include "util/macro.h"

static int _max = 0;
static int _min = 0;

/**
 * @brief Initiator of PWM.
 * 
 * @return 0 if success else -1.
 */
int pwm_init() {
    if (pca_init() != 0) {
        return -1;
    }
    return 0;
}

/**
 * @brief Set pwm output WITH min/max utility.
 * 
 * @param chan 
 *      PCA9685 channel.
 * @param width 
 *      Width of PWM.
 * @return 0 if success else -1.
 */
int pwm_set_width(int chan, int width) {
    return pca_set_pwm(
        chan,
        LIMIT_MAX_MIN(width, _max, _min));
}

/**
 * @brief Turn off a channel without limitation of minimun.
 * 
 * @param chan 
 *      PCA 9685 Channel.
 * @return 0 if success else -1.
 */
int pwm_turn_off(int chan){
    return pca_set_pwm(chan, 0);
}

/**
 * @brief Turn off all channel.
 * 
 */
void pwm_turn_off_all() {
    int i;
    for(i = 0; i < 16; i++) {
        pca_set_pwm(i, 0);
    }
}

//----- PWM Setting.

/**
 * @brief Set the fresh rate of PCA9685.
 * 
 * @param freq 
 *      Frequency.
 * @return 0 if success else -1.
 */
int pwm_set_freq(int freq) {
    return pca_set_frequency(freq);
}

/**
 * @brief Set the maximum width of PWM.
 * 
 * @param max 
 */
void pwm_set_max(int max) {
    _max = max;
}

/**
 * @brief Get the maximum width of PWM.
 * 
 * @return max
 */
int pwm_get_max() {
    return _max;
}

/**
 * @brief Set the minimum width of PWM.
 * 
 * @param min 
 */
void pwm_set_min(int min) {
    _min = min;
}

/**
 * @brief Get the minimun width of PWM.
 * 
 * @return min.
 */
int pwm_get_min() {
    return _min;
}
