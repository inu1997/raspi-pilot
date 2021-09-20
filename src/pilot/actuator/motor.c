#include "motor.h"
#include "driver/pca9685.h"

#include "util/logger.h"
#include "util/macro.h"
#include "util/debug.h"
#include "util/parameter.h"

#include <stdlib.h>

static int _pwm_max;
static int _pwm_min;

/**
 * @brief Initiator of motor.
 * 
 * @return 0 if success else -1.
 */
int motor_init(){
    LOG("Initiating motor.\n");
    //----- Device
    if (pca_init() != 0) {
        LOG_ERROR("Failed to init pca module.\n");
        return -1;
    }
    
    if (atexit(motor_turn_off_all) != 0) {
        LOG_ERROR("Failed to register atexit.\n");
        return -1;
    }

    //----- Variables
    int freq, max, min;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MTR_PWM_FREQ], &freq);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MTR_PWM_MAX], &max);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MTR_PWM_MIN], &min);
    motor_set_pwm_freq(freq);
    motor_set_pwm_max(max);
    motor_set_pwm_min(min);
    
    return 0;
}

/**
 * @brief Update throttle value to real world.
 * 
 * @param chan
 *      PCA9685 Channel.
 * @param throttle
 *      The throttle value by percent(0.1 ~ 100.0f). 0.0 to turn off
 * @return 0 if success else -1.
 */
int motor_set(int chan, float throttle) {
    // Do actuator.
    if (throttle == 0.0) {
        pca_set_pwm(chan, 0);
    } else {
        int width = (float)_pwm_min + (float)(_pwm_max - _pwm_min) * (throttle / 100.0f);
        pca_set_pwm(chan, width);
    }
    return 0;
}

/**
 * @brief Turn off a motor on the channel.
 * 
 * @param chan 
 *      PCA9685 Channel.
 * @return 0 if success else -1.
 */
int motor_turn_off(int chan) {
    pca_set_pwm(chan, 0);
}

/**
 * @brief Turn off all motor. Registed atexit.
 */
void motor_turn_off_all(){
    int i;
    for(i = 0; i < 16; i++){
        pca_set_pwm(i, 0);
    }
}

//----- PWM Settings.

void motor_set_pwm_freq(int freq) {
    pca_set_frequency(freq);
}

void motor_set_pwm_max(int max) {
    _pwm_max = max;
}

int motor_get_pwm_max() {
    return _pwm_max;
}

void motor_set_pwm_min(int min) {
    _pwm_min = min;
}

int motor_get_pwm_min() {
    return _pwm_min;
}
