#include "motor.h"
#include "servo.h"

#include "driver/pca9685.h"

#include "util/logger.h"
#include "util/macro.h"
#include "util/debug.h"
#include "util/parameter.h"

#include <stdlib.h>

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
    int freq;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MTR_PWM_FREQ], &freq);
    motor_set_pwm_freq(freq);
    
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
        pca_set_pwm(chan, 
            LIMIT_MAX_MIN((int)4095.0 * throttle / 100.0,
                4095, 0));
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
    servo_set_freq(freq);
}
