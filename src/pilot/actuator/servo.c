#include "servo.h"
#include "driver/pca9685.h"
#include "util/macro.h"

static int _pwm_max;
static int _pwm_min;

int servo_init() {

    return 0;
}

int servo_set_millisecond(int chan, int milli) {
    return 0;
}

int servo_turn_off(int chan) {
    return pca_set_pwm(chan, 0);
}

void servo_turn_off_all() {
    int i;
    for (i = 0; i < 16; i++) {
        pca_set_pwm(i, 0);
    }
}

//----- PWM Setting.

int servo_set_freq(int freq) {

}