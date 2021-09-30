#include "servo.h"
#include "driver/pca9685.h"
#include "util/macro.h"

int _freq; // PCA frequency cach.

int servo_set_millisecond(int chan, int milli) {
    return 0;
}

int servo_turn_off(int chan) {
    return pca_set_pwm(chan, 0);
}

//----- PWM Setting.

void servo_set_freq(int freq) {
    _freq = freq;
    pca_set_frequency(freq);
    // Compute min max.
}