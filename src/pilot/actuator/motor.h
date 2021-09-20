/**
 * @file motor.h
 * @author LIN 
 * @brief Motor utilities using PCA9685.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdbool.h>

int motor_init();

int motor_set(int chan, float throttle);

int motor_turn_off(int chan);

void motor_turn_off_all();

//----- PWM Setting.

void motor_set_pwm_freq(int freq);

void motor_set_pwm_max(int max);

int motor_get_pwm_max();

void motor_set_pwm_min(int min);

int motor_get_pwm_min();

#endif // _MOTOR_H_
