/**
 * @file pwm.h
 * @author LIN 
 * @brief PWM with min/max utility.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-09-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _PWM_H_
#define _PWM_H_

int pwm_init();

int pwm_set_width(int chan, int width);

int pwm_turn_off(int chan);

void pwm_turn_off_all();

//----- PWM Setting.

int pwm_set_freq(int freq);

void pwm_set_max(int max);

int pwm_get_max();

void pwm_set_min(int min);

int pwm_get_min();

#endif // _PWM_H_
