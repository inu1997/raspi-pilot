/**
 * @file servo.h
 * @author LIN 
 * @brief Servo utilitiy using PCA9685. 
 * WIP.
 * 
 * 
 * @version 0.1
 * @date 2021-09-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _SERVO_H_
#define _SERVO_H_

int servo_init();

int servo_set_millisecond(int chan, int milli);

int servo_turn_off(int chan);

void servo_turn_off_all();

//----- PWM Setting.

int servo_set_freq(int freq);

#endif // _SERVO_H_
