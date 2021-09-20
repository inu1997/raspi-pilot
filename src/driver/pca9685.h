/**
 * @file pca9685.h
 * @author LIN 
 * @brief Driver for PCA9685 PWM module.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-09-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _PCA9685_H_
#define _PCA9685_H_

int pca_init();

int pca_set_frequency(int freq);

int pca_set_pwm(int chan, int width);

int pca_reset();

#endif // _PCA9685_H_
