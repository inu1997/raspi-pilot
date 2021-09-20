/**
 * @file ahrs.h
 * @author LIN 
 * @brief Attitude & Heading Reference System module in measurement.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _AHRS_H_
#define _AHRS_H_

#include <stdbool.h>

int ahrs_init();

void ahrs_update_9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void ahrs_update_6(float ax, float ay, float az, float gx, float gy, float gz);

float ahrs_get_roll();

float ahrs_get_pitch();

float ahrs_get_yaw_heading();

#endif // _AHRS_H_
