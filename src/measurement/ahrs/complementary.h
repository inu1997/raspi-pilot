/**
 * @file complementary.h
 * @author LIN 
 * @brief Complementary Filter computation.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _COMPLEMENTARY_H_
#define _COMPLEMENTARY_H_

float complementary_filter(float old_att, float acc_att, float omega, float alpha);

void accel_to_attitude(float *r, float *p, float ax, float ay, float az);

float yaw_from_mag(float r, float p, float mx, float my, float mz);

#endif // _COMPLEMENTARY_H_
