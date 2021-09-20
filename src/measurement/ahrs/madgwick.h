/**
 * @file madgwick.h
 * @author LIN 
 * @brief Madgwick quaternion filter method.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _MADGWICK_H_
#define _MADGWICK_H_

#include "quaternion.h"

void madgwick_to_euler(struct Quaternion *q, float *r, float *p, float *y);

void madgwick_update_6(struct Quaternion *q, float ax, float ay, float az, float gx, float gy, float gz);

void madgwick_update_9(struct Quaternion *q, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

#endif // _MADGWICK_H_
