/**
 * @file kalmanfilter1d.h
 * @author LIN 
 * @brief Kalman Filter for 1 dimension.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _KALMAN_FILTER_1D_H_
#define _KALMAN_FILTER_1D_H_

struct Kalman1D;

struct Kalman1D *kf_one_init(float prev_x, float p, float q, float r);

float kf_one_update(struct Kalman1D *kf, float x);

#endif // _KALMAN_FILTER_1D_H_
