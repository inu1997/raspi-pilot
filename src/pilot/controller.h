/**
 * @file controller.h
 * @author LIN 
 * @brief Compute PID control and update motors/servos. 
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-09-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

extern struct PID *pidsetting_ax; // Attitude X PID.
extern struct PID *pidsetting_ay; // Attitude Y PID.
extern struct PID *pidsetting_az; // Attitude Z PID.
extern struct PID *pidsetting_avx; // Angular Velocity X PID.
extern struct PID *pidsetting_avy; // Angular Velocity Y PID.
extern struct PID *pidsetting_avz; // Angular Velocity Z PID.
extern struct PID *pidsetting_va; // Vertical acceleration PID.
extern struct PID *pidsetting_alt; // Altitude hold PID.

int controller_init();

void controller_update(uint8_t mode, float thr, float avz);

void controller_reset();

#endif // _CONTROLLER_H_
