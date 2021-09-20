/**
 * @file calibration.h
 * @author LIN 
 * @brief Calibration utilities for AHRS.
 * WIP
 * 
 * 
 * @version 0.1
 * @date 2021-09-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#define CALIBRATION_FILE "Calibration.json"

#include <stdbool.h>

void calibration_gyro(float gx, float gy, float gz, float *ca_gx, float *ca_gy, float *ca_gz);

void calibration_mag(float mx, float my, float mz, float *ca_mx, float *ca_my, float *ca_mz);

void calibration_gather_raw_gyro(float gx, float gy, float gz);

void calibration_gather_raw_mag(float mx, float my, float mz);

void calibration_reset_sample();

void calibration_load();

void calibration_save();

#endif // _CALIBRATION_H_
