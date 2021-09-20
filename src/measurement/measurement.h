/**
 * @file measurement.h
 * @author LIN 
 * @brief Measurement main module
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _MEASUREMENT_H_
#define _MEASUREMENT_H_

//----- include all module in measurement
#include "ahrs/ahrs.h"
#include "barometer.h"
#include "imu.h"
#include "battery.h"

int measurement_init();

void measurement_update();

#endif // _MEASUREMENT_H_
