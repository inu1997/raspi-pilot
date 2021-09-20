/**
 * @file altitude.h
 * @author LIN 
 * @brief Barometer module in measurement.
 * WIP
 * 
 * 
 * @version 0.1
 * @date 2021-08-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _BAROMETER_H_
#define _BAROMETER_H_

int barometer_init();

void barometer_update();

float barometer_get_altitude();

float barometer_get_climb_rate();

float barometer_get_pressure();

float barometer_get_pressure_diff();

float barometer_get_temperature();

#endif // _BAROMETER_H_
