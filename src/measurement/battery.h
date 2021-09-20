/**
 * @file battery.h
 * @author LIN 
 * @brief Measuring battery voltage/current and comsumed current.
 * WIP.
 * 
 * 
 * @version 0.1
 * @date 2021-09-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _BATTERY_H_
#define _BATTERY_H_

int battery_init();

void battery_update();

float battery_get_voltage();

float battery_get_current();

float battery_get_comsumed_current();

float battery_get_remain_time_sec();

#endif // _BATTERY_H_
