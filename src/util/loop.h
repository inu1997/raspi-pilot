/**
 * @file loop.h
 * @author LIN ()
 * @brief Loop control utilities.
 * 
 * 
 *  
 * @version 0.1
 * @date 2021-09-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _LOOP_H_
#define _LOOP_H_

int loop_init(float rate_hz);

void loop_delay_control();

void loop_set_rate(float hz_rate);

float loop_get_rate();

void loop_set_interval(float interval);

float loop_get_interval();

void loop_lock_mutex();

void loop_unlock_mutex();

#endif // _LOOP_H_
