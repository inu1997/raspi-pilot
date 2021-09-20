/**
 * @file pid.h
 * @author LIN 
 * @brief PID control utilities.
 * @version 0.1
 * @date 2021-08-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _PID_H_
#define _PID_H_

#define PIDSETTING_PATH "PIDSettings.json"

struct PID;

struct PID *pid_init();

struct PID *pid_init_param(float p, float i, float d, float err_sum_limit, float output_limit);

float pid_update(struct PID *pid, float sp, float pv);

void pid_reset(struct PID *pid);

//----- Setter and Getters

void pid_tune(struct PID *pid, float p, float i, float d, float err_sum_limit, float output_limit);

void pid_set_p(struct PID *pid, float v);

float pid_get_p(struct PID *pid);

void pid_set_i(struct PID *pid, float v);

float pid_get_i(struct PID *pid);

void pid_set_d(struct PID *pid, float v);

float pid_get_d(struct PID *pid);

void pid_set_err_sum_limit(struct PID *pid, float v);

float pid_get_err_sum_limit(struct PID *pid);

void pid_set_output_limit(struct PID *pid, float v);

float pid_get_output_limit(struct PID *pid);

float pid_get_output(struct PID *pid);

#endif // _PID_H_
