/**
 * @file scheduler.h
 * @author LIN 
 * @brief Real Time Linux utilities.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include <pthread.h>

int scheduler_init_real_time();

int scheduler_create_rt_thread(pthread_t *thread, int priority, void *(*func)(), void *arg);

#endif // _SCHEDULER_H_
