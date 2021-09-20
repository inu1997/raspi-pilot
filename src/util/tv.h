/**
 * @file tv.h
 * @author LIN 
 * @brief Timeval Utilities
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _TV_H_
#define _TV_H_

#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>

#define TV_INITIALIZER {.tv_sec = 0, .tv_usec = 0}

bool tv_is_updated(struct timeval *tv);

uint64_t tv_get_msec_since_epoch();

void tv_sleep_sec(int sec);

void tv_sleep_usec(int usec);

void tv_wait_sec(struct timeval *last_tv, int sec);

void tv_wait_usec(struct timeval *last_tv, int usec);

float tv_get_diff_sec_f(struct timeval *tv, struct timeval *now);

unsigned long tv_get_diff_sec_ul(struct timeval *tv, struct timeval *now);

unsigned long tv_get_diff_msec_ul(struct timeval *tv, struct timeval *now);

unsigned long tv_get_diff_usec_ul(struct timeval *tv, struct timeval *now);

#endif // _TV_H_
