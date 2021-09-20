/**
 * @file smafilter.h
 * @author LIN 
 * @brief Simple Moving Average filter utilities.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _SMA_FILTER_H_
#define _SMA_FILTER_H_

#include <stdbool.h>

struct SMAFilter;

struct SMAFilter *sma_init(int length);

float sma_update(struct SMAFilter *sma, float v);

void sma_reset(struct SMAFilter *sma);

void sma_set_length(struct SMAFilter *sma, int length);

int sma_get_length(struct SMAFilter *sma);

bool sma_buffer_has_cycled(struct SMAFilter *sma);

#endif // _SMA_FILTER_H_
