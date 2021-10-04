/**
 * @file subscription.h
 * @author DeltaDoggo 
 * @brief Subscriptor/Publisher design pattern for multi-thread MAVLink W/R.
 * Create N subscriptors for processing the send messages. (N equals the maximum number of MAVLink Channel.)
 * Index could be the channel of the mavlink thread is using.
 * 
 * @version 0.1
 * @date 2021-10-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _SUBSCRIPTION_H_
#define _SUBSCRIPTION_H_

#include <stdbool.h>
#include <stdint.h>

int subscription_init();

int publish(uint8_t *buf, int len);

int subscriber_read(int index, uint8_t *buf, int len);

void subscriber_reset(int index);

int subscriber_set_active(int index, bool active);

bool subscriber_available(int index);

#endif // _SUBSCRIPTION_H_
