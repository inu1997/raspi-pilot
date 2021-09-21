/**
 * @file circular_queue.h
 * @author DeltaDoggo 
 * @brief Circular Queue data structure.
 * Might prevent memory overflow.
 * 
 * 
 * @version 0.1
 * @date 2021-09-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _CIRCULAR_QUEUE_H_
#define _CIRCULAR_QUEUE_H_

#include <stdbool.h>

struct CircularQueue;

struct CircularQueue *circular_queue_init(int size, int capacity);

void circular_queue_destroy(struct CircularQueue *q);

int circular_queue_enqueue(struct CircularQueue *q, void *data);

int circular_queue_dequeue(struct CircularQueue *q, void *data);

int circular_queue_get_size(struct CircularQueue *q);

int circular_queue_get_capacity(struct CircularQueue *q);

int circular_queue_get_count(struct CircularQueue *q);

bool circular_queue_is_empty(struct CircularQueue *q);

bool circular_queue_is_full(struct CircularQueue *q);

#endif // _CIRCULAR_QUEUE_H_
