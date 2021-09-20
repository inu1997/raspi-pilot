/**
 * @file queue.h
 * @author LIN 
 * @brief Queue data structure.
 * Store data by copying them into buffer array.
 * 
 * 
 * @version 0.1
 * @date 2021-09-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _QUEUE_H_
#define _QUEUE_H_

#include <stdbool.h>

struct Queue;

struct Queue *queue_init(int size);

void queue_destroy(struct Queue *q);

int enqueue(struct Queue *q, void *data);

int dequeue(struct Queue *q, void *data);

int queue_get_size(struct Queue *q);

int queue_get_count(struct Queue *q);

bool queue_is_empty(struct Queue *q);

#endif // _QUEUE_H_
