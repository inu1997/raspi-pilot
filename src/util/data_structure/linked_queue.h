/**
 * @file linked_queue.h
 * @author LIN 
 * @brief Linked Queue data structure.
 * Store data by copying them into linked structure.
 * Never full.
 * 
 * @version 0.1
 * @date 2021-09-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _LINKED_QUEUE_H_
#define _LINKED_QUEUE_H_

#include <stdbool.h>

struct LinkedQueue;

struct LinkedQueue *linked_queue_init(int size);

void linked_queue_destroy(struct LinkedQueue *q);

int linked_queue_enqueue(struct LinkedQueue *q, void *data);

int linked_queue_dequeue(struct LinkedQueue *q, void *data);

int linked_queue_get_size(struct LinkedQueue *q);

int linked_queue_get_count(struct LinkedQueue *q);

bool linked_queue_is_empty(struct LinkedQueue *q);

#endif // _LINKED_QUEUE_H_
