/**
 * @file char_queue.h
 * @author DeltaDoggo 
 * @brief Used as subscribe/publish design pattern queue.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-09-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _CHAR_QUEUE_H_
#define _CHAR_QUEUE_H_

#include <stdbool.h>

struct CharQueue;

struct CharQueue *cq_init();

void cq_destroy(struct CharQueue *q);

int cq_enqueue(struct CharQueue *q, const char *buf, int len);

int cq_dequeue(struct CharQueue *q, char *buf, int len);

int cq_get_count(struct CharQueue *q);

bool cq_is_full(struct CharQueue *q);

bool cq_is_empty(struct CharQueue *q);


#endif // _CHAR_QUEUE_H_
