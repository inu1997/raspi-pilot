#ifndef _QUEUE_H_
#define _QUEUE_H_

#include <stdint.h>
#include <stdbool.h>

struct Queue;

struct Queue *queue_init();

void queue_destroy(struct Queue *q);

void queue_reset(struct Queue *q);

int queue_push(struct Queue *q, const uint8_t *buf, int len);

int queue_pop(struct Queue *q, uint8_t *buf, int len);

bool queue_is_full(struct Queue *q);

bool queue_is_empty(struct Queue *q);

#endif // _QUEUE_H_
