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
