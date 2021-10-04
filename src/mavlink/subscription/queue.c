#include "queue.h"
#include <stdlib.h>
#include <string.h>

#define QUEUE_BUFFER_MAX_LENGTH 2048

struct Queue {
    uint8_t buf[QUEUE_BUFFER_MAX_LENGTH];
    int head;
    int tail;
};

/**
 * @brief Initiator of queue.
 * 
 * @return New queue created.
 */
struct Queue *queue_init() {
    struct Queue *q = malloc(sizeof(struct Queue));
    q->head = q->tail = 0;
    return q;
}

/**
 * @brief Destroyer of queue.
 * 
 * @param q 
 *      The queue.
 */
void queue_destroy(struct Queue *q) {
    free(q);
}

/**
 * @brief Reset the queue by resetting the head and tail.
 * 
 * @param q 
 *      The queue.
 */
void queue_reset(struct Queue *q) {
    q->head = q->tail = 0;
}

/**
 * @brief Push data into queue.
 * 
 * @param q 
 *      The queue.
 * @param buf 
 *      buffer.
 * @param len 
 *      length of buffer.
 * @return Number of character pushed into buffer.
 */
int queue_push(struct Queue *q, const uint8_t *buf, int len) {
    int i;
    for (i = 0; (i < len) && !queue_is_full(q); i++) {
        q->buf[q->tail] = buf[i];
        q->tail = (q->tail + 1) % QUEUE_BUFFER_MAX_LENGTH;
    }
    return i;
}

/**
 * @brief Pop data from queue.
 * 
 * @param q 
 *      The queue.
 * @param buf 
 *      Buffer.
 * @param len 
 *      Length of buffer.
 * @return Number of character read into buffer.
 */
int queue_pop(struct Queue *q, uint8_t *buf, int len) {
    int i;
    for (i = 0; (i < len) && !queue_is_empty(q); i++) {
        buf[i] = q->buf[q->head];
        q->head = (q->head + 1) % QUEUE_BUFFER_MAX_LENGTH;
    }
    return i;
}

/**
 * @brief Check if queue is full.
 * 
 * @param q 
 *      The queue.
 * @return True if full else false.
 */
bool queue_is_full(struct Queue *q) {
    return ((q->tail + 1) % QUEUE_BUFFER_MAX_LENGTH) == q->head;
}

/**
 * @brief Check if queue is empty.
 * 
 * @param q 
 *      The queue.
 * @return True if empty else false.
 */
bool queue_is_empty(struct Queue *q) {
    return q->head == q->tail;
}