#include "char_queue.h"

#include <stdlib.h>
#include <assert.h>

#define CHAR_QUEUE_MAX_LENGTH 2048

struct CharQueue {
    char buf[CHAR_QUEUE_MAX_LENGTH]; // Queue buffer.
    int head; // Head of queue.
    int tail; // Tail of queue.
};

/**
 * @brief Create a char queue structure.
 * 
 * @return char queue structure.
 */
struct CharQueue *cq_init() {
    struct CharQueue *q = malloc(sizeof(struct CharQueue));
    assert(q != NULL);
    q->head = q->tail = 0;
    return q;
}

/**
 * @brief Destroy a char queue structure.
 * 
 * @param q 
 *      Queue to destroy.
 */
void cq_destroy(struct CharQueue *q) {
    free(q);
}

/**
 * @brief Enqueue a char queue structure.
 * 
 * @param q 
 *      Char queue.
 * @param buf 
 *      Char buffer.
 * @param len 
 *      Length of buffer.
 * @return Number of character push into queue buffer.
 */
int cq_enqueue(struct CharQueue *q, const char *buf, int len) {
    if (cq_is_full(q)) {
        return -1;
    }

    int i;
    for(i = 0; (i < len) && !cq_is_full(q); i++) {
        q->buf[q->tail] = buf[i];
        q->tail = (q->tail + 1) % CHAR_QUEUE_MAX_LENGTH;
    }
    return i;
}

/**
 * @brief Dequeue a char queue structure.
 * 
 * @param q 
 *      Char queue.
 * @param buf 
 *      Char buffer to read.
 * @param len 
 *      Length of buffer.
 * @return Number of character got into the read buffer.
 */
int cq_dequeue(struct CharQueue *q, char *buf, int len) {
    if (cq_is_empty(q)) {
        return -1;
    }

    int i;
    for(i = 0; (i < len) && !cq_is_empty(q); i++) {
        buf[i] = q->buf[q->head];
        q->head = (q->head + 1) % CHAR_QUEUE_MAX_LENGTH;
    }
    return i + 1;
}

/**
 * @brief Get the count of character in queue buffer.
 * 
 * @param q 
 *      Char queue.
 * @return Number of character in queue buffer currently.
 */
int cq_get_count(struct CharQueue *q) {
    int ret = 0;
    if (q->head > q->tail) {
        ret = CHAR_QUEUE_MAX_LENGTH - (q->head - q->tail);
    } else {
        ret = q->tail - q->head;
    }
    return ret;
}

/**
 * @brief Check if queue is full.
 * 
 * @param q 
 *      Char queue.
 * @return True if full else false. 
 */
bool cq_is_full(struct CharQueue *q) {
    return ((q->tail + 1) % CHAR_QUEUE_MAX_LENGTH) == q->head;
}

/**
 * @brief Check if queue is empty.
 * 
 * @param q 
 *      Char queue.
 * @return True if empty else false.
 */
bool cq_is_empty(struct CharQueue *q) {
    return q->head == q->tail;
}