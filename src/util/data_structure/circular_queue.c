#include "circular_queue.h"
#include <stdlib.h>
#include <string.h>

struct CircularQueue {
    void **buffer; // Buffer to store data.
    int head; // Head of queue.
    int tail; // Tail of queue.
    int size; // The size of data stored in.
    int capacity; // The capacity of buffer.
};

struct CircularQueue *circular_queue_init(int size, int capacity) {
    struct CircularQueue *q = malloc(sizeof(struct CircularQueue));
    q->head = q->tail = 0;
    q->size = size;
    q->capacity = capacity + 1;
    // Make buffer
    q->buffer = malloc(sizeof(void*) * q->capacity);
    int i;
    for (i = 0; i < q->capacity; i++) {
        // Make space for every buffer element.
        q->buffer[i] = malloc(q->size);
    }
    return q;
}

void circular_queue_destroy(struct CircularQueue *q) {
    // Destroy every element of buffer.
    int i;
    for (i = 0; i < q->capacity; i++) {
        free(q->buffer[i]);
    }
    // Destroy buffer.
    free(q->buffer);
    // Destroy queue.
    free(q);
}

int circular_queue_enqueue(struct CircularQueue *q, void *data) {
    if (circular_queue_is_full(q)) {
        return -1;
    }
    memcpy(q->buffer[q->tail], data, q->size);
    q->tail = (q->tail + 1) % q->capacity;
    return 0;
}

int circular_queue_dequeue(struct CircularQueue *q, void *data) {
    if (circular_queue_is_empty(q)) {
        return -1;
    }
    memcpy(data, q->buffer[q->head], q->size);
    q->head = (q->head + 1) % q->capacity;
    return 0;
}

int circular_queue_get_size(struct CircularQueue *q) {
    return q->size;
}

int circular_queue_get_capacity(struct CircularQueue *q) {
    return q->capacity - 1;
}

int circular_queue_get_count(struct CircularQueue *q) {
    int ret = 0;
    if (q->head > q->tail) {
        ret = q->capacity - (q->head - q->tail);
    } else {
        ret = q->tail - q->head;
    }
    return ret;
}

bool circular_queue_is_empty(struct CircularQueue *q) {
    return q->head == q->tail;
}

bool circular_queue_is_full(struct CircularQueue *q) {
    return ((q->tail + 1) % q->capacity) == q->head;
}