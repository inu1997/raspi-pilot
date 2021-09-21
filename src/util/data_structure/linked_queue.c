#include "linked_queue.h"
#include "util/data_structure/list.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct Node {
    void *obj;
    struct Node *next;
};

struct LinkedQueue {
    struct Node *head;
    struct Node *tail;
    int size; // Size of data in n bytes.
};

/**
 * @brief Initiator of queue structure.
 * 
 * @param size
 *      Size of data(in bytes) to be stored in queue.
 * @return Address of newly created queue.
 */
struct LinkedQueue *linked_queue_init(int size) {
    struct LinkedQueue *q = malloc(sizeof(struct LinkedQueue));
    q->head = q->tail = malloc(sizeof(struct Node));
    q->head->obj = q->head->next = NULL;
    q->size = size;
    return q;
}

/**
 * @brief Destroyer of queue.
 * 
 * @param q
 *      Queue to destroy.
 */
void linked_queue_destroy(struct LinkedQueue *q) {
    int i = 0;
    struct Node *tmp;
    while(q->head != q->tail) {
        // Free a object.
        free(q->head->obj);
        // Move on and store previous head to tmp.
        tmp = q->head;
        q->head = q->head->next;
        // Free previous head.
        free(tmp);
    }
    free(q->head);
    free(q);
}

/**
 * @brief Put data into queue.
 * 
 * @param q
 *      Queue to manipulate.
 * @param data
 *      Data to store.
 * @return 0 if success else -1.
 */
int linked_queue_enqueue(struct LinkedQueue *q, void *data) {
    // Malloc object and store.
    q->tail->obj = malloc(q->size);
    memcpy(q->tail->obj, data, q->size);

    // Create new node.
    struct Node *new_node = malloc(sizeof(struct Node));
    new_node->next = new_node->obj = NULL;
    
    // Link to the new node and move on.
    q->tail->next = new_node;
    q->tail = q->tail->next;
    
    return 0;
}

/**
 * @brief Get data from queue.
 * 
 * @param q
 *      Queue to manipulate.
 * @param data
 *      Data catcher.
 * @return 0 if success else -1.
 */
int linked_queue_dequeue(struct LinkedQueue *q, void *data) {
    if (linked_queue_is_empty(q)) {
        return -1;
    }
    // Copy object to data.
    memcpy(data, q->head->obj, q->size);
    
    // Destroy old node and move on.
    free(q->head->obj);
    struct Node *tmp = q->head;
    q->head = q->head->next;
    free(tmp);

    return 0;
}

/**
 * @brief Getter of queue's size.
 * 
 * @param q
 *      Queue
 * @return Size of data size of queue. 
 */
int linked_queue_get_size(struct LinkedQueue *q) {
    return q->size;
}


/**
 * @brief Getter of queue's data count.
 * 
 * @param q
 *      Queue
 * @return Count of data of queue.
 */
int linked_queue_get_count(struct LinkedQueue *q) {
    int cnt = 0;
    struct Node *tmp = q->head;
    while (tmp != q->tail) {
        tmp = tmp->next;
        cnt++;
    }
    return cnt;
}

/**
 * @brief Check if queue is empty.
 * 
 * @param q
 *      Queue
 * @return True if queue is empty else false. 
 */
bool linked_queue_is_empty(struct LinkedQueue *q) {
    return (q->head == q->tail);
}