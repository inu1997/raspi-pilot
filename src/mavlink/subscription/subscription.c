#include "subscription.h"
#include "queue.h"
#include <pthread.h>
#include "mavlink/c_library_v2/standard/mavlink.h"

// Subscriptor structure.
struct Subscriptor {
    bool active;
    struct Queue *q;
    pthread_mutex_t mutex;
};

static struct Subscriptor subs[MAVLINK_COMM_NUM_BUFFERS]; // Subscriptor array of all subscriptors.

#define NUMBER_OF_SUBSCRIPTOR (sizeof(subs) / sizeof(struct Subscriptor))

/**
 * @brief Initiator of subscription.
 * 
 * @return 0 if success else -1.
 */
int subscription_init() {
    int i;
    for (i = 0; i < NUMBER_OF_SUBSCRIPTOR; i++) {
        subs[i].active = false;
        subs[i].q = queue_init();
        pthread_mutex_init(&subs[i].mutex, NULL);
    }
    return 0;
}

/**
 * @brief Publish  message to subscriptors.
 * 
 * @param buf 
 *      Buffer.
 * @param len 
 *      Length of buffer.
 * @return Number of subscriptor received the message.
 */
int publish(uint8_t *buf, int len) {
    int cnt = 0;

    int i;
    for (i = 0; i < NUMBER_OF_SUBSCRIPTOR; i++) {
        if (subs[i].active) {
            pthread_mutex_lock(&subs[i].mutex);
            
            queue_push(subs[i].q, buf, len);
            
            pthread_mutex_unlock(&subs[i].mutex);
            cnt++;
        }
    }
    
    return cnt;
}

/**
 * @brief Read the subscriptor's message to buffer.
 * 
 * @param index 
 *      Index of subscriptor.
 * @param buf 
 *      Buffer.
 * @param len 
 *      Length of buffer.
 * @return Number of character read into buffer.
 */
int subscriber_read(int index, uint8_t *buf, int len) {
    int ret;
    
    pthread_mutex_lock(&subs[index].mutex);
    
    ret = queue_pop(subs[index].q, buf, len);
    
    pthread_mutex_unlock(&subs[index].mutex);

    return ret;
}

/**
 * @brief Reset the buffer
 * 
 * @param index 
 *      Index of subscriptor.
 */
void subscriber_reset(int index) {

    pthread_mutex_lock(&subs[index].mutex);
    
    subs[index].active = false;
    queue_reset(subs[index].q);
    
    pthread_mutex_unlock(&subs[index].mutex);
}

/**
 * @brief Set active of subscriptor.
 * 
 * @param index 
 *      Index of subscriptor.
 * @param active 
 *      Active.
 * @return 0 if success else -1.
 */
int subscriber_set_active(int index, bool active) {
    int ret = -1;
    pthread_mutex_lock(&subs[index].mutex);
    
    if (subs[index].active != active) {
        subs[index].active = active;
        ret = 0;
    }
    
    pthread_mutex_unlock(&subs[index].mutex);
    return ret;
}

bool subscriber_available(int index) {
    bool ret = false;

    pthread_mutex_lock(&subs[index].mutex);
    
    ret = !queue_is_empty(subs[index].q);
    
    pthread_mutex_unlock(&subs[index].mutex);
    
    return ret;
}