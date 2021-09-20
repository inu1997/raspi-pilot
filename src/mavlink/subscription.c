#include "subscription.h"

#include "util/data_structure/queue.h"
#include "util/data_structure/list.h"

#include "util/logger.h"
#include "util/debug.h"

#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#define MAX_MESSAGE_LENGTH 256

struct Publisher {
    struct List *subs; // List of subscribers.
    pthread_mutex_t mutex; // So it won't be messed up in multi-thread.
};

struct Subscriber{
    struct Queue *queue; // The queue buffer.
    pthread_mutex_t mutex; // So it won't be messed up in multi-thread.
};

struct Message {
    struct Publisher *from_who;
    uint8_t buf[MAX_MESSAGE_LENGTH];
    uint8_t len;
};

int subscriber_send(struct Subscriber *sub, uint8_t *msg, int len, struct Publisher *from_who);

//----- Initiators

/**
 * @brief Initialize a publisher.
 * 
 * @return Newly created publisher.
 */
struct Publisher *publisher_init() {
    struct Publisher *pub = malloc(sizeof(struct Publisher));
    pub->subs = list_init();
    pthread_mutex_init(&pub->mutex, NULL);
    return pub;
}

/**
 * @brief Initialize a subscriber.
 * 
 * @return Newly created subscriber.
 */
struct Subscriber *subscriber_init() {
    struct Subscriber *sub = malloc(sizeof(struct Subscriber));
    sub->queue = queue_init(sizeof(struct Message));
    pthread_mutex_init(&sub->mutex, NULL);
    return sub;
}

void subscriber_destroy(struct Subscriber *sub) {
    queue_destroy(sub->queue);
    pthread_mutex_destroy(&sub->mutex);
    free(sub);
}

//----- Common

/**
 * @brief Connect subscrber to publisher.
 * 
 * @param pub 
 *      The publisher.
 * @param sub 
 *      The subscriber.
 * @return The number of subscribers of publisher after subscribe on success else -1.
 */
int subscribe(struct Publisher *pub, struct Subscriber *sub) {
    int ret;
    pthread_mutex_lock(&pub->mutex);

    DEBUG("Locked mutex.\n");
    if (list_find(pub->subs, sub) != -1) {
        // Subscriber already exist.
        LOG_ERROR("Subscriber already exist in Publisher.\n");
        ret = -1;
    } else {
        ret = list_add(pub->subs, sub);
        DEBUG("Added sunscriber now has %d subscribers.\n", ret);
    }

    pthread_mutex_unlock(&pub->mutex);
    return ret;
}

/**
 * @brief Disconnect subscriber from publisher.
 * 
 * @param pub
 *      The publisher.
 * @param sub
 *      The subscriber.
 * @return 0 if success else -1.
 */
int unsubscribe(struct Publisher *pub, struct Subscriber *sub) {
    int ret;
    pthread_mutex_lock(&pub->mutex);

    ret = list_remove(pub->subs, sub);

    DEBUG("Remove of subscriber returns: %d, Remaining: %d.\n", ret, list_get_count(pub->subs));
    pthread_mutex_unlock(&pub->mutex);
    return ret;
}

//----- Publisher

/**
 * @brief Publish message to subscribers of publisher.
 * 
 * @param pub
 *      The publisher.
 * @param msg
 *      The message.
 * @return The number of subscriber published to.
 */
int publish(struct Publisher *pub, uint8_t *msg, int len) {
    pthread_mutex_lock(&pub->mutex);
    
    int i = 0;
    while(list_iterate(pub->subs, i) != NULL) {
        subscriber_send(list_iterate(pub->subs, i), msg, len, pub);
        i++;
    }

    pthread_mutex_unlock(&pub->mutex);
    return i;
}

/**
 * @brief Get the number of subscribers of publisher..
 * 
 * @param pub
 *      The publisher.
 * @return The number of subscribers.
 */
int publisher_get_subscriber_count(struct Publisher *pub) {
    pthread_mutex_lock(&pub->mutex);

    int cnt = list_get_count(pub->subs);

    pthread_mutex_unlock(&pub->mutex);
    return cnt;
}



//----- Subscriber

/**
 * @brief Get the number of message incoming.
 * 
 * @param sub
 *      The subscriber.
 * @return -1 if no message. Else the number of messages. 
 */
int subscriber_get_message_count(struct Subscriber *sub) {
    pthread_mutex_lock(&sub->mutex);

    int cnt = queue_get_count(sub->queue);

    pthread_mutex_unlock(&sub->mutex);
    return cnt;
}

/**
 * @brief Check if there's message in queue..
 * 
 * @param sub
 *      The subscriber.
 * @return True if there's message in queue else false. 
 */
bool subscriber_has_message(struct Subscriber *sub) {
    pthread_mutex_lock(&sub->mutex);
    
    bool ret = !queue_is_empty(sub->queue);
    
    pthread_mutex_unlock(&sub->mutex);
    return ret;
}


/**
 * @brief Pop message from queue.
 * 
 * @param sub
 *      The subscriber.
 * @param msg
 *      Message to get.
 * @return Length of message, -1 if fail.
 */
int subscriber_receive(struct Subscriber *sub, uint8_t *msg, struct Publisher **from_who) {
    pthread_mutex_lock(&sub->mutex);
    
    struct Message m;
    int ret = -1;

    if (dequeue(sub->queue, &m) != -1) {
        ret = m.len;
        memcpy(msg, m.buf, m.len);
        
        if (from_who != NULL) {
            *from_who = m.from_who;
        }
    }

    pthread_mutex_unlock(&sub->mutex);
    return ret;
}

//-----

/**
 * @brief Update a subscriber. 
 * 
 * @param sub 
 *      Subscriber.
 * @param msg
 *      Message struct to push to queue.
 * @return 0 if success else -1. 
 */
int subscriber_send(struct Subscriber *sub, uint8_t *msg, int len, struct Publisher *from_who) {
    pthread_mutex_lock(&sub->mutex);

    struct Message m;
    m.from_who = from_who;
    m.len = len;
    memcpy(m.buf, msg, len);

    int ret = enqueue(sub->queue, &m);

    pthread_mutex_unlock(&sub->mutex);
    return ret;
}