#include "subscription.h"

#include "util/data_structure/char_queue.h"
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
    bool active;
    struct CharQueue *queue; // The queue buffer.
    pthread_mutex_t mutex; // So it won't be messed up in multi-thread.
};

int subscriber_send(struct Subscriber *sub, const char *buf, int len);

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
    sub->queue = cq_init();
    pthread_mutex_init(&sub->mutex, NULL);
    sub->active = false;
    return sub;
}

void subscriber_destroy(struct Subscriber *sub) {
    cq_destroy(sub->queue);
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
int publish(struct Publisher *pub, const char *buf, int len) {
    pthread_mutex_lock(&pub->mutex);
    
    int i = 0;
    struct Subscriber *sub;
    while((sub = list_iterate(pub->subs, i)) != NULL) {
        if (sub->active) {
            subscriber_send(list_iterate(pub->subs, i), buf, len);
        }
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

    int cnt = cq_get_count(sub->queue);

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
    
    bool ret = !cq_is_empty(sub->queue);
    
    pthread_mutex_unlock(&sub->mutex);
    return ret;
}


/**
 * @brief Pop message from queue.
 * 
 * @param sub
 *      The subscriber.
 * @param buf
 *      Buffer to read.
 * @param len
 *      Length of max read count.
 * @return Length of message, -1 if fail.
 */
int subscriber_receive(struct Subscriber *sub, char *buf, int len) {
    pthread_mutex_lock(&sub->mutex);
    
    int ret = cq_dequeue(sub->queue, buf, len);

    pthread_mutex_unlock(&sub->mutex);
    return ret;
}

//-----

/**
 * @brief Update a subscriber. 
 * 
 * @param sub 
 *      Subscriber.
 * @param buf
 *      Buffer to publish.
 * @param len
 *      Length of buffer.
 * @return 0 if success else -1. 
 */
int subscriber_send(struct Subscriber *sub, const char *buf, int len) {
    pthread_mutex_lock(&sub->mutex);

    int ret = cq_enqueue(sub->queue, buf, len);
    if (ret != len) {
        LOG_ERROR("Queue is full, message probably lost.(ret: %d, len: %d)", ret, len);
    }
    pthread_mutex_unlock(&sub->mutex);
    return ret;
}

int subscriber_set_active(struct Subscriber *sub, bool active) {
    int ret = -1;
    pthread_mutex_lock(&sub->mutex);
    if (sub->active != active) {
        sub->active = active;
        ret = 0;
    }
    pthread_mutex_unlock(&sub->mutex);
    return ret;
}