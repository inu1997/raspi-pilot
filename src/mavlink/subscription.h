/**
 * @file pubsub.h
 * @author LIN
 * @brief Subscriber and Publisher design pattern.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-09-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _SUBSCRIPTION_H_
#define _SUBSCRIPTION_H_

#include <stdint.h>
#include <stdbool.h>

struct Subscriber;

struct Publisher;

//----- Initiators

struct Publisher *publisher_init();

struct Subscriber *subscriber_init();

void subscriber_destroy(struct Subscriber *sub);

//----- Common


int subscribe(struct Publisher *pub, struct Subscriber *sub);

int unsubscribe(struct Publisher *pub, struct Subscriber *sub);

//----- Poblisher

int publish(struct Publisher *pub, uint8_t *msg, int len);

int publisher_get_subscriber_count(struct Publisher *pub);

//----- Subscriber
int subscriber_get_message_count(struct Subscriber *sub);

bool subscriber_has_message(struct Subscriber *sub);

int subscriber_receive(struct Subscriber *sub, uint8_t *msg, struct Publisher **from_who);

int subscriber_set_on_queue_full(struct Subscriber *sub, void (*on_full)());

int subscriber_set_on_queue_empty(struct Subscriber *sub, void (*on_empty)());

#endif // _SUBSCRIPTION_H_
