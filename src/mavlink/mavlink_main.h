/**
 * @file mavlink_main.h
 * @author LIN
 * @brief My personal mavlink message wrapper for this project.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _MAVLINK_MAIN_H_
#define _MAVLINK_MAIN_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct __mavlink_message mavlink_message_t;

#define MAVLINK_SEND(msg) do { \
    char buf[256]; \
    int len = mavlink_msg_to_send_buffer(buf, msg); \
    mavlink_publish(buf, len); \
} while(0)

struct MAVLinkFile {
    char name[16]; // Name of connection.
    int fd; // File Descriptor for R/W.
    int mav_channel; // mavlink channel to use.
    void (*on_begin)(); // Things to do on begin.
    void (*on_end)(); // Things to do on end.
    bool exit_on_error;
};

#define MAVLINK_SYS_ID 1
extern struct Publisher *mavlink_publisher;

int mavlink_init();

int mavlink_send(mavlink_message_t *msg);

int mavlink_publish(uint8_t *buf, int len);

void *mavlink_connection_handler(void *mavlink_file);

int mavlink_get_active_connections();

void mavlink_on_connection_active();

void mavlink_on_connection_inactive();
//----- Utility

int mavlink_ocupy_usable_channel();

void mavlink_release_channel(int chan);

bool mavlink_is_channel_occupied(int chan);

#endif // _MAVLINK_MAIN_H_
