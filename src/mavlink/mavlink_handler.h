/**
 * @file mavlink_handler.h
 * @author your name (you@domain.com)
 * @brief My personal mavlink command handler for this project.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-09-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _MAVLINK_HANDLER_H_
#define _MAVLINK_HANDLER_H_

typedef struct __mavlink_message mavlink_message_t;

int mavlink_handler_handle_msg(mavlink_message_t *msg);

const mavlink_message_t *mavlink_handler_get_current_msg();

#endif // _MAVLINK_HANDLER_H_
