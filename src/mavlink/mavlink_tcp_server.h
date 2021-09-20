/**
 * @file mavlink_tcp_server.h
 * @author LIN 
 * @brief MAVLink server utilities. 
 * Initiate a server binding to certain port and wait for connection.
 * Incoming connection will be treated as mavlink connection.
 * 
 * @version 0.1
 * @date 2021-09-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _MAVLINK_TCP_SERVER_H_
#define _MAVLINK_TCP_SERVER_H_

int mavlink_init_tcp_server();

#endif // _MAVLINK_TCP_SERVER_H_
