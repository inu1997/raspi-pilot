/**
 * @file signal_hanlder.h
 * @author your name (you@domain.com)
 * @brief Signal Handler Utilities.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-09-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _SIGNAL_HANLDER_H_
#define _SIGNAL_HANLDER_H_

int signal_handler_init();

void signal_set_on_sigint(void (*func)());

void signal_set_on_sigpipe(void (*func)());

#endif // _SIGNAL_HANLDER_H_
