/**
 * @file logger.h
 * @author LIN 
 * @brief LOGGER Utilities
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <stdio.h>

#define _COLOR_RED "\033[0;31m"
#define _COLOR_BLUE "\033[0;34m"
#define _COLOR_NONE "\033[1;0m"

#define LOG(format, ...) printf("[%s]: " format, __func__, ##__VA_ARGS__)

#define LOG_ERROR(format, ...) fprintf(stderr, _COLOR_RED "[%s] Error: " format _COLOR_NONE, __func__, ##__VA_ARGS__)

#endif // _LOGGER_H_
