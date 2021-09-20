/**
 * @file controller.h
 * @author LIN 
 * @brief Flight Controller header containing all PID set point and maximum function.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _PILOT_H_
#define _PILOT_H_

#include <stdint.h>
#include <stdbool.h>

#define PILOT_AMRED 0x80

// Same as mavlink mode
enum PILOT_MODE{
    PILOT_MODE_PREFLIGHT = 0,
    PILOT_MODE_STABILIZE = 80,
    PILOT_MODE_MANUAL = 64,
    PILOT_MODE_GUIDED = 88,
    PILOT_MODE_AUTO = 92,
    PILOT_MODE_TEST = 66
};

int pilot_init();

void pilot_update();

void pilot_lock_mutex();

void pilot_unlock_mutex();

bool pilot_is_armed();

int pilot_arm();

int pilot_disarm();

void pilot_handle_menual(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t btns);

//----- Setters and Getters.

void pilot_set_mode(int mode);

int pilot_get_mode();

void pilot_set_limit_avz(float lmt);

float pilot_get_limit_avz();

#endif // _PILOT_H_
