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

#define PILOT_AMRED_FLAG 0x80

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

void pilot_handle_menual(
    int16_t x, int16_t y, int16_t z, 
    int16_t r, int16_t s, int16_t t,
    uint16_t btns1, uint16_t btns2);

//----- Setters and Getters.

void pilot_set_mode(int mode);

int pilot_get_mode();

// Range setter/getter.
void pilot_set_thr_max(float max);

float pilot_get_thr_max();

void pilot_set_thr_min(float min);

float pilot_get_thr_min();

void pilot_set_avx_range(float radsec);

float pilot_get_avx_range();

void pilot_set_avy_range(float radsec);

float pilot_get_avy_range();

void pilot_set_avz_range(float radsec);

float pilot_get_avz_range();

// Control parameter setter/getter.
void pilot_set_thr(float thr);

float pilot_get_thr();

void pilot_set_avx(float radsec);

float pilot_get_avx();

void pilot_set_avy(float radsec);

float pilot_get_avy();

void pilot_set_avz(float radsec);

float pilot_get_avz();

bool pilot_heading_is_locked();

void pilot_set_heading(float heading);

float pilot_get_heading();

#endif // _PILOT_H_
