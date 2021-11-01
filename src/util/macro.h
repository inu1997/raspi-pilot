/**
 * @file macro.h
 * @author LIN 
 * @brief Pre-defined macros for easier coding.
 * 
 * 
 * 
 * @version 0.2
 * @date 2021-11-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _MACRO_H_
#define _MACRO_H_

#include <math.h>

#define PI 3.14159265358979323846

#define GET_VAR_NAME(var) #var

#define BIT(i) (0x01 << i) 

#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define MIN(a, b) ((a) > (b) ? (b) : (a))

#define ABS(a) ( \
    (a) > 0? (a) : \
    (a) < 0? -(a) : \
    (a) \
)

#define LIMIT_MAX_MIN(V, MAX, MIN) ( \
    (V) > (MAX) ? (MAX) : \
    (V) < (MIN) ? (MIN) : \
    (V) \
)

#define TO_DEG  (180.0f / PI)

#define TO_RAD  (PI / 180.0f)

#define RAD_TO_DEG(rad) ((rad) * TO_DEG)

#define DEG_TO_RAD(deg) ((deg) * TO_RAD)

#define ANGLE_LIMIT_DEG(a) ( \
    (a) > 180.0f ? fmodf(a, 360.0f) - 360.0f : \
    (a) < -180.0f ? fmodf(a, 360.0f) + 360.0f : \
    (a) \
)

#define GET_MIN_INCLUDED_ANGLE(from, to) ANGLE_LIMIT_DEG(ANGLE_LIMIT_DEG(to) - ANGLE_LIMIT_DEG(from))

#define DEADBAND(value, threshold) ( \
    (value) >= (threshold) ? (value) : \
    (value) <= -(threshold) ? (value) : \
    0 \
) 
#define DEADBAND_OFFSET(value, threshold, offset) ( \
    ((value) - (offset)) >= (threshold) ? (value) : \
    ((value) - (offset)) <= -(threshold) ? (value) : \
    (offset) \
)

/**
 * @brief MAP a value from "range from" to "range to".
 * 
 */
#define MAP(value, from_start, from_end, to_start, to_end) ( \
    (to_start) + (((value) - (from_start)) / ((from_end) - (from_start))) * ((to_end) - (to_start)) \
)

#define SQUARE(a) ((a) * (a))

#define STR_MATCH(a, b) (strcmp((a), (b)) == 0)

#endif // _MACRO_H_
