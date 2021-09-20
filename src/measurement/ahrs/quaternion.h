/**
 * @file quaternion.h
 * @author LIN 
 * @brief Quaternion utilities.
 * 
 * 
 * 
 * @version 0.1
 * @date 2021-08-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _QUATERNION_H_
#define _QUATERNION_H_

/*
 * @brief Quaternion structure
 */
struct Quaternion {
    float q1; // w
    float q2; // x
    float q3; // y
    float q4; // z
};

void quat_from_euler(struct Quaternion *q, float r, float p, float y);

void quat_to_euler(struct Quaternion *q, float *r, float *p, float *y);

struct Quaternion *quat_sum(struct Quaternion *result, struct Quaternion *a, struct Quaternion *b);

struct Quaternion *quat_sub(struct Quaternion *result, struct Quaternion *a, struct Quaternion *b);

struct Quaternion *quat_mult(struct Quaternion *result, struct Quaternion *a, struct Quaternion *b);

struct Quaternion *quat_rotate(struct Quaternion *result, struct Quaternion *src, struct Quaternion *rot);

struct Quaternion *quat_scale(struct Quaternion *result, struct Quaternion *q, float scale);

struct Quaternion *quat_conjugate(struct Quaternion *result, struct Quaternion *q);

void quat_cpy(struct Quaternion *src, struct Quaternion *dest);

float quat_abs(struct Quaternion *q);

struct Quaternion *quat_normalize(struct Quaternion *result, struct Quaternion *q);

#endif // _QUATERNION_H_
