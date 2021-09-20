#include "quaternion.h"
#include <math.h>

/**
 * @brief Convert euler angles in Degree to quaternion.
 * @param q
 *      Destination of converted result.
 * @param r
 *      Roll in radian.
 * @param p
 *      Pitch in radian.
 * @param y
 *      Yaw in radian.
 */
void quat_from_euler(struct Quaternion *q, float r, float p, float y){
    float cr = cos(r * 0.5);
    float sr = sin(r * 0.5);
    float cp = cos(p * 0.5);
    float sp = sin(p * 0.5);
    float cy = cos(y * 0.5);
    float sy = sin(y * 0.5);

    q->q1 = cr * cp * cy + sr * sp * sy;
    q->q2 = sr * cp * cy - cr * sp * sy;
    q->q3 = cr * sp * cy + sr * cp * sy;
    q->q4 = cr * cp * sy - sr * sp * cy;
}

/**
 * @brief Convert quaternion to euler angles in Radian.
 * @param q
 *      The quaternion
 * @param r
 *      Destnation of result roll. Unit: Radian
 * @param p
 *      Destnation of result pitch. Unit: Radian
 * @param y
 *      Destnation of result yaw. Unit: Radian
 */
void quat_to_euler(struct Quaternion *q, float *r, float *p, float *y){
    // Roll
    *r = atan2f(
        2 * (q->q1 * q->q2 + q->q3 * q->q4),
        1 - 2 * (q->q2 * q->q2 + q->q3 * q->q3)
    );

    // Pitch
    *p = asinf(2 * (q->q1 * q->q3 - q->q2 * q->q4));

    // Yaw
    *y = atan2f(
        2 * (q->q1 * q->q4 + q->q2 * q->q3),
        1 - 2 * (q->q3 * q->q3 + q->q4 * q->q4)
    );
}

/**
 * @brief Quaternion sum.
 * @param result
 *      Destination of result. result = a + b
 * @param a
 *      Operand a.
 * @param b
 *      Operand b.
 * @return result.
 */
struct Quaternion *quat_sum(struct Quaternion *result, struct Quaternion *a, struct Quaternion *b){
    result->q1 = a->q1 + b->q1;
    result->q2 = a->q2 + b->q2;
    result->q3 = a->q3 + b->q3;
    result->q4 = a->q4 + b->q4;
    
    return result;
}

/**
 * @brief Quaternion subtration.
 * @param result
 *      Destination of result. result = a - b
 * @param a
 *      Operand a.
 * @param b
 *      Operand b.
 * @return result.
 */
struct Quaternion *quat_sub(struct Quaternion *result, struct Quaternion *a, struct Quaternion *b){
    result->q1 = a->q1 - b->q1;
    result->q2 = a->q2 - b->q2;
    result->q3 = a->q3 - b->q3;
    result->q4 = a->q4 - b->q4;
    
    return result;
}

/**
 * @brief Quaternion multiplication.
 * @param result
 *      Destination of result. result = a * b
 * @param a
 *      Operand a.
 * @param b
 *      Operand b.
 * @return result.
 */
struct Quaternion *quat_mult(struct Quaternion *result, struct Quaternion *a, struct Quaternion *b){
    float w, x, y, z;
    w = (a->q1 * b->q1) - (a->q2 * b->q2) - (a->q3 * b->q3) - (a->q4 * b->q4);
    x = (a->q1 * b->q2) + (a->q2 * b->q1) + (a->q3 * b->q4) - (a->q4 * b->q3);
    y = (a->q1 * b->q3) - (a->q2 * b->q4) + (a->q3 * b->q1) + (a->q4 * b->q2);
    z = (a->q1 * b->q4) + (a->q2 * b->q3) - (a->q3 * b->q2) + (a->q4 * b->q1);
    result->q1 = w;
    result->q2 = x;
    result->q3 = y;
    result->q4 = z;

    return result;
}

/**
 * @brief Quaternion rotate.
 * @param result
 *      Destination of resulf, result = rot * src * (rot^-1)
 * @param src
 *      Target to rotate.
 * @param rot
 *      Rotate quaternion.
 * @return result
 */
struct Quaternion *quat_rotate(struct Quaternion *result, struct Quaternion *src, struct Quaternion *rot){
    struct Quaternion rot_conj = {0};
    struct Quaternion tmp = {0};
    quat_conjugate(&rot_conj, rot);
    quat_mult(&tmp, rot, src);
    quat_mult(result, &tmp, &rot_conj);
    
    return result;
}

/**
 * @brief Quaternion scale.
 * @param result
 *      Destination of result. result = q * scalar
 * @param q
 *      Target to scale.
 * @param scalar
 *      The scalar.
 */
struct Quaternion *quat_scale(struct Quaternion *result, struct Quaternion *q, float scalar){
    result->q1 = q->q1 * scalar;
    result->q2 = q->q2 * scalar;
    result->q3 = q->q3 * scalar;
    result->q4 = q->q4 * scalar;
    
    return result;
}

/**
 * @brief Quaternion conjugate.
 * @param result
 *      Destination of result. result = q* = q ^ -1
 * @param q
 *      Target to conjugate.
 * @return result.
 */
struct Quaternion *quat_conjugate(struct Quaternion *result, struct Quaternion *q){
    result->q1 = q->q1;
    result->q2 = -q->q2;
    result->q3 = -q->q3;
    result->q4 = -q->q4;
    
    return result;
}
/**
 * @brief Quaternion copy function.
 * @param src
 *      The to be copied.
 * @param dest
 *      The to be writen.
 */
void quat_cpy(struct Quaternion *src, struct Quaternion *dest){
    dest->q1 = src->q1;
    dest->q2 = src->q2;
    dest->q3 = src->q3;
    dest->q4 = src->q4;
}

/**
 * @brief Quaternion absolute value.
 * @param q
 *      Get the absolute value of q.
 * @return absolute value of q.
 */
float quat_abs(struct Quaternion *q){
    return sqrtf(
        q->q1 * q->q1 +
        q->q2 * q->q2 +
        q->q3 * q->q3 +
        q->q4 * q->q4
    );
}

/**
 * @brief Normalize quaternion.
 * @param result
 *      The destination of result. result = q / abs(q)
 * @param q
 *      Target to normalize.
 * @return result.
 */
struct Quaternion *quat_normalize(struct Quaternion *result, struct Quaternion *q){
    float norm = quat_abs(q);
    result->q1 = q->q1 / norm;
    result->q2 = q->q2 / norm;
    result->q3 = q->q3 / norm;
    result->q4 = q->q4 / norm;
    
    return result;
}