#include "madgwick.h"
#include "util/loop.h"
#include <stdio.h>
#include <math.h>

/**
 * @brief Definition to dt
 */
#define DELTA_TIME loop_get_interval()

/**
 * @brief BETA in madgwick calculation
 */
#define BETA 0.5

//-----

/**
 * @brief Quaternion to Euler function in Madgwick's paper.
 * 
 * @param q 
 *      The quaternion.
 * @param r 
 *      Roll in radian.
 * @param p 
 *      Pitch in radian.
 * @param y 
 *      Yaw in radian.
 */
void madgwick_to_euler(struct Quaternion *q, float *r, float *p, float *y){
    // Roll
    *r = atan2f(
        2 * (q->q3 * q->q4 - q->q1 * q->q2),
        2 * (q->q1 * q->q1 + q->q4 * q->q4) - 1
    );

    // Pitch
    *p = -asinf(2 * (q->q1 * q->q3 + q->q2 * q->q4));

    // Yaw
    *y = atan2f(
        2 * (q->q2 * q->q3 - q->q1 * q->q4),
        2 * (q->q1 * q->q1 + q->q2 * q->q2) - 1
    );
}

/**
 * @brief Quaternion filter using madgwick method.
 * @param q
 *      Quaternion estimation to update and compute.
 * @param ax,ay,az
 *      Accelerometer reading in whatever unit.
 * @param gx,gy,gz
 *      Gyro reading in Rad/s.
*/
void madgwick_update_6(struct Quaternion *q, float ax, float ay, float az, float gx, float gy, float gz) {
    struct Quaternion q_prev = {
        .q1 = q->q1,
        .q2 = q->q2,
        .q3 = q->q3,
        .q4 = q->q4,
    };
    //----- Orientation from angular rate.
    // Quaternion of angular velocity
    struct Quaternion q_w = {
        .q1 = 0,
        .q2 = gx,
        .q3 = gy,
        .q4 = gz
    };
    struct Quaternion q_dot = {0}; // Quaternion derivative

    // q_dot = 0.5 * (q_prev * q_w)
    quat_mult(&q_dot, &q_prev, &q_w);
    quat_scale(&q_dot, &q_dot, 0.5);
    
    //----- Orientation from vector observation.
    struct Quaternion q_a = {
        .q1 = 0,
        .q2 = ax,
        .q3 = ay,
        .q4 = az
    };
    quat_normalize(&q_a, &q_a);
    float f_g[3] = {0}; // madgwick objective function of gravity f(q,d,s) = q^-1 * d * q - s
    float j_g[3][4] = {0};  // jacobian matrix of gravity
    // Compute objective function with g = {0, 0, 0, 1} = d, and q_a = s
    f_g[0] = 2 * (q_prev.q2 * q_prev.q4 - q_prev.q1 * q_prev.q3) - q_a.q2;
    f_g[1] = 2 * (q_prev.q1 * q_prev.q2 + q_prev.q3 * q_prev.q4) - q_a.q3;
    f_g[2] = 2 * (0.5 - q_prev.q2 * q_prev.q2 - q_prev.q3 * q_prev.q3) - q_a.q4;

    // Compute jacobian matrix
    j_g[0][0] = -2 * q_prev.q3;
    j_g[0][1] =  2 * q_prev.q4;
    j_g[0][2] = -2 * q_prev.q1;
    j_g[0][3] =  2 * q_prev.q2;
    
    j_g[1][0] = 2 * q_prev.q2;
    j_g[1][1] = 2 * q_prev.q1;
    j_g[1][2] = 2 * q_prev.q4;
    j_g[1][3] = 2 * q_prev.q3;
    
    j_g[2][0] = 0;
    j_g[2][1] = -4 * q_prev.q2;
    j_g[2][2] = -4 * q_prev.q3;
    j_g[2][3] = 0;
    
    // Compute gradient = j * f / abs(j * f)
    struct Quaternion gradient;

    gradient.q1 = j_g[0][0] * f_g[0] + j_g[1][0] * f_g[1] + j_g[2][0] * f_g[2];
    gradient.q2 = j_g[0][1] * f_g[0] + j_g[1][1] * f_g[1] + j_g[2][1] * f_g[2];
    gradient.q3 = j_g[0][2] * f_g[0] + j_g[1][2] * f_g[1] + j_g[2][2] * f_g[2];
    gradient.q4 = j_g[0][3] * f_g[0] + j_g[1][3] * f_g[1] + j_g[2][3] * f_g[2];

    quat_normalize(&gradient, &gradient);

    // q_est = q_est_prev + (q_dot - BETA * gradient) * DELTA_TIME
    q->q1 = q->q1 + (q_dot.q1 - BETA * gradient.q1) * DELTA_TIME;
    q->q2 = q->q2 + (q_dot.q2 - BETA * gradient.q2) * DELTA_TIME;
    q->q3 = q->q3 + (q_dot.q3 - BETA * gradient.q3) * DELTA_TIME;
    q->q4 = q->q4 + (q_dot.q4 - BETA * gradient.q4) * DELTA_TIME;
    
    quat_normalize(q, q);
}
/**
 * @brief Quaternion filter using madgwick method.
 * @param q
 *      Quaternion estimation to update and compute.
 * @param ax,ay,az
 *      Accelerometer reading in whatever unit.
 * @param gx,gy,gz
 *      Gyro reading in Rad/s.
 * @param mx,my,mz
 *      Magnetometer reading in whatever unit.
*/
void madgwick_update_9(struct Quaternion *q, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    
    struct Quaternion q_prev = {
        .q1 = q->q1,
        .q2 = q->q2,
        .q3 = q->q3,
        .q4 = q->q4,
    };
    //----- Orientation from angular rate.
    struct Quaternion q_w = {.q1 = 0, .q2 = gx, .q3 = gy, .q4 = gz}; // Quaternion from gyro
    struct Quaternion q_dot = {0}; // Quaternion derivative

    // q_dot = 0.5 * (q_prev * q_w)
    quat_mult(&q_dot, &q_prev, &q_w);
    quat_scale(&q_dot, &q_dot, 0.5);
    
    //----- Orientation from vector observation.
    //----- Accelerometer
    struct Quaternion q_a = {.q1 = 0, .q2 = ax, .q3 = ay, .q4 = az};
    quat_normalize(&q_a, &q_a);
    float f_g[3] = {0}; // madgwick objective function of gravity
    float j_g[3][4] = {0}; // jacobian matrix of gravity
    // Compute objective function f(q,g,s) = q^-1 * d * q - s with d = g = {0, 0, 0, 1}, s = {0, ax, ay, az}
    f_g[0] = 2 * (q_prev.q2 * q_prev.q4 - q_prev.q1 * q_prev.q3) - q_a.q2;
    f_g[1] = 2 * (q_prev.q1 * q_prev.q2 + q_prev.q3 * q_prev.q4) - q_a.q3;
    f_g[2] = 2 * (0.5 - q_prev.q2 * q_prev.q2 - q_prev.q3 * q_prev.q3) - q_a.q4;

    // Compute jacobian matrix
    j_g[0][0] = -2 * q_prev.q3;
    j_g[0][1] =  2 * q_prev.q4;
    j_g[0][2] = -2 * q_prev.q1;
    j_g[0][3] =  2 * q_prev.q2;
    
    j_g[1][0] = 2 * q_prev.q2;
    j_g[1][1] = 2 * q_prev.q1;
    j_g[1][2] = 2 * q_prev.q4;
    j_g[1][3] = 2 * q_prev.q3;
    
    j_g[2][0] = 0;
    j_g[2][1] = -4 * q_prev.q2;
    j_g[2][2] = -4 * q_prev.q3;
    j_g[2][3] = 0;
    
    //-----Magnetometer
    struct Quaternion q_m = {.q1 = 0, .q2 = mx, .q3 = my, .q4 = mz};
    quat_normalize(&q_m, &q_m);
    float f_b[3] = {0}; // madgwick objective function for magnetic field
    float j_b[3][4] = {0}; // jacobian matrix of magnetic field
    // Compute h = q_prev * m * q_prev^-1
    struct Quaternion h = {0};
    quat_rotate(&h, &q_m, &q_prev);
    // Compute b
    struct Quaternion b = {
        0,
        sqrtf(h.q2 * h.q2 + h.q3 * h.q3),
        0,
        h.q4
    };

    // Compute objective function f(q,g,s) = q^-1 * d * q - s with d = b, s = m
    f_b[0] = 2 * b.q2 * (0.5 - q_prev.q3 * q_prev.q3 - q_prev.q4 * q_prev.q4) + 2 * b.q4 * (q_prev.q2 * q_prev.q4 - q_prev.q1 * q_prev.q3) - q_m.q2;
    f_b[1] = 2 * b.q2 * (q_prev.q2 * q_prev.q3 - q_prev.q1 * q_prev.q4) + 2 * b.q4 * (q_prev.q1 * q_prev.q2 + q_prev.q3 * q_prev.q4) - q_m.q3;
    f_b[2] = 2 * b.q2 * (q_prev.q1 * q_prev.q3 + q_prev.q2 * q_prev.q4) + 2 * b.q4 * (0.5 - q_prev.q2 * q_prev.q2 - q_prev.q3 * q_prev.q3) - q_m.q4;
    
    // Compute jacobian matrix
    j_b[0][0] = -2 * b.q4 * q_prev.q3;
    j_b[0][1] =  2 * b.q4 * q_prev.q4;
    j_b[0][2] = -4 * b.q2 * q_prev.q3 - 2 * b.q4 * q_prev.q1;
    j_b[0][3] = -4 * b.q2 * q_prev.q4 + 2 * b.q4 * q_prev.q2;
    
    j_b[1][0] = -2 * b.q2 * q_prev.q4 + 2 * b.q4 * q_prev.q2;
    j_b[1][1] =  2 * b.q2 * q_prev.q3 + 2 * b.q4 * q_prev.q1;
    j_b[1][2] =  2 * b.q2 * q_prev.q2 + 2 * b.q4 * q_prev.q4;
    j_b[1][3] = -2 * b.q2 * q_prev.q1 + 2 * b.q4 * q_prev.q3;
    
    j_b[2][0] =  2 * b.q2 * q_prev.q3;
    j_b[2][1] =  2 * b.q2 * q_prev.q4 - 4 * b.q4 * q_prev.q2;
    j_b[2][2] =  2 * b.q2 * q_prev.q1 - 4 * b.q4 * q_prev.q3;
    j_b[2][3] =  2 * b.q2 * q_prev.q2;
    // Compute gradient = j * f / abs(j * f)
    struct Quaternion gradient;

    gradient.q1 = j_g[0][0] * f_g[0] + j_g[1][0] * f_g[1] + j_g[2][0] * f_g[2] + j_b[0][0] * f_b[0] + j_b[1][0] * f_b[1] + j_b[2][0] * f_b[2];
    gradient.q2 = j_g[0][1] * f_g[0] + j_g[1][1] * f_g[1] + j_g[2][1] * f_g[2] + j_b[0][1] * f_b[0] + j_b[1][1] * f_b[1] + j_b[2][1] * f_b[2];
    gradient.q3 = j_g[0][2] * f_g[0] + j_g[1][2] * f_g[1] + j_g[2][2] * f_g[2] + j_b[0][2] * f_b[0] + j_b[1][2] * f_b[1] + j_b[2][2] * f_b[2];
    gradient.q4 = j_g[0][3] * f_g[0] + j_g[1][3] * f_g[1] + j_g[2][3] * f_g[2] + j_b[0][3] * f_b[0] + j_b[1][3] * f_b[1] + j_b[2][3] * f_b[2];

    quat_normalize(&gradient, &gradient);
    
    // q_est = q_est_prev + (q_dot - BETA * gradient) * DELTA_TIME
    q->q1 = q->q1 + (q_dot.q1 - BETA * gradient.q1) * DELTA_TIME;
    q->q2 = q->q2 + (q_dot.q2 - BETA * gradient.q2) * DELTA_TIME;
    q->q3 = q->q3 + (q_dot.q3 - BETA * gradient.q3) * DELTA_TIME;
    q->q4 = q->q4 + (q_dot.q4 - BETA * gradient.q4) * DELTA_TIME;
    
    quat_normalize(q, q);
}