#include "complementary.h"
#include "util/loop.h"
#include <math.h>

#define DELTA_TIME loop_get_interval()

/**
 * @brief Complementary filter calculation.
 * 
 * @param prev_att
 *      Previous attitude from last cycle.
 * @param acc_att
 *      Current atttitude from accelerometer.
 * @param omega
 *      Gyro reading value in same unit of attitude.
 * @param alpha
 *      Weight.
 * @return Filtered attitude in same unit.
 */
float complementary_filter(float old_att, float acc_att, float omega, float alpha) {
    return acc_att * (1 - alpha) + (old_att + omega * DELTA_TIME) * alpha;
}

/**
 * @brief Compute attitude from accelerometer readings.
 * 
 * @param r 
 *      Destination of result of attitude roll in Radian.
 * @param p 
 *      Destination of result of attitude pitch in Radian.
 * @param ax,ay,az
 *      Accelerometer readings.
 */
void accel_to_attitude(float *r, float *p, float ax, float ay, float az) {
    *r = atan2f(ay, sqrtf(ax * ax + az * az));
    *p = atan2f(-ax, sqrtf(ay * ay + az * az));
}

/**
 * @brief Compute the yaw attitude from mangetometer reading.
 * 
 * @param r
 *      Roll attitude in Radian.
 * @param p
 *      Pitch attitude in Radian.
 * @param mx,my,mz
 *      Magnetometer readings.
 * @return yaw attitude in Radian.
 */
float yaw_from_mag(float r, float p, float mx, float my, float mz){ 
   float yh = (my * cosf(r)) - (mz * sinf(r));
   float xh = (mx * cosf(p)) + (my * sinf(r) * sinf(p)) + (mz * cosf(r) * sinf(p));

   return atan2f(xh, xh);
}