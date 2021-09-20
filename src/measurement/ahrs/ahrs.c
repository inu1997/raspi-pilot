#include "ahrs.h"
#include "complementary.h"
#include "quaternion.h"
#include "madgwick.h"

#include "util/logger.h"
#include "util/debug.h"

// #define UPDATE_METHOD_COMPLEMENTARY
#define UPDATE_METHOD_MADGWICK

#define COMPLEMENTARY_ALPHA 0.98

struct Quaternion _q; // Quaternion.
float _r; // Attitude Roll in Degree.
float _p; // Attitude Pitch in Degree.
float _y; // Attitude Yaw in Degree.

/**
 * @brief Initiator of ahrs, will initiate IMU and MAG also.
 * 
 * @return 0 if success else -1.
 */
int ahrs_init(){
    _r = _p = _y = 0;
    _q.q1 = 1.0f;
    _q.q2 = 0.0f;
    _q.q3 = 0.0f;
    _q.q4 = 0.0f;

    LOG("Done.\n");
    return 0;
}

/**
 * @brief Update AHRS with 9 axis sensor reading.
 * 
 * @param ax Accelerometer reading.
 * @param ay Accelerometer reading.
 * @param az Accelerometer reading.
 * @param gx Gyroscope reading in Rad/s.
 * @param gy Gyroscope reading in Rad/s.
 * @param gz Gyroscope reading in Rad/s.
 * @param mx Mangetometer reading.
 * @param my Mangetometer reading.
 * @param mz Mangetometer reading.
 */
void ahrs_update_9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
#ifdef UPDATE_METHOD_COMPLEMENTARY
    float acc_r, acc_p;
    acc_r = roll_from_accel(ax, ay, az);
    acc_p = pitch_from_accel(ax, ay, az);
    
    acc_r = RAD_TO_DEG(acc_r);
    acc_p = RAD_TO_DEG(acc_p);

    _r = complementary_filter(_r, acc_r, gx, COMPLEMENTARY_ALPHA);
    _p = complementary_filter(_p, acc_p, gy, COMPLEMENTARY_ALPHA);
    
    _y = yaw_from_mag(_r, _p, mx, my, mz);

#endif // UPDATE_METHOD_COMPLEMENTARY

#ifdef UPDATE_METHOD_MADGWICK
    
    madgwick_update_9(
        &_q,
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        mx,
        my,
        mz
    );

    quat_to_euler(&_q, &_r, &_p, &_y);
#endif // UPDATE_METHOD_MADGWICK
}

/**
 * @brief Update AHRS with 6 axis sensor reading.
 * 
 * @param ax Accelerometer reading.
 * @param ay Accelerometer reading.
 * @param az Accelerometer reading.
 * @param gx Gyroscope reading in Rad/s.
 * @param gy Gyroscope reading in Rad/s.
 * @param gz Gyroscope reading in Rad/s.
 */
void ahrs_update_6(float ax, float ay, float az, float gx, float gy, float gz) {

    // Calculate Eular angles
#ifdef UPDATE_METHOD_COMPLEMENTARY
    float acc_r, acc_p;
    acc_r = roll_from_accel(ax, ay, az);
    acc_p = pitch_from_accel(ax, ay, az);
    
    acc_r = RAD_TO_DEG(acc_r);
    acc_p = RAD_TO_DEG(acc_p);

    _r = complementary_filter(_r, acc_r, gy, COMPLEMENTARY_ALPHA);
    _p = complementary_filter(_p, acc_p, gz, COMPLEMENTARY_ALPHA);
    
#endif // UPDATE_METHOD_COMPLEMENTARY

#ifdef UPDATE_METHOD_MADGWICK
    
    madgwick_update_6(
        &_q,
        ax,
        ay,
        az,
        gx,
        gy,
        gz
    );

    quat_to_euler(&_q, &_r, &_p, &_y);
#endif // UPDATE_METHOD_MADGWICK
}

/**
 * @brief Reset all ahrs variables.
 */
void ahrs_reset(){
    _r = _p = _y = 0;
    _q.q1 = 1.0f;
    _q.q2 = 0.0;
    _q.q3 = 0.0;
    _q.q4 = 0.0;
}

/**
 * @brief Getter of attitude Roll in Radian.
 * 
 * @return Roll.
 */
float ahrs_get_roll() {
    return _r;
}

/**
 * @brief Getter of attitude Pitch in Radian.
 * 
 * @return Pitch.
 */
float ahrs_get_pitch() {
    return _p;
}

/**
 * @brief Getter of heading yaw in Radian.
 * 
 * @return Yaw.
 */
float ahrs_get_yaw_heading() {
    return _y;
}