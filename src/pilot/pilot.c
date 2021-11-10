#include "pilot.h"
#include "controller.h"

#include "util/loop.h"
#include "util/logger.h"
#include "util/debug.h"
#include "util/macro.h"
#include "util/parameter.h"

#include "driver/pca9685.h"
#include "mavlink/mavlink_main.h"

#include "measurement/measurement.h"
#include "measurement/calibration.h"

#include <unistd.h>
#include <pthread.h>
#include <string.h>

#define DEFAULT_MODE PILOT_MODE_PREFLIGHT
#define DEAD_BAND 50

static int _mode = DEFAULT_MODE;
static bool _heading_is_locked;

static pthread_mutex_t _pilot_mutex;

//----- Limitations
static uint16_t _prev_btn_state;
static float _thr; // Throttle.
static float _thr_min; // throttle stroke min. Only clamp if manual control got non-zero throttle.
static float _thr_max; // throttle stroke max.
static float _avx; // Angular velocity x.
static float _avy; // Angular velocity y.
static float _avz; // Angular velocity z.
static float _heading; // Locked heading.
static float _avx_range; // Range of angular velocity x.
static float _avy_range; // Range of angular velocity y.
static float _avz_range; // Range of angular velocity z.
static int _deadband; // Manual control deadband.
static float _gimbal_velocity_x;
static float _gimbal_position_x;

//-----

enum BUTTON {
    BUTTON_CALIB_MAG = 0x01,
    BUTTON_CALIB_GYRO = 0x02
};

/**
 * @brief Initiator of flight controller.
 * 
 * @return 0 if success else -1.
 */
int pilot_init(){
    LOG("Initiating pilot.\n");
    pthread_mutex_init(&_pilot_mutex, NULL);

    if (controller_init() != 0) {
        LOG_ERROR("Failed to initiate controller.\n");
        return -1;
    }

    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MANUAL_THR_MAX], &_thr_max);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MANUAL_THR_MIN], &_thr_min);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MANUAL_AVX_RAN], &_avx_range);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MANUAL_AVY_RAN], &_avy_range);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MANUAL_AVZ_RAN], &_avz_range);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MANUAL_DEADBAND], &_deadband);
    LOG(
        "Loaded manual control ranges:\n"
        "   THR_MIN: %6.2f\n"
        "   THR_MAX: %6.2f\n"
        "   AVX_RANGE: %6.2f\n"
        "   AVY_RANGE: %6.2f\n"
        "   AVZ_RANGE: %6.2f\n"
        "   DEADBAND: %d\n",
        _thr_min,
        _thr_max,
        _avx_range,
        _avy_range,
        _avz_range,
        _deadband);
    
    _gimbal_velocity_x = 0;
    _gimbal_position_x = 1000.0f;
    _heading_is_locked = false;
    _prev_btn_state = 0;
    LOG("Done.\n");

    // Nod the camera.
    pca_write_servo(15, 2000);
    usleep(1000000);
    pca_write_servo(15, 1000);
    
    //----- Ready
    pilot_set_mode(PILOT_MODE_STABILIZE);
    return 0;
}

/**
 * @brief Compute PID control and update all motors.
 * 
 */
void pilot_update(){
    pilot_lock_mutex();
    
    if (pilot_is_armed()) {
        controller_update(
            _mode & (~PILOT_AMRED_FLAG),
            _thr,
            _avz,
            _heading);
    } 
    
    // Control the gimbal
    _gimbal_position_x += _gimbal_velocity_x * loop_get_interval();
    _gimbal_position_x = LIMIT_MAX_MIN(_gimbal_position_x, 2000, 1000);
    pca_write_servo(15, _gimbal_position_x);

    pilot_unlock_mutex();
    
    if (pilot_is_armed() && mavlink_get_active_connections() == 0) {
        pilot_disarm();
    }
}

/**
 * @brief Preventing Flight Controller get messed up by multi-threading.
 * 
 */
void pilot_lock_mutex() {
    pthread_mutex_lock(&_pilot_mutex);
}

/**
 * @brief Preventing Flight Controller get messed up by multi-threading.
 * 
 */
void pilot_unlock_mutex() {
    pthread_mutex_unlock(&_pilot_mutex);
}

/**
 * @brief Check if pilot is amred.
 * 
 * @return True if armed else false.
 */
bool pilot_is_armed() {
    return _mode & PILOT_AMRED_FLAG ? true : false;
}

/**
 * @brief Arm the pilot.
 * 
 * @return 0 if success else -1. 
 */
int pilot_arm() {
    pilot_lock_mutex();
    
    int ret = -1;
    if (!pilot_is_armed()) {
        _mode |= PILOT_AMRED_FLAG;
        ret = 0;
        controller_reset();
        // Disable running calibration.
        calibration_set_mag_gathering_enable(false);
        calibration_set_gyro_gathering_enable(false);
        // Set all value to zero.
        pilot_set_avx(0);
        pilot_set_avy(0);
        pilot_set_avz(0);
        pilot_set_thr(0);
    }
    
    pilot_unlock_mutex();
    return ret;
}

/**
 * @brief Disarm the pilot.
 * 
 * @return 0 if success else -1.
 */
int pilot_disarm() {
    pilot_lock_mutex();
    
    int ret = -1;
    if (pilot_is_armed()) {
        _mode &= ~PILOT_AMRED_FLAG;
        ret = 0;
        controller_reset();
    }

    pilot_unlock_mutex();
    return ret;
}

/**
 * @brief Handle the MAVLink manual control message.
 *      Map them to controller parameter value.
 * @param x MAVLink manual control x.
 * @param y MAVLink manual control y.
 * @param z MAVLink manual control z.
 * @param r MAVLink manual control r.
 * @param s MAVLink manual control s.
 * @param t MAVLink manual control t.
 * @param btns1 MAVLink manual control button1.
 * @param btns2 MAVLink manual control button2.
 */
void pilot_handle_menual(
    int16_t x, int16_t y, int16_t z, 
    int16_t r, int16_t s, int16_t t,
    uint16_t btns1, uint16_t btns2) {
    pilot_lock_mutex();
    
    x = DEADBAND(x, _deadband);
    y = DEADBAND(y, _deadband);
    z = DEADBAND_OFFSET(z, _deadband, 500);
    r = DEADBAND(r, _deadband);
    s = DEADBAND(s, _deadband);
    t = DEADBAND(t, _deadband);
    
    // MAP Z axis to throttle.
    pilot_set_thr(MAP(z, 500.0f, 1000.0f, 0.0f, 100.0f));
    
    // MAP R axis to avz.
    pilot_set_avz(MAP(-r, -1000.0f, 1000.0f, -_avz_range, _avz_range));

    // MAP X axis to gimbal valocity.
    pilot_set_gimbal_velocity(MAP(x, -1000, 1000.0f, -600, 600));
    
    // Handle button.
    if ((_prev_btn_state ^ btns1) & BUTTON_CALIB_MAG) {
        if (btns1 & BUTTON_CALIB_MAG) {
            // Toggle calibration.
            calibration_set_mag_gathering_enable(!calibration_mag_gathering_is_enabled());
        }
    }
    
    if ((_prev_btn_state ^ btns1) & BUTTON_CALIB_GYRO) {
        if (btns1 & BUTTON_CALIB_GYRO) {
            // Toggle calibration.
            calibration_set_gyro_gathering_enable(!calibration_gyro_gathering_is_enabled());
        }
    }

    _prev_btn_state = btns1;

    pilot_unlock_mutex();
}

//----- Setter and Getters.

/**
 * @brief Setter of flight controller mode.
 * @param mode 
 *      Mode to set flight controller.
 */
void pilot_set_mode(int mode){
    _mode &= PILOT_AMRED_FLAG;
    _mode |= mode & 0x7f;
}

/**
 * @brief Getter of flight controller mode.
 * 
 * @return the current mode.
 */
int pilot_get_mode(){
    return _mode;
}

/**
 * @brief Set the deadband of manual control.
 * 
 * @param db 
 *      Deadband.
 */
void pilot_set_deadband(int db);

/**
 * @brief Get the deadband of manual control.
 * 
 * @return Deadband.
 */
int pilot_get_deadband();

// Range setter/getter.
void pilot_set_thr_max(float max) {
    _thr_max = max;
}

float pilot_get_thr_max() {
    return _thr_max;
}

void pilot_set_thr_min(float min) {
    _thr_min = min;
}

float pilot_get_thr_min() {
    return _thr_min;
}

void pilot_set_avx_range(float radsec) {
    _avx_range = radsec;
}

float pilot_get_avx_range() {
    return _avx_range;
}

void pilot_set_avy_range(float radsec) {
    _avy_range = radsec;
}

float pilot_get_avy_range() {
    return _avy_range;
}

void pilot_set_avz_range(float radsec) {
    _avz_range = radsec;
}

float pilot_get_avz_range() {
    return _avz_range;
}

// Control parameter setter/getter.
/**
 * @brief Set the throttle from 0.0 to 100.0 %
 * 
 * @param thr 
 *      Throttle by percent.
 */
void pilot_set_thr(float thr) {
    thr = LIMIT_MAX_MIN(thr, 100.0, -100.0);
    _thr = thr > 0 ? MAP(thr, 0, 100.0f, _thr_min, _thr_max) :
        thr < 0 ? MAP(thr, 0, -100.0f, -_thr_min, -_thr_max) :
        0.0;
}

/**
 * @brief Get the clamped throttle.
 * 
 * @return Throttle.
 */
float pilot_get_thr() {
    return _thr;
}

/**
 * @brief Set the angular velocity x.
 * 
 * @param radsec
 *      Angular velocity in unit of radian per second speed. 
 */
void pilot_set_avx(float radsec) {
    _avx = LIMIT_MAX_MIN(
        radsec, 
        _avx_range,
        -_avx_range);
    // DEBUG("avx: %5.1f rad/sec.\n", _avx);
}

/**
 * @brief Get the angular velocity x.
 * 
 * @return Angular velocity x.
 */
float pilot_get_avx() {
    return _avx;
}

/**
 * @brief Set the angular velocity y.
 * 
 * @param radsec
 *      Angular velocity in unit of radian per second speed. 
 */
void pilot_set_avy(float radsec) {
    _avy = LIMIT_MAX_MIN(
        radsec, 
        _avy_range,
        -_avy_range);
        
    // DEBUG("avy: %5.1f rad/sec.\n", _avy);
}

/**
 * @brief Get the angular velocity y.
 * 
 * @return Angular velocity y.
 */
float pilot_get_avy() {
    return _avy;
}

/**
 * @brief Set the angular velocity z.
 *      0.0 is to lock the heading.
 *      Non-zero value will unlock the heading.
 * @param radsec
 *      Angular velocity in unit of radian per second speed. 
 */
void pilot_set_avz(float radsec) {
    _avz = LIMIT_MAX_MIN(
        radsec, 
        _avz_range,
        -_avz_range);
    
    if (_avz != 0.0) {
        _heading_is_locked = false;
    }
    
    // DEBUG("avz: %5.1f rad/sec.\n", _avz);

    // 0.0 is to lock the heading.
    if (_avz == 0.0 && !pilot_heading_is_locked()) {
        pilot_set_heading(ahrs_get_yaw_heading());
        // DEBUG("Locking yaw: %6.1f.\n", _heading);
    } 
}

/**
 * @brief Get the angular velocity z.
 * 
 * @return Angular velocity z.
 */
float pilot_get_avz() {
    return _avz;
}

/**
 * @brief Check is heading is locked.
 *      It can be locked via pilot_set_heading function.
 * 
 * @return True if heading is locked else false.
 */
bool pilot_heading_is_locked() {
    return _heading_is_locked;
}

void pilot_set_gimbal_velocity(float radsec) {
    _gimbal_velocity_x = radsec;
}

float pilot_get_gimbal_velocity() {
    return _gimbal_velocity_x;
}

void pilot_set_gimbal_position(float position) {
    _gimbal_position_x = position;
}

float pilot_get_gimbal_position() {
    return _gimbal_position_x;
}

/**
 * @brief Set the lock heading of pilot.
 * 
 * @param heading 
 *      The heading from -Pi to +Pi.
 */
void pilot_set_heading(float heading) {
    _heading_is_locked = true;
    _heading = heading;
}

/**
 * @brief Get the (previous) locking heading of pilot.
 * 
 * @return Heading.
 */
float pilot_get_heading() {
    return _heading;
}
//-----