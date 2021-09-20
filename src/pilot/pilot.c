#include "pilot.h"
#include "controller.h"

#include "util/logger.h"
#include "util/debug.h"
#include "util/parameter.h"

#include <pthread.h>
#include <string.h>

#define DEFAULT_MODE PILOT_MODE_PREFLIGHT

int _mode = DEFAULT_MODE;

pthread_mutex_t _pilot_mutex;

//----- Limitations
float _thr; // Throttle.
float _avz; // Angular velocity z.
float _lmt_avz; // Limit of angular velocity z.

//-----
static void load_param_manual();
//-----

/**
 * @brief Initiator of flight controller.
 * 
 * @return 0 if success else -1.
 */
int pilot_init(){
    LOG("Initiating pilot.\n");
    pthread_mutex_init(&_pilot_mutex, 0);

    if (controller_init() != 0) {
        LOG_ERROR("Failed to initiate controller.\n");
        return -1;
    }

    LOG("Done.\n");
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
        // controller_update(
        //     _mode & (~PILOT_AMRED),
        //     _thr,
        //     _avz);
    } 

    pilot_unlock_mutex();
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


bool pilot_is_armed() {
    return _mode & PILOT_AMRED ? true : false;
}

int pilot_arm() {
    pilot_lock_mutex();
    
    int ret = -1;
    if (!pilot_is_armed()) {
        _mode |= PILOT_AMRED;
        ret = 0;
    }
    
    pilot_unlock_mutex();
    return ret;
}

int pilot_disarm() {
    pilot_lock_mutex();
    
    int ret = -1;
    if (pilot_is_armed()) {
        _mode &= ~PILOT_AMRED;
        ret = 0;
        controller_reset();
    }

    pilot_unlock_mutex();
    return ret;
}

void pilot_handle_menual(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t btns) {
    pilot_lock_mutex();
    
    // MAP R axis to throttle.
    _thr = (float)(r - 500) / 500.0 * 100.0;
    
    // MAP X axis to avz.
    _avz = (float)x / 1000.0 * _lmt_avz;
    
    pilot_unlock_mutex();
}

//----- Setter and Getters.

/**
 * @brief Setter of flight controller mode.
 * @param mode 
 *      Mode to set flight controller.
 */
void pilot_set_mode(int mode){
    _mode &= PILOT_AMRED;
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

void pilot_set_limit_avz(float lmt);

float pilot_get_limit_avz();

//-----

void load_param_manual() {
    float avz_max;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MANUAL_AVZ_MAX], &avz_max);
}