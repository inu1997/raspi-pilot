#include "loop.h"
#include "util/tv.h"
#include "util/logger.h"

#include <pthread.h>

static struct timeval _loop_tv;

static float _loop_interval;

static pthread_mutex_t _loop_mutex;

//-----

/*
 * @brief Initiator of loop
 * 
 * @param rate_hz
 *      How fast loop runs
 * @return 0 if success else 1.
 */
int loop_init(float rate_hz){
    _loop_interval = 1.0 / rate_hz;
    _loop_tv.tv_sec = _loop_tv.tv_usec = 0;
    pthread_mutex_init(&_loop_mutex, NULL);

    return 0;
}

/**
 * @brief Delay the loop using busy loop to get constant interval.
 * 
 */
void loop_delay_control(){
    if(!tv_is_updated(&_loop_tv)){
        gettimeofday(&_loop_tv, NULL);
    }
    
    unsigned long usec;

    struct timeval now;
    gettimeofday(&now, NULL);
    while((usec = tv_get_diff_usec_ul(&_loop_tv, &now)) < (unsigned long)(1e6f * _loop_interval)){
        // Using busy loop to control the loop timing.
        gettimeofday(&now, NULL);
    }

    // Check delay to make sure it's not over-delayed.
    if(usec > (unsigned long)(1e6f * _loop_interval) + 10){
        LOG_ERROR("Loop take too long, %d microseconds.\n", usec);
    }
    
    // Update original timeval
    gettimeofday(&_loop_tv, NULL);

}

/**
 * @brief Setter of loop rate.
 * 
 * @param hz_rate 
 *      Loop rate.
 */
void loop_set_rate(float hz_rate){
    _loop_interval = 1.0f / hz_rate;
}

/**
 * @brief Getter of loop rate.
 * 
 * @return Loop rate in Hz.
 */
float loop_get_rate(){
    return 1.0f / _loop_interval;
}

/**
 * @brief Setter of loop interval.
 * 
 * @param usec_interval 
 *      Interval.
 */
void loop_set_interval(float interval){
    _loop_interval = interval;
}

/**
 * @brief Getter of loop interval.
 * 
 * @return Loop interval.
 */
float loop_get_interval(){
    return _loop_interval;
}

/**
 * @brief Lock the mutex to protect control process.
 * 
 */
void loop_lock_mutex() {
    pthread_mutex_lock(&_loop_mutex);
}

/**
 * @brief Lock the mutex to protect control process.
 * 
 */
void loop_unlock_mutex() {
    pthread_mutex_unlock(&_loop_mutex);
}