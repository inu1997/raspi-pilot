#include "tv.h"

/**
 * @brief Check if tv is non-zero
 * 
 * @param tv 
 * @return True if the tv is non-zero else false. 
 */
bool tv_is_updated(struct timeval *tv) {
    return (tv->tv_sec != 0) && (tv->tv_usec != 0);
}

/**
 * @brief Get the time pass since 1970.1.1
 * 
 * @return time
 */
uint64_t tv_get_msec_since_epoch() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/**
 * @brief Busy loop wait second.
 * 
 * @param sec 
 */
void tv_sleep_sec(int sec) {
    struct timeval tv, now;
    gettimeofday(&tv, 0);
    gettimeofday(&now, 0);
    while(tv_get_diff_sec_ul(&tv, &now) < sec) {
        gettimeofday(&now, 0);
    }
}

/**
 * @brief Busy loop wait micro second.
 * 
 * @param usec 
 */
void tv_sleep_usec(int usec) {
    struct timeval tv, now;
    gettimeofday(&tv, 0);
    gettimeofday(&now, 0);
    while(tv_get_diff_usec_ul(&tv, &now) < usec) {
        gettimeofday(&now, 0);
    }
}

/**
 * @brief Wait time passed from last_tv for seconds. and update it.
 * 
 * @param last_tv
 *      The last tv for this computation.
 * @param sec
 */
void tv_wait_sec(struct timeval *last_tv, int sec) {
    struct timeval now;
    gettimeofday(&now, 0);
    while (tv_get_diff_sec_ul(last_tv, &now) < sec) {
        gettimeofday(&now, 0);
    }
    gettimeofday(last_tv, 0);
}
/**
 * @brief Wait time passed from last_tv for micro seconds. and update it.
 * 
 * @param last_tv
 *      The last tv for this computation.
 * @param sec
 */
void tv_wait_usec(struct timeval *last_tv, int usec) {
    struct timeval now;
    gettimeofday(&now, 0);
    while (tv_get_diff_usec_ul(last_tv, &now) < usec) {
        gettimeofday(&now, 0);
    }
    gettimeofday(last_tv, 0);
}

/**
 * @brief Get time diff in sec(float).
 * 
 * @param tv
 *      The early one.
 * @param now
 *      The present one.
 * @return Time diff.
 */
float tv_get_diff_sec_f(struct timeval *tv, struct timeval *now) {
    unsigned long sec_diff = now->tv_sec - tv->tv_sec;
    unsigned long usec_diff = now->tv_usec - tv->tv_usec;
    
    return sec_diff + usec_diff * 1000000;
}

/**
 * @brief Get time diff in sec(ul).
 * 
 * @param tv
 *      The early one.
 * @param now
 *      The present one.
 * @return Time diff.
 */
unsigned long tv_get_diff_sec_ul(struct timeval *tv, struct timeval *now) {
    unsigned long sec_diff = now->tv_sec - tv->tv_sec;
    
    return sec_diff;
}

/**
 * @brief Get time diff in msec(ul).
 * 
 * @param tv
 *      The early one.
 * @param now
 *      The present one.
 * @return Time diff.
 */
unsigned long tv_get_diff_msec_ul(struct timeval *tv, struct timeval *now) {
    unsigned long sec_diff = now->tv_sec - tv->tv_sec;
    unsigned long usec_diff = now->tv_usec - tv->tv_usec;
    
    return (sec_diff * 1000L) + (usec_diff / 1000L);
}

/**
 * @brief Get time diff in usec(ul).
 * 
 * @param tv
 *      The early one.
 * @param now
 *      The present one.
 * @return Time diff.
 */
unsigned long tv_get_diff_usec_ul(struct timeval *tv, struct timeval *now) {
    unsigned long sec_diff = now->tv_sec - tv->tv_sec;
    unsigned long usec_diff = now->tv_usec - tv->tv_usec;

    return (sec_diff * 1000000L) + usec_diff;
}