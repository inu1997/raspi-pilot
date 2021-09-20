#include "kalman_filter_1d.h"
#include <stdlib.h>

struct Kalman1D{
    float prev_x; // previous data
    float p; // estimate covariance
    float q; // process noise covariance
    float r; // observation covariance
    float k; // kalman gain
};

/**
 * @brief Initiater of Kalman1D struct.
 * 
 * @param prev_x
 *      Data target.
 * @param p
 *      Estmate covariance.
 * @param q
 *      Process noise.
 * @param r
 *      Observation covariance.
 * @return Address of newly created Kalman1D struct.
 */
struct Kalman1D *kf_one_init(float prev_x, float p, float q, float r){
    struct Kalman1D *kf = malloc(sizeof(struct Kalman1D));
    
    kf->prev_x = prev_x;
    kf->p = p;
    kf->q = q;
    kf->r = r;
    kf->k = 0;

    return kf;
}

/**
 * @brief Computation of Kalman filter 1 Dimansion.
 * 
 * @param kf
 *      The kalman filter struct.
 * @param x
 *      The new value.
 * @return The output.
 */
float kf_one_update(struct Kalman1D *kf, float x){
    kf->p = kf->p + kf->q;
    
    float y, s;
    y = x - kf->prev_x;
    s = kf->p + kf->r;
    kf->k = kf->p / s;
    
    kf->prev_x = kf->prev_x + kf->k * y;
    kf->p = (1.0f - kf->k) * kf->p;

    return kf->prev_x;
}