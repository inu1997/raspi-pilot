#include "pid.h"
#include "util/macro.h"
#include "util/logger.h"

#include <stdio.h>
#include <string.h>
#include <json-c/json.h>

struct PID{
    float p; // parameter P
    float i; // parameter I
    float d; // parameter D
    float err_sum;  // for integral calculation
    float err_sum_limit; // limit the integral
    float err_prev; // for differential calculation
    float output_limit; // limit the output
    float output; // output of calculation
};

//-----

/**
 * @brief Initiator of PID struct.
 * @param name
 *      Be used to recongnize and to be found in list.
 * @return The address of newly created PID struct.
 */
struct PID *pid_init(){
    struct PID *pid = malloc(sizeof(struct PID));
    memset(pid, 0, sizeof(struct PID));

    return pid;
}

/**
 * @brief Initiator of PID struct with parameters.
 * 
 * @param p 
 *      P gain.
 * @param i 
 *      I gain.
 * @param d 
 *      D gain.
 * @param err_sum_limit
 *      I Limit. 
 * @param output_limit
 *      Output Limit. 
 * @return The address of newly created PID struct. 
 */
struct PID *pid_init_param(float p, float i, float d, float err_sum_limit, float output_limit) {
    struct PID *pid = pid_init();
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->err_sum_limit = err_sum_limit;
    pid->output_limit = output_limit;
    
    return pid;
}

/**
 * @brief PID computation update.
 * 
 * @param pid
 *      The pid to update.
 * @param sp
 *      The set point.
 * @param pv
 *      The process value.
 * @return The output of pid computation.
 */
float pid_update(struct PID *pid, float sp, float pv){
    float err;
    err = sp - pv;
    float output_p, output_i, output_d, output;
    
    // P
    output_p = pid->p * err;

    // I
    pid->err_sum += err;
    pid->err_sum = LIMIT_MAX_MIN(pid->err_sum, pid->err_sum_limit, -pid->err_sum_limit);
    output_i = pid->i * pid->err_sum;

    // D
    output_d = pid->d * (err - pid->err_prev);
    pid->err_prev = err;
    
    output = output_p + output_i + output_d;
    pid->output = LIMIT_MAX_MIN(output, pid->output_limit, -pid->output_limit);
    
    return output;
}

/**
 * @brief Reset the differential and integral calculation.
 */
void pid_reset(struct PID *pid){
    pid->err_sum = 0;
    pid->err_prev = 0;
}

//----- Setter and Getters

/**
 * @brief All in one function to tune PID.
 * 
 * @param pid 
 * @param p 
 *      P gain.
 * @param i 
 *      I gain.
 * @param d 
 *      D gain.
 * @param err_sum_limit 
 *      I limit.
 * @param output_limit
 *      Output limit. 
 */
void pid_tune(struct PID *pid, float p, float i, float d, float err_sum_limit, float output_limit) {
    pid_set_p(pid, p);
    pid_set_i(pid, i);
    pid_set_d(pid, d);
    pid_set_err_sum_limit(pid, err_sum_limit);
    pid_set_output_limit(pid, output_limit);
}

/**
 * @brief Setter of PID parameter p.
 * @param pid
 *      The PID struct to be set.
 * @param v
 *      New value to set.
 */
void pid_set_p(struct PID *pid, float v){
    pid->p = v;
}

/**
 * @brief Getter of PID parameter p.
 * @param pid
 *      The PID struct to be read.
 * @return Requested parameter.
 */
float pid_get_p(struct PID *pid){
    return pid->p;
}

/**
 * @brief Setter of PID parameter i.
 * @param pid
 *      The PID struct to be set.
 * @param v
 *      New value to set.
 */
void pid_set_i(struct PID *pid, float v){
    pid->i = v;
}

/**
 * @brief Getter of PID parameter i.
 * @param pid
 *      The PID struct to be read.
 * @return Requested parameter.
 */
float pid_get_i(struct PID *pid){
    return pid->i;
}

/**
 * @brief Setter of PID parameter d.
 * @param pid
 *      The PID struct to be set.
 * @param v
 *      New value to set.
 */
void pid_set_d(struct PID *pid, float v){
    pid->d = v;
}

/**
 * @brief Getter of PID parameter d.
 * @param pid
 *      The PID struct to be read.
 * @return Requested parameter.
 */
float pid_get_d(struct PID *pid){
    return pid->d;
}

/**
 * @brief Setter of PID parameter err_sum_limit.
 * @param pid
 *      The PID struct to be set.
 * @param v
 *      New value to set.
 */
void pid_set_err_sum_limit(struct PID *pid, float v){
    pid->err_sum_limit = v;
}

/**
 * @brief Getter of PID parameter err_sum_limit.
 * @param pid
 *      The PID struct to be read.
 * @return Requested parameter.
 */
float pid_get_err_sum_limit(struct PID *pid){
    return pid->err_sum_limit;
}

/**
 * @brief Setter of PID parameter output_limit.
 * @param pid
 *      The PID struct to be set.
 * @param v
 *      New value to set.
 */
void pid_set_output_limit(struct PID *pid, float v){
    pid->output_limit = v;
}

/**
 * @brief Getter of PID parameter output_limit.
 * @param pid
 *      The PID struct to be read.
 * @return Requested parameter.
 */
float pid_get_output_limit(struct PID *pid){
    return pid->output_limit;
}

/**
 * @brief Getter of PID output.
 * 
 * @param pid
 *      The PID struct to be read.
 * @return The output.
 */
float pid_get_output(struct PID *pid) {
    return pid->output;
}

//-----