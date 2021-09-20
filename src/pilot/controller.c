#include "controller.h"
#include "actuator/motor.h"
#include "pid.h"

#include "util/logger.h"
#include "util/debug.h"
#include "util/parameter.h"

/**
 * All controlling diagram:
 * Altitude
 * a --> X --> [Attitude PID] --> X --> [Rate PID] --> output
 *       ↑                        ↑
 *     Attitude                  Rate
 * 
 * Vertical acceleration
 * 0 --> X --> [Vertical Acceleration] --> output
 *       ↑
 *     Accelerometer Z
 * 
 * Altitude hold
 * a --> X --> [Altitude PID] --> output
 *       ↑
 *     Altitude
 */

struct PID *pidsetting_ax; // Attitude X PID.
struct PID *pidsetting_ay; // Attitude Y PID.
struct PID *pidsetting_az; // Attitude Z PID.

struct PID *pidsetting_avx; // Angular Velocity X PID.
struct PID *pidsetting_avy; // Angular Velocity Y PID.
struct PID *pidsetting_avz; // Angular Velocity Z PID.

struct PID *pidsetting_va; // Vertical acceleration PID.
struct PID *pidsetting_alt; // Altitude hold PID.

//----- PID outputs

float _output_pid_ax; // Output of Attitude X PID.
float _output_pid_ay; // Output of Attitude Y PID.
float _output_pid_az; // Output of Attitude Z PID.
float _output_pid_avx; // Output of Angular Velocity X PID.
float _output_pid_avy; // Output of Angular Velocity Y PID.
float _output_pid_avz; // Output of Angular Velocity Z PID.
float _output_pid_va; // Output of Vertical Acceleration PID.
float _output_pid_alt;  // Output of altitude hold PID

//----- Variables

void load_param_ax();
void load_param_ay();
void load_param_az();
void load_param_avx();
void load_param_avy();
void load_param_avz();
void load_param_va();
void load_param_alt();

//-----

int controller_init() {
    LOG("Initiating Controller.\n");

    LOG("Initiating PID.\n");
    //----- PID
    pidsetting_ax = pid_init();
    pidsetting_ay = pid_init();
    pidsetting_az = pid_init();
    pidsetting_avx = pid_init();
    pidsetting_avy = pid_init();
    pidsetting_avz = pid_init();
    pidsetting_va = pid_init();
    pidsetting_alt = pid_init();

    load_param_ax();
    load_param_ay();
    load_param_az();
    load_param_avx();
    load_param_avy();
    load_param_avz();
    load_param_va();
    load_param_alt();
    
    LOG("PID Settings:\n");
    LOG(
        "PID_AX: P: %6.2f, I: %6.2f, D: %6.2f, I_LIMIT: %6.2f, O_LIMIT: %6.2f\n",
        pid_get_p(pidsetting_ax),
        pid_get_i(pidsetting_ax),
        pid_get_d(pidsetting_ax),
        pid_get_err_sum_limit(pidsetting_ax),
        pid_get_output_limit(pidsetting_ax)
    );
    
    LOG(
        "PID_AY: P: %6.2f, I: %6.2f, D: %6.2f, I_LIMIT: %6.2f, O_LIMIT: %6.2f\n",
        pid_get_p(pidsetting_ay),
        pid_get_i(pidsetting_ay),
        pid_get_d(pidsetting_ay),
        pid_get_err_sum_limit(pidsetting_ay),
        pid_get_output_limit(pidsetting_ay)
    );
    
    LOG(
        "PID_AZ: P: %6.2f, I: %6.2f, D: %6.2f, I_LIMIT: %6.2f, O_LIMIT: %6.2f\n",
        pid_get_p(pidsetting_az),
        pid_get_i(pidsetting_az),
        pid_get_d(pidsetting_az),
        pid_get_err_sum_limit(pidsetting_az),
        pid_get_output_limit(pidsetting_az)
    );
    
    LOG(
        "PID_AVX: P: %6.2f, I: %6.2f, D: %6.2f, I_LIMIT: %6.2f, O_LIMIT: %6.2f\n",
        pid_get_p(pidsetting_avx),
        pid_get_i(pidsetting_avx),
        pid_get_d(pidsetting_avx),
        pid_get_err_sum_limit(pidsetting_avx),
        pid_get_output_limit(pidsetting_avx)
    );
    
    LOG(
        "PID_AVY: P: %6.2f, I: %6.2f, D: %6.2f, I_LIMIT: %6.2f, O_LIMIT: %6.2f\n",
        pid_get_p(pidsetting_avy),
        pid_get_i(pidsetting_avy),
        pid_get_d(pidsetting_avy),
        pid_get_err_sum_limit(pidsetting_avy),
        pid_get_output_limit(pidsetting_avy)
    );
    
    LOG(
        "PID_AVZ: P: %6.2f, I: %6.2f, D: %6.2f, I_LIMIT: %6.2f, O_LIMIT: %6.2f\n",
        pid_get_p(pidsetting_avz),
        pid_get_i(pidsetting_avz),
        pid_get_d(pidsetting_avz),
        pid_get_err_sum_limit(pidsetting_avz),
        pid_get_output_limit(pidsetting_avz)
    );
    
    LOG(
        "PID_VA: P: %6.2f, I: %6.2f, D: %6.2f, I_LIMIT: %6.2f, O_LIMIT: %6.2f\n",
        pid_get_p(pidsetting_va),
        pid_get_i(pidsetting_va),
        pid_get_d(pidsetting_va),
        pid_get_err_sum_limit(pidsetting_va),
        pid_get_output_limit(pidsetting_va)
    );
    
    LOG(
        "PID_ALT: P: %6.2f, I: %6.2f, D: %6.2f, I_LIMIT: %6.2f, O_LIMIT: %6.2f\n",
        pid_get_p(pidsetting_alt),
        pid_get_i(pidsetting_alt),
        pid_get_d(pidsetting_alt),
        pid_get_err_sum_limit(pidsetting_alt),
        pid_get_output_limit(pidsetting_alt)
    );

    // if (motor_init() != 0) {
    //     LOG_ERROR("Failed to initiate motor.\n");
    //     return -1;
    // }

    LOG("Done.\n");
    return 0;
}

void controller_update(uint8_t mode, float thr, float avz) {

}

void controller_get_thr_output(float *thr1, float *thr2, float *thr3, float *thr4);

void controller_reset() {
    motor_turn_off_all();
    pid_reset(pidsetting_ax);
    pid_reset(pidsetting_ay);
    pid_reset(pidsetting_az);
    pid_reset(pidsetting_avx);
    pid_reset(pidsetting_avy);
    pid_reset(pidsetting_avz);
    pid_reset(pidsetting_va);
    pid_reset(pidsetting_alt);
}

//-----

void load_param_ax() {
    float p, i, d, i_limit, o_limit;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AX_P], &p);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AX_I], &i);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AX_D], &d);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AX_I_LIMIT], &i_limit);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AX_O_LIMIT], &o_limit);
    pid_tune(pidsetting_ax, p, i, d, i_limit, o_limit);
}

void load_param_ay() {
    float p, i, d, i_limit, o_limit;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AY_P], &p);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AY_I], &i);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AY_D], &d);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AY_I_LIMIT], &i_limit);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AY_O_LIMIT], &o_limit);
    pid_tune(pidsetting_ay, p, i, d, i_limit, o_limit);
}

void load_param_az() {
    float p, i, d, i_limit, o_limit;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AZ_P], &p);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AZ_I], &i);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AZ_D], &d);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AZ_I_LIMIT], &i_limit);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AZ_O_LIMIT], &o_limit);
    pid_tune(pidsetting_az, p, i, d, i_limit, o_limit);
}
void load_param_avx() {
    float p, i, d, i_limit, o_limit;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVX_P], &p);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVX_I], &i);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVX_D], &d);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVX_I_LIMIT], &i_limit);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVX_O_LIMIT], &o_limit);
    pid_tune(pidsetting_avx, p, i, d, i_limit, o_limit);
}

void load_param_avy() {
    float p, i, d, i_limit, o_limit;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVY_P], &p);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVY_I], &i);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVY_D], &d);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVY_I_LIMIT], &i_limit);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVY_O_LIMIT], &o_limit);
    pid_tune(pidsetting_avy, p, i, d, i_limit, o_limit);
}

void load_param_avz() {
    float p, i, d, i_limit, o_limit;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVZ_P], &p);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVZ_I], &i);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVZ_D], &d);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVZ_I_LIMIT], &i_limit);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_AVZ_O_LIMIT], &o_limit);
    pid_tune(pidsetting_avz, p, i, d, i_limit, o_limit);
}

void load_param_va() {
    float p, i, d, i_limit, o_limit;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_VA_P], &p);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_VA_I], &i);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_VA_D], &d);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_VA_I_LIMIT], &i_limit);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_VA_O_LIMIT], &o_limit);
    pid_tune(pidsetting_va, p, i, d, i_limit, o_limit);
}

void load_param_alt() {
    float p, i, d, i_limit, o_limit;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_ALT_P], &p);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_ALT_I], &i);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_ALT_D], &d);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_ALT_I_LIMIT], &i_limit);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_PID_ALT_O_LIMIT], &o_limit);
    pid_tune(pidsetting_alt, p, i, d, i_limit, o_limit);
}