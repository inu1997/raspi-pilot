#include "controller.h"
#include "driver/pca9685.h"
#include "pid.h"

#include "measurement/measurement.h"

#include "util/logger.h"
#include "util/debug.h"
#include "util/macro.h"
#include "util/parameter.h"
#include "util/io/gpio.h"

#define PIN_M1_CW  20
#define PIN_M1_CCW 21

#define PIN_M2_CW  19
#define PIN_M2_CCW 26

#define PIN_M3_CW  27
#define PIN_M3_CCW 17

#define PIN_M4_CW  13
#define PIN_M4_CCW 6

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

static float _output_pid_ax; // Output of Attitude X PID.
static float _output_pid_ay; // Output of Attitude Y PID.
static float _output_pid_az; // Output of Attitude Z PID.
static float _output_pid_avx; // Output of Angular Velocity X PID.
static float _output_pid_avy; // Output of Angular Velocity Y PID.
static float _output_pid_avz; // Output of Angular Velocity Z PID.
static float _output_pid_va; // Output of Vertical Acceleration PID.
static float _output_pid_alt;  // Output of altitude hold PID

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

    if (pca_init() != 0) {
        LOG_ERROR("Failed to initiate motor.\n");
        return -1;{}
    }
    
    int freq;
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_MTR_PWM_FREQ], &freq);
    pca_set_frequency(freq);

    LOG("Initiating GPIO.\n");
    gpio_set_export(PIN_M1_CCW);
    gpio_set_export(PIN_M1_CW);
    gpio_set_export(PIN_M2_CCW);
    gpio_set_export(PIN_M2_CW);
    gpio_set_export(PIN_M3_CCW);
    gpio_set_export(PIN_M3_CW);
    gpio_set_export(PIN_M4_CCW);
    gpio_set_export(PIN_M4_CW);
    gpio_set_direction(PIN_M1_CW, "out");
    gpio_set_direction(PIN_M1_CCW, "out");
    gpio_set_direction(PIN_M2_CW, "out");
    gpio_set_direction(PIN_M2_CCW, "out");
    gpio_set_direction(PIN_M3_CW, "out");
    gpio_set_direction(PIN_M3_CCW, "out");
    gpio_set_direction(PIN_M4_CW, "out");
    gpio_set_direction(PIN_M4_CCW, "out");

    LOG("Done.\n");
    return 0;
}


void controller_update(uint8_t mode, float thr, float avz, float heading) {
    if (thr != 0.0 || avz != 0.0) {
        // Do computation.
        float _thr[4] = {thr, thr, thr, thr};
        _output_pid_az = avz != 0.0 ? avz : pid_update(pidsetting_az, 0, GET_MIN_INCLUDED_ANGLE_RAD(ahrs_get_yaw_heading(), heading));
        _output_pid_avz = pid_update(pidsetting_avz, _output_pid_az, imu_get_gz());

        // Map pid output to throttle array.
        _thr[0] += -_output_pid_avz;
        _thr[1] += -_output_pid_avz;
        _thr[2] += _output_pid_avz;
        _thr[3] += _output_pid_avz;
        
        // Update.
        _thr[0] = LIMIT_MAX_MIN(_thr[0], 100.0, -100.0);
        _thr[1] = LIMIT_MAX_MIN(_thr[1], 100.0, -100.0);
        _thr[2] = LIMIT_MAX_MIN(_thr[2], 100.0, -100.0);
        _thr[3] = LIMIT_MAX_MIN(_thr[3], 100.0, -100.0);

        pca_write_throttle(0, fabsf(_thr[0]));
        pca_write_throttle(1, fabsf(_thr[1]));
        pca_write_throttle(2, fabsf(_thr[2]));
        pca_write_throttle(3, fabsf(_thr[3]));
        
        // Turn on direction.
        static uint32_t prev_thr[4]; // Be used to detect direction change.
        
        if ((prev_thr[0] ^ *(uint32_t*)&_thr[0]) & 0x80000000) {

            gpio_write(PIN_M1_CW,  _thr[0] > 0 ? 1 : 0);
            gpio_write(PIN_M1_CCW, _thr[0] > 0 ? 0 : 1);
        }

        if ((prev_thr[1] ^ *(uint32_t*)&_thr[1]) & 0x80000000) {

            gpio_write(PIN_M2_CW,  _thr[1] > 0 ? 1 : 0);
            gpio_write(PIN_M2_CCW, _thr[1] > 0 ? 0 : 1);
        }
        
        if ((prev_thr[2] ^ *(uint32_t*)&_thr[2]) & 0x80000000) {

            gpio_write(PIN_M3_CW,  _thr[2] > 0 ? 0 : 1);
            gpio_write(PIN_M3_CCW, _thr[2] > 0 ? 1 : 0);
        }

        if ((prev_thr[3] ^ *(uint32_t*)&_thr[3]) & 0x80000000) {

            gpio_write(PIN_M4_CW,  _thr[3] > 0 ? 0 : 1);
            gpio_write(PIN_M4_CCW, _thr[3] > 0 ? 1 : 0);
        }

        prev_thr[0] = *(uint32_t*)&_thr[0];
        prev_thr[1] = *(uint32_t*)&_thr[1];
        prev_thr[2] = *(uint32_t*)&_thr[2];
        prev_thr[3] = *(uint32_t*)&_thr[3];

    } else {
        pca_write_pwm(0, 0);
        pca_write_pwm(1, 0);
        pca_write_pwm(2, 0);
        pca_write_pwm(3, 0);
    }
}

void controller_get_thr_output(float *thr1, float *thr2, float *thr3, float *thr4);

void controller_reset() {
    pca_reset();
    pid_reset(pidsetting_ax);
    pid_reset(pidsetting_ay);
    pid_reset(pidsetting_az);
    pid_reset(pidsetting_avx);
    pid_reset(pidsetting_avy);
    pid_reset(pidsetting_avz);
    pid_reset(pidsetting_va);
    pid_reset(pidsetting_alt);
    gpio_write(PIN_M1_CW, 0);
    gpio_write(PIN_M1_CCW, 0);
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