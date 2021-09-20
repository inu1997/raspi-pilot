#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include <stdbool.h>

extern const char *parameter_keys[];

// Same as mavlink param type
enum PARAMETER_TYPE {
    PARAMETER_TYPE_INT32 = 6,
    PARAMETER_TYPE_FLOAT = 9
};

enum PARAMETER {
    PARAMETER_MTR_PWM_FREQ,
    PARAMETER_MTR_PWM_MAX,
    PARAMETER_MTR_PWM_MIN,
    PARAMETER_MANUAL_AX_MAX,
    PARAMETER_MANUAL_AY_MAX,
    PARAMETER_MANUAL_AVZ_MAX,
    PARAMETER_CALIB_OFFSET_GX,
    PARAMETER_CALIB_OFFSET_GY,
    PARAMETER_CALIB_OFFSET_GZ,
    PARAMETER_CALIB_OFFSET_MX,
    PARAMETER_CALIB_OFFSET_MY,
    PARAMETER_CALIB_OFFSET_MZ,
    PARAMETER_CALIB_SCALE_MX,
    PARAMETER_CALIB_SCALE_MY,
    PARAMETER_CALIB_SCALE_MZ,
    PARAMETER_PID_AX_P,
    PARAMETER_PID_AX_I,
    PARAMETER_PID_AX_D,
    PARAMETER_PID_AX_I_LIMIT,
    PARAMETER_PID_AX_O_LIMIT,
    PARAMETER_PID_AY_P,
    PARAMETER_PID_AY_I,
    PARAMETER_PID_AY_D,
    PARAMETER_PID_AY_I_LIMIT,
    PARAMETER_PID_AY_O_LIMIT,
    PARAMETER_PID_AZ_P,
    PARAMETER_PID_AZ_I,
    PARAMETER_PID_AZ_D,
    PARAMETER_PID_AZ_I_LIMIT,
    PARAMETER_PID_AZ_O_LIMIT,
    PARAMETER_PID_AVX_P,
    PARAMETER_PID_AVX_I,
    PARAMETER_PID_AVX_D,
    PARAMETER_PID_AVX_I_LIMIT,
    PARAMETER_PID_AVX_O_LIMIT,
    PARAMETER_PID_AVY_P,
    PARAMETER_PID_AVY_I,
    PARAMETER_PID_AVY_D,
    PARAMETER_PID_AVY_I_LIMIT,
    PARAMETER_PID_AVY_O_LIMIT,
    PARAMETER_PID_AVZ_P,
    PARAMETER_PID_AVZ_I,
    PARAMETER_PID_AVZ_D,
    PARAMETER_PID_AVZ_I_LIMIT,
    PARAMETER_PID_AVZ_O_LIMIT,
    PARAMETER_PID_VA_P,
    PARAMETER_PID_VA_I,
    PARAMETER_PID_VA_D,
    PARAMETER_PID_VA_I_LIMIT,
    PARAMETER_PID_VA_O_LIMIT,
    PARAMETER_PID_ALT_P,
    PARAMETER_PID_ALT_I,
    PARAMETER_PID_ALT_D,
    PARAMETER_PID_ALT_I_LIMIT,
    PARAMETER_PID_ALT_O_LIMIT
};

int parameter_init();

bool parameter_exist(const char *key);

int parameter_get_count();

int parameter_get_count_no_mutex();

int parameter_get_value(const char *key, void *data);

int parameter_get_value_no_mutex(const char *key, void *data);

int parameter_set_value(const char *key, void *data, bool save);

int parameter_set_value_no_mutex(const char *key, void *data, bool save);

int parameter_get_type(const char *key);

int parameter_get_type_no_mutex(const char *key);

int parameter_get_index(const char *key);

int parameter_get_index_no_mutex(const char *key);

void parameter_lock_mutex();

void parameter_unlock_mutex();

#endif // _PARAMETER_H_
