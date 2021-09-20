#include "calibration.h"

#include "util/parameter.h"
#include "util/logger.h"
#include "util/debug.h"
#include <string.h>

//----- Global sampling variable.
float _sample_mx_max = 0;
float _sample_mx_min = 0;
float _sample_my_max = 0;
float _sample_my_min = 0;
float _sample_mz_max = 0;
float _sample_mz_min = 0;
float _sample_gx[100];
float _sample_gy[100];
float _sample_gz[100];
int _sample_g_index = 0;

//----- Offsets.
float _gx_offset = 0;
float _gy_offset = 0;
float _gz_offset = 0;

// m_real = (m - offset) / m_scale
float _mx_offset = 103.5;
float _my_offset = 76.0;
float _mz_offset = -194.0;
float _mx_scale = 1.197;
float _my_scale = 1.865; 
float _mz_scale = 1.042;


//-----

void calibration_gyro(float gx, float gy, float gz, float *ca_gx, float *ca_gy, float *ca_gz) {
    
    *ca_gx = gx - _gx_offset;
    *ca_gy = gy - _gy_offset;
    *ca_gz = gz - _gz_offset;

}

void calibration_mag(float mx, float my, float mz, float *ca_mx, float *ca_my, float *ca_mz) {

    float tmp_x, tmp_y, tmp_z;
    tmp_x = (mx * _mx_scale) - _mx_offset;
    tmp_y = (my * _my_scale) - _my_offset;
    tmp_z = (mz * _mz_scale) - _mz_offset;
    // tmp_x = (mx - _mx_offset) * _mx_scale;
    // tmp_y = (my - _my_offset) * _my_scale;
    // tmp_z = (mz - _mz_offset) * _mz_scale;
    *ca_mx = tmp_x;
    *ca_my = tmp_y;
    *ca_mz = tmp_z;

}

void calibration_gather_raw_gyro(float gx, float gy, float gz) {
    _sample_gx[_sample_g_index] = gx;
    _sample_gy[_sample_g_index] = gy;
    _sample_gz[_sample_g_index] = gz;
    _sample_g_index = (_sample_g_index + 1) % 100;
    
    float _gx_sum = 0;
    float _gy_sum = 0;
    float _gz_sum = 0;
    int i;
    for(i = 0; i < 100; i++) {
        _gx_sum += _sample_gx[i];
        _gy_sum += _sample_gy[i];
        _gz_sum += _sample_gz[i];
    }
    _gx_offset = _gx_sum / 100;
    _gy_offset = _gy_sum / 100;
    _gz_offset = _gz_sum / 100;
}

void calibration_reset_sample() {
    _sample_mx_max = 0;
    _sample_mx_min = 0;
    _sample_my_max = 0;
    _sample_my_min = 0;
    _sample_mz_max = 0;
    _sample_mz_min = 0;
    memset(_sample_gx, 0, sizeof(_sample_gx));
    memset(_sample_gy, 0, sizeof(_sample_gy));
    memset(_sample_gz, 0, sizeof(_sample_gz));
    _sample_g_index = 0;
}

void calibration_gather_raw_mag(float mx, float my, float mz) {
    // Get max and min
    _sample_mx_max = mx > _sample_mx_max ? mx : _sample_mx_max;
    _sample_my_max = my > _sample_my_max ? my : _sample_my_max;
    _sample_mz_max = mz > _sample_mz_max ? mz : _sample_mz_max;

    _sample_mx_min = mx < _sample_mx_min ? mx : _sample_mx_min;
    _sample_my_min = my < _sample_my_min ? my : _sample_my_min;
    _sample_mz_min = mz < _sample_mz_min ? mz : _sample_mz_min;
    
    // Compute result.
    _mx_offset = (_sample_mx_max + _sample_mx_min) / 2;
    _my_offset = (_sample_my_max + _sample_my_min) / 2;
    _mz_offset = (_sample_mz_max + _sample_mz_min) / 2;
    float avg_delta_x = (_sample_mx_max - _sample_mx_min) / 2;
    float avg_delta_y = (_sample_my_max - _sample_my_min) / 2;
    float avg_delta_z = (_sample_mz_max - _sample_mz_min) / 2;
    float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;
    _mx_scale = avg_delta / avg_delta_x;
    _my_scale = avg_delta / avg_delta_y;
    _mz_scale = avg_delta / avg_delta_z;
}

void calibration_load() {
    LOG("Loading offset.\n");
    
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_GX], &_gx_offset);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_GY], &_gy_offset);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_GZ], &_gz_offset);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_MX], &_mx_offset);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_MY], &_my_offset);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_MZ], &_mz_offset);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_CALIB_SCALE_MX], &_mx_scale);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_CALIB_SCALE_MY], &_my_scale);
    parameter_get_value_no_mutex(parameter_keys[PARAMETER_CALIB_SCALE_MZ], &_mz_scale);
    LOG("Offset of Mag: [%7.2f, %7.2f, %7.2f]\n", _mx_offset, _my_offset, _mz_offset);
    LOG("Scale of Mag: [%7.2f, %7.2f, %7.2f]\n", _mx_scale, _my_scale, _mz_scale);
    LOG("Offset of Gyro: [%7.2f, %7.2f, %7.2f]\n", _gx_offset, _gy_offset, _gz_offset);
}

void calibration_save() {
    LOG("Saving offset.\n");
    
    parameter_set_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_GX], &_gx_offset, false);
    parameter_set_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_GY], &_gy_offset, false);
    parameter_set_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_GZ], &_gz_offset, false);
    parameter_set_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_MX], &_mx_offset, false);
    parameter_set_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_MY], &_my_offset, false);
    parameter_set_value_no_mutex(parameter_keys[PARAMETER_CALIB_OFFSET_MZ], &_mz_offset, false);
    parameter_set_value_no_mutex(parameter_keys[PARAMETER_CALIB_SCALE_MX], &_mx_scale, false);
    parameter_set_value_no_mutex(parameter_keys[PARAMETER_CALIB_SCALE_MY], &_my_scale, false);
    parameter_set_value_no_mutex(parameter_keys[PARAMETER_CALIB_SCALE_MZ], &_mz_scale, true);
    
    LOG("Offset of Mag: [%7.2f, %7.2f, %7.2f]\n", _mx_offset, _my_offset, _mz_offset);
    LOG("Scale of Mag: [%7.2f, %7.2f, %7.2f]\n", _mx_scale, _my_scale, _mz_scale);
    LOG("Offset of Gyro: [%7.2f, %7.2f, %7.2f]\n", _gx_offset, _gy_offset, _gz_offset);
    calibration_reset_sample();
}