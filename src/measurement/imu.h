#ifndef _IMU_H_
#define _IMU_H_

#include <stdbool.h>

int imu_init();

void imu_update();

bool imu_mag_data_updated();

float imu_get_ax();
float imu_get_ay();
float imu_get_az();
float imu_get_gx();
float imu_get_gy();
float imu_get_gz();
float imu_get_mx();
float imu_get_my();
float imu_get_mz();

float imu_get_raw_ax();
float imu_get_raw_ay();
float imu_get_raw_az();
float imu_get_raw_gx();
float imu_get_raw_gy();
float imu_get_raw_gz();
float imu_get_raw_mx();
float imu_get_raw_my();
float imu_get_raw_mz();

#endif // _IMU_H_
