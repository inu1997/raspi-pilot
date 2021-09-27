#include "imu.h"
#include "calibration.h"

#include <stdint.h>
#include <unistd.h>

#include "driver/mpu6050.h"
#include "driver/ak8963.h"
#include "driver/hmc5883l.h"

#include "util/macro.h"
#include "util/logger.h"
#include "util/debug.h"

#include "util/filter/sma_filter.h"

//----- Configurations.
#define COMPLEMENTARY_ALPHA 0.98

#define UPDATE_METHOD_MADGWICK
// #define UPDATE_METHOD_COMPLEMENTARY

#define SMA_BUFFER_LENGTH_A 2
#define SMA_BUFFER_LENGTH_G 2
#define SMA_BUFFER_LENGTH_M 3

#define IMU_CFG_MPU_DLPF 0
#define IMU_CFG_MPU_GYRO_FS MPU_GYRO_FS_2000_DPS
#define IMU_CFG_MPU_GYRO_FCHOICE 0
#define IMU_CFG_MPU_ACCEL_FS MPU_ACCEL_FS_8_G
#define IMU_CFG_MPU_ACCEL_FCHOICE 0
#define IMU_CFG_MPU_ACCEL_DLPF 0
#define IMU_CFG_AK_16_BIT

float _raw_a[3]; // Raw/Uncalibrated measurements from Accelerometer. In X,Y,Z order.
float _raw_g[3]; // Raw/Uncalibrated measurements from Gyroscope(Degree/s). In X,Y,Z order.
float _raw_m[3]; // Raw/Uncalibrated measurements from Magnetometer. In X,Y,Z order.

float _est_a[3]; // Filtered/Calibrated measurements from accelerometer. In X,Y,Z order.
float _est_g[3]; // Filtered/Calibrated measurements from gyroscope(Degree/s). In X,Y,Z order.
float _est_m[3]; // Filtered/Calibrated measurements from magnetometer. In X,Y,Z order.

float _scale_accel; // Accelerometer sensitivity scale.
float _scale_gyro; // Gyro  sensitivity scale.

struct SMAFilter *_sma_a[3]; // Simeple Moving Average Filter for Accelerometer.
struct SMAFilter *_sma_g[3]; // Simeple Moving Average Filter for Gyro.
struct SMAFilter *_sma_m[3]; // Simeple Moving Average Filter for Megnetometer.

bool _mag_data_updated; // True if magnetometer is updated in this loop else false.

int imu_init_mpu();

int imu_init_ak();

/**
 * @brief Initiate IMU sensors.
 * 
 * @return 0 if success else -1.
 */
int imu_init() {
    LOG("Initiating IMU.\n");

    LOG("Initiating modules.\n");
    if (imu_init_mpu() != 0) {
        LOG_ERROR("Failed to initiate MPU.\n");
        return -1;
    }
    if (imu_init_ak() != 0) {
        LOG_ERROR("Failed to initiate AK.\n");
        return -1;
    }

    calibration_load();
    
    LOG("Initiating variables.\n");
    int i;
    for(i = 0; i < 3; i++) {
        _est_a[i] = _est_g[i] = _est_m[i] = _raw_a[i] = _raw_g[i] = _raw_m[i] = 0;
        
        _sma_a[i] = sma_init(SMA_BUFFER_LENGTH_A);

        _sma_g[i] = sma_init(SMA_BUFFER_LENGTH_G);
        
        _sma_m[i] = sma_init(SMA_BUFFER_LENGTH_M);
    }
    _mag_data_updated = false;
    LOG("Done.\n");
    return 0;
}

/**
 * @brief Update all sensor readings.
 * 
 */
void imu_update() {
    // Read new value
    int16_t a_read[3];
    int16_t g_read[3];
    int16_t m_read[3];
    float m_read_calibrated[3];
    
    mpu_read_all(
        &a_read[0], &a_read[1], &a_read[2],
        &g_read[0], &g_read[1], &g_read[2],
        &_mag_data_updated,
        &m_read[1], &m_read[0], &m_read[2]);

    // mpu_read_accel(&a_read[0], &a_read[1], &a_read[2]);
    // mpu_read_gyro(&g_read[0], &g_read[1], &g_read[2]);

    // Update raw values, those variables are for debugging.
    _raw_a[0] = (float)a_read[0] / _scale_accel;
    _raw_a[1] = (float)a_read[1] / _scale_accel;
    _raw_a[2] = (float)a_read[2] / _scale_accel;
    _raw_g[0] = (float)g_read[0] / _scale_gyro * TO_RAD;
    _raw_g[1] = (float)g_read[1] / _scale_gyro * TO_RAD;
    _raw_g[2] = (float)g_read[2] / _scale_gyro * TO_RAD;
    
    if (_mag_data_updated) {
        _raw_m[0] = (float)m_read[0];
        _raw_m[1] = (float)m_read[1];
        _raw_m[2] = (float)m_read[2];
        calibration_mag(
            m_read[0],
            m_read[1],
            m_read[2],
            &m_read_calibrated[0],
            &m_read_calibrated[1],
            &m_read_calibrated[2]
        );
    }
    // Filter
    int i;
    for(i = 0; i < 3; i++) {
        _est_a[i] = sma_update(_sma_a[i], _raw_a[i]);
        _est_g[i] = sma_update(_sma_g[i], _raw_g[i]);
        if (_mag_data_updated) {
            _est_m[i] = sma_update(_sma_m[i], m_read_calibrated[i]);
        }
    }
}

/**
 * @brief Check if Magnetometer updated.
 * 
 * @return True if updated else false.
 */
bool imu_mag_data_updated() {
    return _mag_data_updated;
}

float imu_get_ax() {return _est_a[0];}
float imu_get_ay() {return _est_a[1];}
float imu_get_az() {return _est_a[2];}
float imu_get_gx() {return _est_g[0];}
float imu_get_gy() {return _est_g[1];}
float imu_get_gz() {return _est_g[2];}
float imu_get_mx() {return _est_m[0];}
float imu_get_my() {return _est_m[1];}
float imu_get_mz() {return _est_m[2];}

float imu_get_raw_ax() {return _raw_a[0];};
float imu_get_raw_ay() {return _raw_a[1];};
float imu_get_raw_az() {return _raw_a[2];};
float imu_get_raw_gx() {return _raw_g[0];};
float imu_get_raw_gy() {return _raw_g[1];};
float imu_get_raw_gz() {return _raw_g[2];};
float imu_get_raw_mx() {return _raw_m[0];};
float imu_get_raw_my() {return _raw_m[1];};
float imu_get_raw_mz() {return _raw_m[2];};

//-----

/**
 * @brief Initate MPU module.
 * 
 * @return 0 if success else -1.
 */
int imu_init_mpu() {
    LOG("Initiating MPU.\n");
    if (mpu_init() != 0) {
        LOG_ERROR("Failed to init mpu.\n");
        return -1;
    }
    usleep(10000);
    if (mpu_set_dlpf(IMU_CFG_MPU_DLPF) != 0) {
        LOG_ERROR("Failed to set DLPF.\n");
        return -1;
    }
    usleep(10000);
    if ((_scale_gyro = mpu_set_gyro_fullscale(IMU_CFG_MPU_GYRO_FS)) == 0.0f) {
        LOG_ERROR("Failed to set gyro fullscale.\n");
        return -1;
    }
    usleep(10000);
    if (mpu_set_gyro_fchoice(IMU_CFG_MPU_GYRO_FCHOICE) != 0) {
        LOG_ERROR("Failed to set gyro fchoice.\n");
        return -1;
    }
    usleep(10000);
    if ((_scale_accel = mpu_set_accel_fullscale(IMU_CFG_MPU_ACCEL_FS)) == 0.0f) {
        LOG_ERROR("Failed to set accel fullscale.\n");
        return -1;
    }
    usleep(10000);
    if (mpu_set_accel_fchoice(IMU_CFG_MPU_ACCEL_FCHOICE != 0)) {
        LOG_ERROR("Failed to set accel fchoice.\n");
        return -1;
    }
    usleep(10000);
    if (mpu_set_accel_dlpf(IMU_CFG_MPU_ACCEL_DLPF) != 0) {
        LOG_ERROR("Failed to set accel dlpf.\n");
        return -1;
    }
    usleep(10000);
    LOG("Done.\n");
    return 0;
}

/**
 * @brief Initate AK module.
 * 
 * @return 0 if success else -1.
 */
int imu_init_ak() {
    LOG("Initiating AK8963.\n");
    
    if (mpu_enable_master_mode() != 0) {
        LOG_ERROR("Failed to enable master mode.\n");
        return -1;
    }
    
    usleep(10000);
    if (ak_init() != 0) {
        LOG_ERROR("Failed to init ak.\n");
        return -1;
    }
    usleep(10000);
#ifdef IMU_CFG_AK_16_BIT
    if (ak_set_16_bit() != 0) {
        LOG_ERROR("Failed to set 16 bit mode.\n");
        return -1;
    }
#else
    if (ak_set_14_bit() != 0) {
        LOG_ERROR("Failed to set 14 bit mode.\n");
        return -1;
    }
#endif // IMU_CFG_AK_16_BIT
    usleep(10000);

    if (ak_set_mode(AK_MODE_CONTINUOUS_MEASUREMENT_100HZ) != 0) {
        LOG_ERROR("Failed to set mode.\n");
        return -1;
    }
    usleep(10000);
    LOG("Done.\n");
    return 0;
}