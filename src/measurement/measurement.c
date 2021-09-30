#include "measurement.h"
#include "calibration.h"
#include "util/logger.h"

/**
 * @brief This function will initiate all modules which are able to initiate in measurement.
 * 
 * @return 0 if success else -1.
 */
int measurement_init() {
    if (imu_init() != 0) {
        LOG_ERROR("Failed to initiate IMU.\n");
        return -1;
    }
    if (barometer_init() != 0) {
        LOG_ERROR("Failed to initiate Altitude.\n");
        return -1;
    }
    if (battery_init() != 0) {
        LOG_ERROR("Failed to initiate Battery.\n");
        return -1;
    }
    if (ahrs_init() != 0) {
        LOG_ERROR("Failed to initiate AHRS.\n");
        return -1;
    }

    return 0;
}

/**
 * @brief This function will update all modules which are able to update in measurement.
 */
void measurement_update() {
    barometer_update();
    
    imu_update();
    
    if(imu_mag_data_updated()) {
        ahrs_update_9(
            imu_get_ax(),
            imu_get_ay(),
            imu_get_az(),
            imu_get_gx(),
            imu_get_gy(),
            imu_get_gz(),
            imu_get_mx(),
            imu_get_my(),
            imu_get_mz()
        );
    } else {
        ahrs_update_6(
            imu_get_ax(),
            imu_get_ay(),
            imu_get_az(),
            imu_get_gx(),
            imu_get_gy(),
            imu_get_gz()
        );
    }
    if (calibration_gyro_gathering_is_enabled()) {
        calibration_gather_raw_gyro(
            imu_get_raw_gx(),
            imu_get_raw_gy(),
            imu_get_raw_gz());
    }
    if (calibration_mag_gathering_is_enabled()) {
        if (imu_mag_data_updated()) {
            calibration_gather_raw_mag(
                imu_get_raw_mx(),
                imu_get_raw_my(),
                imu_get_raw_mz());
        }
    }
    battery_update();
}
