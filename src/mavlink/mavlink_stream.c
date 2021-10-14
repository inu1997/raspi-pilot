#include "mavlink_stream.h"
#include "mavlink_main.h"

#include "c_library_v2/standard/mavlink.h"

#include "camera/camera.h"
#include "measurement/measurement.h"
#include "pilot/pilot.h"

#include "util/logger.h"
#include "util/debug.h"
#include "util/tv.h"
#include "util/macro.h"
#include "util/system/scheduler.h"

#include <pthread.h>
#include <unistd.h>


struct TaskBlock {
    void (*func)();
    float hz;
    struct timeval tv;
};

#define TIMED_TASK(_func, _hz) {.func = _func, .hz = _hz, .tv = TV_INITIALIZER}

#define DO_TIMED_TASK(tv, hz, func) do { \
    struct timeval now; \
    gettimeofday(&now, NULL); \
    if(tv_get_diff_msec_ul(&tv, &now) >= 1000 / hz) { \
        gettimeofday(&tv, NULL); \
        func(); \
    } \
} while(0);

pthread_t _stream_thread;

void *mavlink_stream_handler(void *);

//-----

int mavlink_init_stream() {
    LOG("Initiating MAVLink stream.\n");
    
    if (pthread_create(&_stream_thread, NULL, mavlink_stream_handler, NULL) != 0) {
        LOG_ERROR("Failed to create thread.\n");
        return -1;
    }

    LOG("Done.\n");
    return 0;
}

//-----

void mavlink_stream_hb_auto_pilot();
void mavlink_stream_hb_camera();
void mavlink_stream_hb_imu();
void mavlink_stream_hb_battery();
void mavlink_stream_attitude();
void mavlink_stream_sensor();
void mavlink_stream_battery();
void mavlink_stream_camera_capture_status();

void (*tasks[])() = {
    mavlink_stream_hb_auto_pilot,
    mavlink_stream_hb_camera,
    // mavlink_stream_hb_imu,
    // mavlink_stream_hb_battery,
    mavlink_stream_attitude,
    mavlink_stream_sensor,
    mavlink_stream_battery,
    mavlink_stream_camera_capture_status
};

//-----

void *mavlink_stream_handler(void *arg) {
    int i = 0;
    while (1) {
        tasks[i]();
    
        usleep(10000);

        i = (i + 1) % (sizeof(tasks) / sizeof(void (*)()));
    }
}

//-----

void mavlink_stream_hb_auto_pilot() {
    static struct timeval tv = TV_INITIALIZER;
    const float hz = 1;
    struct timeval now;
    gettimeofday(&now, NULL);
    if (tv_get_diff_msec_ul(&tv, &now) < 1000 / hz) {
        return;
    }
    gettimeofday(&tv, NULL);
    
    //-----
    mavlink_message_t msg;
    
    mavlink_msg_heartbeat_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_AUTOPILOT1,
        MAVLINK_COMM_0,
        &msg,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        pilot_get_mode(),
        0,
        MAV_STATE_ACTIVE);

    MAVLINK_SEND(&msg);
}

void mavlink_stream_hb_camera() {
    static struct timeval tv = TV_INITIALIZER;
    const float hz = 1;
    struct timeval now;
    gettimeofday(&now, NULL);
    if (tv_get_diff_msec_ul(&tv, &now) < 1000 / hz) {
        return;
    }
    gettimeofday(&tv, NULL);
    
    //-----
    mavlink_message_t msg;
    
    mavlink_msg_heartbeat_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_CAMERA,
        MAVLINK_COMM_0,
        &msg,
        MAV_TYPE_CAMERA,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        MAV_STATE_ACTIVE);

    MAVLINK_SEND(&msg);
}

void mavlink_stream_hb_imu() {
    static struct timeval tv = TV_INITIALIZER;
    const float hz = 1;
    struct timeval now;
    gettimeofday(&now, NULL);
    if (tv_get_diff_msec_ul(&tv, &now) < 1000 / hz) {
        return;
    }
    gettimeofday(&tv, NULL);
    
    //-----
    mavlink_message_t msg;
    
    mavlink_msg_heartbeat_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_IMU,
        MAVLINK_COMM_0,
        &msg,
        0,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        MAV_STATE_ACTIVE);

    MAVLINK_SEND(&msg);
}

void mavlink_stream_hb_battery() {
    static struct timeval tv = TV_INITIALIZER;
    const float hz = 1;
    struct timeval now;
    gettimeofday(&now, NULL);
    if (tv_get_diff_msec_ul(&tv, &now) < 1000 / hz) {
        return;
    }
    gettimeofday(&tv, NULL);
    
    //-----
    mavlink_message_t msg;
    
    mavlink_msg_heartbeat_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_BATTERY,
        MAVLINK_COMM_0,
        &msg,
        0,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        MAV_STATE_ACTIVE);

    MAVLINK_SEND(&msg);
}
void mavlink_stream_attitude() {
    static struct timeval tv = TV_INITIALIZER;
    const float hz = 5;
    struct timeval now;
    gettimeofday(&now, NULL);
    if (tv_get_diff_msec_ul(&tv, &now) < 1000 / hz) {
        return;
    }
    gettimeofday(&tv, NULL);
    
    //-----
    mavlink_message_t msg;
    
    mavlink_msg_attitude_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_AUTOPILOT1,
        MAVLINK_COMM_0,
        &msg,
        tv_get_msec_since_epoch(),
        ahrs_get_roll(),
        ahrs_get_pitch(),
        -ahrs_get_yaw_heading(),
        imu_get_gx(),
        imu_get_gy(),
        imu_get_gz());

    MAVLINK_SEND(&msg);
}

void mavlink_stream_sensor() {
    static struct timeval tv = TV_INITIALIZER;
    const float hz = 5;
    struct timeval now;
    gettimeofday(&now, NULL);
    if (tv_get_diff_msec_ul(&tv, &now) < 1000 / hz) {
        return;
    }
    gettimeofday(&tv, NULL);
    
    //-----
    mavlink_message_t msg;
    mavlink_msg_hil_sensor_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_AUTOPILOT1,
        MAVLINK_COMM_0,
        &msg,
        tv_get_msec_since_epoch(),
        imu_get_ax(),
        imu_get_ay(),
        imu_get_az(),
        imu_get_gx(),
        imu_get_gy(),
        imu_get_gz(),
        imu_get_mx(),
        imu_get_my(),
        imu_get_mz(),
        barometer_get_pressure(),
        barometer_get_pressure_diff(),
        barometer_get_altitude(),
        barometer_get_temperature(),
        0x80000000,
        0);

    MAVLINK_SEND(&msg);
}

void mavlink_stream_battery() {
    static struct timeval tv = TV_INITIALIZER;
    const float hz = 0.2;
    struct timeval now;
    gettimeofday(&now, NULL);
    if (tv_get_diff_msec_ul(&tv, &now) < 1000 / hz) {
        return;
    }
    gettimeofday(&tv, NULL);
    
    //-----
    mavlink_message_t msg;
    mavlink_msg_battery_status_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_AUTOPILOT1,
        MAVLINK_COMM_0,
        &msg,
        0,
        MAV_BATTERY_FUNCTION_ALL,
        MAV_BATTERY_TYPE_LIPO,
        INT16_MAX,
        (uint16_t*){0, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX , UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX},
        -1,
        -1,
        -1,
        -1,
        -1,
        MAV_BATTERY_CHARGE_STATE_OK,
        0,
        0,
        0);

    MAVLINK_SEND(&msg);
}

void mavlink_stream_camera_capture_status() {
    if (!(camera_is_image_capturing() || camera_is_video_capturing())) {
        return;
    }

    static struct timeval tv = TV_INITIALIZER;
    const float hz = 1;
    struct timeval now;
    gettimeofday(&now, NULL);
    if (tv_get_diff_msec_ul(&tv, &now) < camera_get_status_interval_msec()) {
        return;
    }
    gettimeofday(&tv, NULL);
    
    //-----
    mavlink_message_t msg;
    mavlink_msg_camera_capture_status_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_CAMERA,
        MAVLINK_COMM_0,
        &msg,
        tv_get_msec_since_epoch(),
        camera_get_image_capture_interval() != 0.0 ? (camera_is_image_capturing() ? 3 : 2) : (camera_is_image_capturing() ? 1 : 0),
        camera_is_video_capturing() ? 1 : 0,
        camera_get_image_capture_interval(),
        0,
        0,
        camera_get_image_capture_number());

    MAVLINK_SEND(&msg);
}