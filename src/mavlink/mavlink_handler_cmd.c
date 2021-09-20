#include "mavlink_handler_cmd.h"
#include "mavlink_handler.h"

#include "mavlink_main.h"

#include "c_library_v2/standard/mavlink.h"

#include "pilot/pilot.h"

#include "util/parameter.h"
#include "util/logger.h"
#include "util/tv.h"
#include "util/debug.h"

#include <unistd.h>

int mavlink_handle_cmd_request_protocol_version();

int mavlink_handle_cmd_request_autopilot_capabilities();

int mavlink_handle_cmd_request_camera_information();

int mavlink_handle_cmd_request_camera_capture_status();

int mavlink_handle_cmd_request_camera_setting();

int mavlink_handle_cmd_request_storage_information();

int mavlink_handle_cmd_arm_disarm();

//-----

/**
 * @brief Proccess command long.
 * 
 * @param msg
 *      MAVLink message received.
 * @return -1 if the command is not supported. 
 */
int mavlink_handle_cmd_long() {
    mavlink_command_long_t decoded;
    mavlink_msg_command_long_decode(
        mavlink_handler_get_current_msg(),
        &decoded);
    DEBUG(
        "Got command: %u, confirmation: %u, Param: %f, %f, %f, %f, %f, %f, %f, target_comp: %u, target_sys: %u\n",
        decoded.command,
        decoded.confirmation,
        decoded.param1,
        decoded.param2,
        decoded.param3,
        decoded.param4,
        decoded.param5,
        decoded.param6,
        decoded.param7,
        decoded.target_component,
        decoded.target_system);

    uint8_t result = MAV_RESULT_ACCEPTED;
    uint8_t progress = 100;
    int32_t result_param = 0;

    int ret = -1;

    switch (decoded.command) {
        case MAV_CMD_DO_DIGICAM_CONTROL:
            ret = DEBUG("%s.\n", MAV_CMD_DO_DIGICAM_CONTROL);
        break;
        case MAV_CMD_DO_DIGICAM_CONFIGURE:
            ret = DEBUG("%s.\n", TO_STRING(MAV_CMD_DO_DIGICAM_CONFIGURE));
        break;
        case MAV_CMD_REQUEST_PROTOCOL_VERSION:
            DEBUG("%s.\n", TO_STRING(MAV_CMD_REQUEST_PROTOCOL_VERSION));
            ret = mavlink_handle_cmd_request_protocol_version();
        break;
        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            DEBUG("%s.\n", TO_STRING(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES));
            ret = mavlink_handle_cmd_request_autopilot_capabilities();
        break;
        case MAV_CMD_REQUEST_CAMERA_INFORMATION:
            DEBUG("%s.\n", TO_STRING(MAV_CMD_REQUEST_CAMERA_INFORMATION));
            ret = mavlink_handle_cmd_request_camera_information();
        break;
        case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            DEBUG("%s.\n", TO_STRING(MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS));
            ret = mavlink_handle_cmd_request_camera_capture_status();
        break;
        case MAV_CMD_REQUEST_CAMERA_SETTINGS:
            DEBUG("%s.\n", TO_STRING(MAV_CMD_REQUEST_CAMERA_SETTINGS));
            ret = mavlink_handle_cmd_request_camera_setting();
        break;
        case MAV_CMD_REQUEST_STORAGE_INFORMATION:
            DEBUG("%s.\n", TO_STRING(MAV_CMD_REQUEST_STORAGE_INFORMATION));
            ret = mavlink_handle_cmd_request_storage_information();
        break;
        case MAV_CMD_COMPONENT_ARM_DISARM:
            DEBUG("%s.\n", TO_STRING(MAV_CMD_COMPONENT_ARM_DISARM));
            ret = mavlink_handle_cmd_arm_disarm();
        break;
        default:
            result = MAV_RESULT_UNSUPPORTED;
            progress = UINT8_MAX;
            LOG_ERROR(
                "Unsupported command(%d).\n",
                decoded.command);

    }
    
    mavlink_message_t response;
    mavlink_msg_command_ack_pack_chan(
        1,
        decoded.target_component,
        MAVLINK_COMM_0,
        &response,
        decoded.command,
        result,
        progress,
        result_param,
        mavlink_handler_get_current_msg()->sysid,
        mavlink_handler_get_current_msg()->compid);

    MAVLINK_SEND(&response);

    return ret;
}

//-----
int mavlink_handle_cmd_request_protocol_version() {
    mavlink_message_t new_msg;

    mavlink_msg_protocol_version_pack_chan(
        1,
        MAV_COMP_ID_AUTOPILOT1,
        MAVLINK_COMM_0,
        &new_msg,
        mavlink_get_proto_version(MAVLINK_COMM_0),
        1,
        mavlink_get_proto_version(MAVLINK_COMM_0),
        (uint8_t*){0, 0, 0, 0, 0, 0, 0, 0},
        (uint8_t*){0, 0, 0, 0, 0, 0, 0, 0});

    MAVLINK_SEND(&new_msg);
    return 0;
}

int mavlink_handle_cmd_request_autopilot_capabilities() {
    mavlink_message_t new_msg;
    mavlink_msg_autopilot_version_pack_chan(
        1,
        MAV_COMP_ID_AUTOPILOT1,
        MAVLINK_COMM_0,
        &new_msg,
        MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
        MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
        MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT | 
        MAV_PROTOCOL_CAPABILITY_PARAM_UNION,
        1,
        1,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        getuid(),
        0
    );
    
    MAVLINK_SEND(&new_msg);
    return 0;
}

int mavlink_handle_cmd_request_camera_information() {
    mavlink_message_t new_msg;
    mavlink_msg_camera_information_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_CAMERA,
        MAVLINK_COMM_0,
        &new_msg,
        tv_get_msec_since_epoch(),
        "Null",
        "0001",
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE|
        CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
        CAMERA_CAP_FLAGS_CAPTURE_VIDEO ,
        0,
        "");

    MAVLINK_SEND(&new_msg);
    return 0;
}

int mavlink_handle_cmd_request_camera_capture_status() {
    mavlink_message_t new_msg;
    mavlink_msg_camera_capture_status_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_CAMERA,
        MAVLINK_COMM_0,
        &new_msg,
        tv_get_msec_since_epoch(),
        0,
        0,
        0,
        0,
        0,
        0);
    
    MAVLINK_SEND(&new_msg);
    return 0;
}

int mavlink_handle_cmd_request_camera_setting() {
    mavlink_message_t new_msg;
    mavlink_msg_camera_settings_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_CAMERA,
        MAVLINK_COMM_0,
        &new_msg,
        tv_get_msec_since_epoch(),
        CAMERA_MODE_IMAGE,
        0,
        0);
    
    MAVLINK_SEND(&new_msg);
    return 0;
}

int mavlink_handle_cmd_request_storage_information() {
    mavlink_message_t new_msg;
    mavlink_msg_storage_information_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_CAMERA,
        MAVLINK_COMM_0,
        &new_msg,
        tv_get_msec_since_epoch(),
        1,
        0,
        STORAGE_STATUS_NOT_SUPPORTED,
        0,
        0,
        0,
        0,
        0,
        STORAGE_TYPE_UNKNOWN,
        "null");

    MAVLINK_SEND(&new_msg);
    return 0;
}

/**
 * @brief Handle arm/disarm function
 * 
 * @param cmd 
 * @return int 
 */
int mavlink_handle_cmd_arm_disarm() {

    mavlink_command_long_t decoded;
    mavlink_msg_command_long_decode(
        mavlink_handler_get_current_msg(),
        &decoded);

    int ret = -1;
    if (decoded.target_system != MAVLINK_SYS_ID) {
        DEBUG("Not targetting this UAV. The target is %d.\n", decoded.target_system);
        return -1;
    }
    if (decoded.param1 == 1.0) {
        DEBUG("Arm!\n");
        ret = pilot_arm();
    } else if (decoded.param1 == 0.0) {
        DEBUG("Disarm!\n");
        ret = pilot_disarm();
    }

    return ret;
}