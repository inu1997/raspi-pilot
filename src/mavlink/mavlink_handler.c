#include "mavlink_handler.h"
#include "mavlink_handler_cmd.h"
#include "mavlink_main.h"
#include "mavlink_util.h"

#include "c_library_v2/standard/mavlink.h"

#include "pilot/pilot.h"

#include "util/parameter.h"
#include "util/logger.h"
#include "util/tv.h"
#include "util/debug.h"

#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>

// Only one message can be handled between threads.
pthread_mutex_t _handler_mutex = PTHREAD_MUTEX_INITIALIZER;

int mavlink_handle_system_time();

int mavlink_handle_manual_control();

int mavlink_handle_serial_control();

int mavlink_handle_mission_request_list();

int mavlink_handle_param_set();

int mavlink_handle_param_request_read();

int mavlink_handle_param_request_list();

//-----
mavlink_message_t _current_msg;

/**
 * @brief  Handle the incoming message, execute it.
 * 
 * @param msg
 *      Message to handle.
 * @return -1 if meessage is not supported else length of respond buffer.
 */
int mavlink_handler_handle_msg(mavlink_message_t *msg) {

    pthread_mutex_lock(&_handler_mutex);
    // Copy message to current_msg to let handler functions has better way to refer to original message.
    memcpy(&_current_msg, msg, sizeof(mavlink_message_t));

    int ret = -1;
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            // Do nothing.
            ret = 0;
        break;
        case MAVLINK_MSG_ID_SYSTEM_TIME:
            DEBUG("%s.\n", TO_STRING(MAVLINK_MSG_ID_SYSTEM_TIME));
            ret = mavlink_handle_system_time();
        break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            // DEBUG("%s.\n", TO_STRING(MAVLINK_MSG_ID_MANUAL_CONTROL));
            ret = mavlink_handle_manual_control();
        break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            DEBUG("%s.\n", TO_STRING(MAVLINK_MSG_ID_COMMAND_LONG));
            ret = mavlink_handle_cmd_long();
        break;
        case MAVLINK_MSG_ID_SERIAL_CONTROL:
            DEBUG("%s.\n", TO_STRING(MAVLINK_MSG_ID_SERIAL_CONTROL));
            ret = mavlink_handle_serial_control();
        break;
        case MAVLINK_MSG_ID_PARAM_SET:
            DEBUG("%s.\n", TO_STRING(MAVLINK_MSG_ID_PARAM_SET));
            ret = mavlink_handle_param_set();
        break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            DEBUG("%s.\n", TO_STRING(MAVLINK_MSG_ID_PARAM_REQUEST_READ));
            ret = mavlink_handle_param_request_read();
        break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            DEBUG("%s.\n", TO_STRING(MAVLINK_MSG_ID_PARAM_REQUEST_LIST));
            ret = mavlink_handle_param_request_list();
        break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
            DEBUG("%s.\n", TO_STRING(MAVLINK_MSG_ID_MISSION_REQUEST_LIST));
            ret = mavlink_handle_mission_request_list();
        break;
        default:
            LOG_ERROR("Unsupported message(id: %d).\n", msg->msgid);
    }
    
    pthread_mutex_unlock(&_handler_mutex);
    return ret;
}

/**
 * @brief Get the original message which is currently handling.
 * 
 * @return The original message.
 */
const mavlink_message_t *mavlink_handler_get_current_msg() {
    return &_current_msg;
}

//----- Handler function implementation.

/**
 * @brief Handle system time message.
 * 
 * @param msg 
 *      MAVLink message received.
 * @return 0 if success else -1.
 */
int mavlink_handle_system_time() {

    mavlink_system_time_t decoded;
    mavlink_msg_system_time_decode(
        mavlink_handler_get_current_msg(),
        &decoded);

    LOG(
        "System time: %u, Boot ms: %u.\n",
        decoded.time_unix_usec,
        decoded.time_boot_ms);
    return 0;
}

/**
 * @brief handle manual control message.
 * 
 * @param msg
 *      MAVLink message received.
 * @return 0 if success else -1.
 */
int mavlink_handle_manual_control() {

    mavlink_manual_control_t decoded;
    mavlink_msg_manual_control_decode(
        mavlink_handler_get_current_msg(),
        &decoded);

    if (decoded.target != MAVLINK_SYS_ID) {
        DEBUG("Not target, target is %d\n", decoded.target);
        return -1;
    }

    // DEBUG("Target: %d, X: %d, Y: %d, Z: %d, R: %d. Buttons: 0x%04x\n", decoded.target, decoded.x, decoded.y, decoded.z, decoded.r, decoded.buttons);
    // map values to controller.

    return 0;
}
/**
 * @brief Proccess serial control.(Shell)
 * 
 * @param msg
 *      MAVLink message received.
 * @return Length of buf. 0 if not supported.
 */
int mavlink_handle_serial_control() {

    mavlink_serial_control_t decoded;
    mavlink_msg_serial_control_decode(
        mavlink_handler_get_current_msg(),
        &decoded);
        
    DEBUG("count: %u, data: %s, device: %u, flags: 0x%02x, timeout: %u, br: %u.\n",decoded.count, decoded.data, decoded.device, decoded.flags, decoded.timeout, decoded.baudrate);
    return 0;
}

/**
 * @brief Handle mission request list message.
 * 
 * @param msg 
 *      MAVLink message received.
 * @return int 
 */
int mavlink_handle_mission_request_list() {
    mavlink_mission_request_list_t decoded;
    mavlink_msg_mission_request_list_decode(
        mavlink_handler_get_current_msg(),
        &decoded);

    int ret = -1;
    if (decoded.target_system == MAVLINK_SYS_ID) {
        mavlink_message_t msg;
        mavlink_msg_mission_count_pack_chan(
            MAVLINK_SYS_ID,
            MAV_COMP_ID_AUTOPILOT1,
            MAVLINK_COMM_0,
            &msg,
            mavlink_handler_get_current_msg()->sysid,
            mavlink_handler_get_current_msg()->compid,
            0,
            MAV_MISSION_TYPE_MISSION);
        MAVLINK_SEND(&msg);
        ret = 0;
    }

    return ret;
}

/**
 * @brief Handle param set message.
 * 
 * @param msg 
 *      MAVLink message received.
 * @return 0 if success else -1.
 */
int mavlink_handle_param_set() {
    int ret = -1;
    mavlink_param_set_t decoded;
    mavlink_msg_param_set_decode(
        mavlink_handler_get_current_msg(),
        &decoded);

    if (decoded.target_system == MAVLINK_SYS_ID) {
        ret = parameter_set_value(decoded.param_id, &decoded.param_value, true);

        // Send ack.
        mavlink_message_t msg;

        // mavlink_msg_param_ack_transaction_pack_chan(
        //     MAVLINK_SYS_ID,
        //     MAV_COMP_ID_AUTOPILOT1,
        //     MAVLINK_COMM_0,
        //     &msg,
        //     msg.sysid,
        //     msg.compid,
        //     decoded.param_id,
        //     decoded.param_value,
        //     decoded.param_type,
        //     ret == 0 ? PARAM_ACK_ACCEPTED : PARAM_ACK_FAILED
        // );
        // MAVLINK_SEND(&msg);
        
        // Send back parameter again.
        mavlink_send_parameter(decoded.param_id);
    }

    return ret;
}

/**
 * @brief Handle param request read message.
 * 
 * @param msg 
 *      MAVLink message received.
 * @return 0 if success else -1.
 */
int mavlink_handle_param_request_read() {
    mavlink_param_request_read_t decoded;
    mavlink_msg_param_request_read_decode(
        mavlink_handler_get_current_msg(),
        &decoded);

    if (decoded.target_system != MAVLINK_SYS_ID) {
        DEBUG("Not requesting ours.\n");
        return -1;
    }

    int ret = -1;
    
    DEBUG(
        "Got request param \"%s\", %d,\n",
        decoded.param_id,
        decoded.param_index);
    struct Parameter *p;
    
    ret = mavlink_send_parameter(decoded.param_id);

    return ret;
}

/**
 * @brief Handle param request list message
 * 
 * @param msg 
 *      MAVLink message received.
 * @return 0 if success else -1.
 */
int mavlink_handle_param_request_list() {
    return mavlink_send_parameter_list();
}