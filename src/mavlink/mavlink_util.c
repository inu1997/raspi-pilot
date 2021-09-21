#include "mavlink_util.h"
#include "mavlink_main.h"

#include "c_library_v2/standard/mavlink.h"

#include "util/logger.h"
#include "util/debug.h"
#include "util/parameter.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

/**
 * @brief MAVLink print string using status text message.
 * 
 * @param severity
 *      Severity of this message.
 * @param fmt 
 *      Format of string.
 * @param ... 
 *      va_args
 * @return -1 if fail else the number of characters print.
 */
int mavlink_printf(uint8_t severity, const char *fmt, ...) {
    char str[128];
    // Prepare string.
    va_list args;
    va_start(args, fmt);
    int length = vsprintf(str, fmt, args);
    va_end(args);

    int ret;
    // Check.
    if ((ret = strlen(str)) > 50) {
        LOG_ERROR("String must be less than 50 characters.\n");
        return -1;
    }
    
    // Pack and send.
    mavlink_message_t msg;
    mavlink_msg_statustext_pack_chan(
        MAVLINK_SYS_ID,
        MAV_COMP_ID_AUTOPILOT1,
        MAVLINK_COMM_0,
        &msg,
        severity,
        str,
        0,
        0
    );

    MAVLINK_SEND(&msg);
    return ret;
}

/**
 * @brief Send parameter through mavlink.
 * 
 * @param key 
 *      Key/Name of parameter.
 * @return 0 if success else -1.
 */
int mavlink_send_parameter(const char *key) {
    
    int ret = -1;
    if (parameter_exist(key)) {
        parameter_lock_mutex();
        
        mavlink_message_t new_msg;
        float value;
        parameter_get_value_no_mutex(key, (void*)&value);

        mavlink_msg_param_value_pack_chan(
            MAVLINK_SYS_ID,
            MAV_COMP_ID_AUTOPILOT1,
            MAVLINK_COMM_0,
            &new_msg,
            key,
            value,
            parameter_get_type_no_mutex(key),
            parameter_get_count_no_mutex(),
            parameter_get_index_no_mutex(key)
        );
        
#ifdef _DEBUG
        if (parameter_get_type_no_mutex(key) == PARAMETER_TYPE_FLOAT) {

            DEBUG("Sending parameter \"%s\" with value: %f.\n", key, value);
        } else {

            DEBUG("Sending parameter \"%s\" with value: %d.\n", key, *(int32_t*)&value);
        }
#endif // _DEBUG

        parameter_unlock_mutex();

        MAVLINK_SEND(&new_msg);
        ret = 0;
    }
    return ret;
}

pthread_mutex_t mavlink_send_parameter_list_mutex = PTHREAD_MUTEX_INITIALIZER;
void *mavlink_send_parameter_list_handler(void *arg);

/**
 * @brief Create a thread in order to send parameter list while keep transmition thread active.
 * 
 * @return 0 if success else errno.
 */
int mavlink_send_parameter_list() {
    pthread_t th;
    return pthread_create(&th, NULL, mavlink_send_parameter_list_handler, NULL);
}

void *mavlink_send_parameter_list_handler(void *arg) {
    pthread_detach(pthread_self());
    if (pthread_mutex_trylock(&mavlink_send_parameter_list_mutex) != 0) {
        LOG_ERROR("There is a trasmition currently running.\n");
        pthread_exit(NULL);
    }

    int i;
    int cnt = parameter_get_count_no_mutex();
    for (i = 0; i < cnt; i++) {
        mavlink_send_parameter(parameter_keys[i]);
        usleep(100000);
        if (mavlink_get_active_connections() == 0) {
            LOG_ERROR("No connection is alive. Stop sending parameter list.\n");
            break;
        }
    }
    pthread_mutex_unlock(&mavlink_send_parameter_list_mutex);
    pthread_exit(NULL);
}