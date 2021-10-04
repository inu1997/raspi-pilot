#include "mavlink_main.h"
#include "mavlink_handler.h"
#include "mavlink_tcp_server.h"
#include "mavlink_udp.h"
#include "mavlink_stream.h"
#include "mavlink_util.h"
#include "mavlink_serial.h"

#include "subscription/subscription.h"
#include "c_library_v2/standard/mavlink.h"

#include "measurement/measurement.h"

#include "util/tv.h"
#include "util/logger.h"
#include "util/debug.h"
#include "util/macro.h"

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <termio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

pthread_mutex_t _n_connect_mutex;

static int _n_connection;

void mavlink_on_connection_active() {
    pthread_mutex_lock(&_n_connect_mutex);
    _n_connection++;
    DEBUG("Connection active. now: %d.\n", _n_connection);
    pthread_mutex_unlock(&_n_connect_mutex);
}

void mavlink_on_connection_inactive() {
    pthread_mutex_lock(&_n_connect_mutex);
    _n_connection--;
    DEBUG("Connection inactive. now: %d.\n", _n_connection);
    pthread_mutex_unlock(&_n_connect_mutex);
}

//-----

int mavlink_init() {
    LOG("Initiating MAVLink.\n");
    pthread_mutex_init(&_n_connect_mutex, NULL);
    _n_connection = 0;

    if (subscription_init() != 0) {
        LOG_ERROR("Failed to initiate Subscription.\n");
        return -1;
    }

    if (mavlink_init_tcp_server() != 0) {
        LOG_ERROR("Failed to initiate server.\n");
        return -1;
    }
    
    // if (mavlink_init_udp() != 0) {
    //     LOG_ERROR("Failed to initiate UDP.\n");
    //     return -1;
    // }

    // if (mavlink_init_serial("/dev/ttyS0") != 0) {
    //     LOG_ERROR("Failed to start serial.\n");
    //     return -1;
    // }
    
    if (mavlink_init_stream() != 0) {
        LOG_ERROR("Failed to initiate stream.\n");
        return -1;
    }

    LOG("Done.\n");
    return 0;
}

int mavlink_send(mavlink_message_t *msg) {
    char buf[256]; \
    int len = mavlink_msg_to_send_buffer(buf, msg); \
    mavlink_publish(buf, len); \
}

/**
 * @brief Publish a message to mavlink_publisher to all subscriber in threads.
 * 
 * @param buf
 *      Message buffer
 * @param len 
 *      Length of buffer
 * @return Number of subscriber receive this message.
 */
int mavlink_publish(uint8_t *buf, int len) {
    return publish(buf, len);
}

/**
 * @brief Mavlink thread function.
 * 
 * @param arg
 *      ConnectionArg struct.
 * @return NULL
 */
void *mavlink_connection_handler(void *mavlink_file) {
    pthread_detach(pthread_self());
    
    struct MAVLinkFile *connection = (struct MAVLinkFile*)mavlink_file;
    
    LOG("Connection to \"%s\" established. Channel: %d.\n", connection->name, connection->mav_channel);
    LOG("Initializing.\n");
    if (connection->on_begin != NULL) {
        DEBUG("Executing on_begin function.\n");
        connection->on_begin();
    }
    DEBUG("Setting detachable.\n");

    LOG("Activate subscribtor.\n");
    subscriber_set_active(connection->mav_channel, true);

    int ret; // Return value of R/W a fd.
    int i; // For recurse.
    int w_cnt; // Write count.
    char w_buf[256]; // Buffer for write.
    int r_cnt; // Read count.
    char r_buf[256]; // buffer for read.

    mavlink_message_t r_msg; // mavlink message received.
    mavlink_status_t mav_status; // Mavlink message status.
    
    bool active = false; // Check if connection active/ inactive.
    struct timeval tv_since_last_hb = TV_INITIALIZER; // Check idle time.

    fcntl(connection->fd, F_SETFL, fcntl(connection->fd, F_GETFL, 0) | O_NONBLOCK);
    LOG("Connection \"%s\" is now trasmitting.\n", connection->name);
    while (1) {
        // Limit rate.
        usleep(10000);
        
        // Send
        while (subscriber_available(connection->mav_channel)) {
            if ((w_cnt = subscriber_read(connection->mav_channel, w_buf, sizeof(w_buf))) > 0) {
                // Send
                if ((ret = write(connection->fd, w_buf, w_cnt)) < 0) {
                    LOG_ERROR("Error while write.(%s)\n", strerror(ret));
                    if (connection->exit_on_error) {
                        goto CONNECTION_DIED;
                    }
                    
                } 
                tcdrain(connection->fd);
            }
        }
        // Read
        r_cnt = read(connection->fd, r_buf, sizeof(r_buf));
        
        if (r_cnt > 0) {

            for(i = 0; i < r_cnt; i++) {

                if (mavlink_parse_char(connection->mav_channel, r_buf[i], &r_msg, &mav_status)) {
                    // GOT MESSAGE!
#ifdef _DEBUG
                    if (r_msg.msgid != 0 && r_msg.msgid != MAVLINK_MSG_ID_MANUAL_CONTROL) {
                        DEBUG("Got message. sys_id: %d, comp_id: %d, seq: %d, msg_id: %d.\n", r_msg.sysid, r_msg.compid, r_msg.seq, r_msg.msgid);
                    }
#endif // _DEBUG
                    // Handle message.
                    if ((mavlink_handler_handle_msg(&r_msg)) == 0) {
#ifdef _DEBUG
                        if (r_msg.msgid != MAVLINK_MSG_ID_MANUAL_CONTROL && r_msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
                            DEBUG("Handle success.\n");
#endif // _DEBUG

                        }
                    } else {
                        LOG_ERROR("Handler failure.\n");
                    }

                    // Check heartbeat.
                    if (r_msg.msgid == 0) {
                        // Heartbeat received.
                        if (active == false) {
                            mavlink_on_connection_active();
                            subscriber_set_active(connection->mav_channel, true);
                            active = true;
                        }
                        // Update timeval.
                        gettimeofday(&tv_since_last_hb, NULL);
                    }
                }
            }
        }
        // Check inactive.
        struct timeval now;
        gettimeofday(&now, NULL);
        if (tv_get_diff_sec_ul(&tv_since_last_hb, &now) >= 5) {
            if (active == true) {
                mavlink_on_connection_inactive();
                subscriber_set_active(connection->mav_channel, false);
                active = false;
            }
        }
    }
    // Connection died.
    CONNECTION_DIED:
    if (active == true) {
        mavlink_on_connection_inactive();
        active = false;
    }

    subscriber_reset(connection->mav_channel);

    mavlink_release_channel(connection->mav_channel);
    
    if (connection->on_end != NULL) {
        DEBUG("Executing end function.\n");
        connection->on_end();
    }
    
    close(connection->fd);
    LOG("Connection to \"%s\" died.\n", connection->name);
    free(connection);
    pthread_exit(NULL);
}

/**
 * @brief Get the number of active mavlink connections.
 * 
 * @return Number of active connections.
 */
int mavlink_get_active_connections() {
    pthread_mutex_lock(&_n_connect_mutex);
    int ret = _n_connection;
    pthread_mutex_unlock(&_n_connect_mutex);
    return ret;
}

//----- Utilities.

volatile bool channel_is_occupied[MAVLINK_COMM_NUM_BUFFERS] = {false, false, false, false};
pthread_mutex_t channel_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief Get an unoccupied channel for thread and occupy it.
 * 
 * @return -1 if no channel is available else the index of channel. 
 */
int mavlink_ocupy_usable_channel() {
    pthread_mutex_lock(&channel_mutex);
    
    int ch = -1;
    int i;
    for (i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        if (!channel_is_occupied[i]) {
            channel_is_occupied[i] = true;
            ch = i;
            break;
        }
    }
    
    pthread_mutex_unlock(&channel_mutex);

    return ch;
}

/**
 * @brief Release the channel for other threads to use.
 * 
 * @param chan
 *      Index of channel.
 */
void mavlink_release_channel(int chan) {
    pthread_mutex_lock(&channel_mutex);
    
    if (chan < MAVLINK_COMM_NUM_BUFFERS) {
        channel_is_occupied[chan] = false;
    }
    mavlink_reset_channel_status(chan);
    pthread_mutex_unlock(&channel_mutex);
}

/**
 * @brief Check if channel is occupied by index.
 * 
 * @param chan
 *      Index of channel.
 * @return true if occupied else false.
 */
bool mavlink_is_channel_occupied(int chan){
    return chan >= MAVLINK_COMM_NUM_BUFFERS? true : channel_is_occupied[chan];
}

//-----
