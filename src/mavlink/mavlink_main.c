#include "mavlink_main.h"
#include "mavlink_handler.h"
#include "mavlink_tcp.h"
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
#include "util/system/scheduler.h"

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <termio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

pthread_mutex_t _n_connect_mutex; // Be used to count connections.
pthread_mutex_t _communication_init_mutex; // Be used to initiate communication.

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
    pthread_mutex_init(&_communication_init_mutex, NULL);
    _n_connection = 0;

    if (subscription_init() != 0) {
        LOG_ERROR("Failed to initiate Subscription.\n");
        return -1;
    }

    if (mavlink_init_tcp() != 0) {
        LOG_ERROR("Failed to initiate tcp.\n");
        return -1;
    }
    
    // if (mavlink_init_udp() != 0) {
    //     LOG_ERROR("Failed to initiate UDP.\n");
    //     return -1;
    // }

    if (mavlink_init_serial("/dev/ttyS0") != 0) {
        LOG_ERROR("Failed to start serial.\n");
        return -1;
    }
    
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

void mavlink_communication(const int fd, const bool exit_on_error, const bool exit_on_idle, const bool wait_for_heartbeat) {
    int channel; // MAVLink channel.

    int ret; // Return value of R/W a fd.
    int i; // For recurse.
    int w_cnt; // Write count.
    char w_buf[256]; // Buffer for write.
    int r_cnt; // Read count.
    char r_buf[256]; // buffer for read.
    bool active = false; // Check if connection is active or idle.
    struct timeval tv_since_last_hb; // Time since last time received heartbeat.
    struct timeval now; // For getting current time.
    mavlink_message_t r_msg; // Received MAVLink message.
    mavlink_status_t r_status; // Received MAVLink status.
    
    // Initialization/
    channel = mavlink_ocupy_usable_channel();
    mavlink_reset_channel_status(channel);
    
    if (wait_for_heartbeat == false) {
        // Don't wait for heartbeat. Start the subscriber anyway.
        subscriber_set_active(channel, true);
    }
    // Make the RW non-blocking.
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    gettimeofday(&tv_since_last_hb, NULL);

    // Communication loop.
    while (1) {
        usleep(100000);

        // Write, check if there is message in subscriber queue.
        while (subscriber_available(channel)) {
            // Read from subscriber.
            if ((w_cnt = subscriber_read(channel, w_buf, sizeof(w_buf))) > 0) {
                // Write to fd.
                if ((ret = write(fd, w_buf, w_cnt)) < 0) {
                    if (exit_on_error) {
                        LOG_ERROR("Error occured while writing.\n");
                        goto EXIT;
                    }
                }
                tcdrain(fd);
            }
        }

        // Read
        if ((r_cnt = read(fd, r_buf, sizeof(r_buf))) > 0) {
            for (i = 0; i < r_cnt; i++) {
                // Parse read characters.
                if (mavlink_parse_char(channel, r_buf[i], &r_msg, &r_status)) {
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
                        }
#endif // _DEBUG
                    } else {
                        LOG_ERROR("Handler failure.\n");
                    }

                    // Check heartbeat if wait_for_heartbeat is set.
                    if (r_msg.msgid == 0) {
                        // Heartbeat received.
                        // Update timeval.
                        gettimeofday(&tv_since_last_hb, NULL);

                        if (active == false) {
                            // This communication is active.
                            active = true;
                            mavlink_on_connection_active();
                            scheduler_set_real_time(true, 5);

                            if (wait_for_heartbeat) {
                                subscriber_set_active(channel, true);
                            }
                        }
                    }
                }
            }
        }

        
        // Check inactive if exit_on_idle is set..
        gettimeofday(&now, NULL);
        if (tv_get_diff_sec_ul(&tv_since_last_hb, &now) >= 5) {
            
            if (exit_on_idle) {
                LOG_ERROR("Communication idle. Exiting.\n");
                goto EXIT;
            }

            if (active == true) {
                // This communication is longer active.
                active = false;
                mavlink_on_connection_inactive();

                scheduler_set_real_time(false, 0);
                if (wait_for_heartbeat) {
                    subscriber_set_active(channel, false);
                }
            }
        }
    }

    EXIT:
    // Turning off.
    if (active == true) {
        active = false;
        mavlink_on_connection_inactive();
        scheduler_set_real_time(false, 0);
    }
    subscriber_set_active(channel, false);
    mavlink_release_channel(channel);
    close(fd);
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
