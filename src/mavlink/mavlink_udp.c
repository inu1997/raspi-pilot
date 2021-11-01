#include "mavlink_udp.h"
#include "mavlink_main.h"
#include "mavlink_handler.h"

#include "util/system/scheduler.h"
#include "util/tv.h"
#include "subscription/subscription.h"

#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <string.h>
#include <pthread.h>

#include "c_library_v2/standard/mavlink.h"

#include "util/logger.h"
#include "util/debug.h"
#include "util/system/scheduler.h"

#define GC_PORT 14550
#define GC_IP "192.168.0.10"

void *mavlink_udp_handler(void *arg);

int mavlink_init_udp() {
    pthread_t th;
    return scheduler_create_rt_thread(&th, 5, mavlink_udp_handler, NULL);
}

void *mavlink_udp_handler(void *arg) {
    pthread_detach(pthread_self());
    
    int sock_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_fd < 0) {
        LOG_ERROR("Failed to open fd.\n");
        return NULL;
    }
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(14551);

    if (bind(sock_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        printf("Failed to bind.\n");
        close(sock_fd);
        return NULL;
    }

	if (fcntl(sock_fd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0) {
        LOG_ERROR("Failed to set non-blocking.\n");
        close(sock_fd);
        return NULL;
    }

    LOG("Initializing.\n");

    int chan = mavlink_ocupy_usable_channel();

    LOG("Activate subscribtor.\n");
    subscriber_set_active(chan, true);

    int ret; // Return value of R/W a fd.
    int i; // For recurse
    int w_cnt; // Length of write buffer.
    char w_buf[256]; // Buffer for write.
    int r_cnt;
    char r_buf[256]; // buffer for read.
    mavlink_message_t r_msg; // mavlink message received.
    mavlink_status_t mav_status; // Mavlink message status.
    
    bool active = false; // Check if connection active/ inactive.
    struct timeval tv_since_last_hb = TV_INITIALIZER; // Check idle time.
    
    LOG("UDP connection is now trasmitting.\n");

    struct sockaddr_in gc_addr;
    memset(&gc_addr, 0, sizeof(gc_addr));

    gc_addr.sin_family = AF_INET;
    gc_addr.sin_addr.s_addr = inet_addr(GC_IP);
    gc_addr.sin_port = htons(GC_PORT);

    socklen_t slen = sizeof(gc_addr);

    while (1) {
        // Limit rate.
        usleep(10000);
        
        // Send
        while (subscriber_available(chan)) {
            if ((w_cnt = subscriber_read(chan, w_buf, sizeof(w_buf))) > 0) {
                // Send
                if ((ret = sendto(sock_fd, w_buf, w_cnt, 0, (struct sockaddr*)&gc_addr, slen)) < 0) {
                    LOG_ERROR("Error while write.(%s)\n", strerror(ret));
                    goto CONNECTION_DIED;
                } 
            }
        }
        // Read
        r_cnt = recvfrom(sock_fd, r_buf, sizeof(r_buf), 0, (struct sockaddr*)&gc_addr, &slen);

        if (r_cnt > 0) {
            for(i = 0; i < r_cnt; i++) {
                if (mavlink_parse_char(chan, r_buf[i], &r_msg, &mav_status)) {
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

    subscriber_reset(chan);

    mavlink_release_channel(chan);
    
    close(sock_fd);
    LOG("UDP connection died.\n");
    
    pthread_exit(NULL);

}