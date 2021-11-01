#include "mavlink_tcp_server.h"
#include "mavlink_main.h"
#include "util/logger.h"
#include "util/debug.h"
#include "util/system/scheduler.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define PORT 1128

static int _server_fd; // File descriptor of server socket.

static int _n_connection; // Number of connections.

static pthread_t _server_thread; // Thread handler of server.

void *_server_handler(void *arg);

static void on_connect();

static void on_disconnect();

static void server_close();

//-----

/*
 * @brief Initiate server.
 * @return 0 if success else -1.
 */
int mavlink_init_tcp_server() {
    LOG("Initiating server.\n");

    _n_connection = 0;
    _server_fd = socket(AF_INET, SOCK_STREAM, 0);
    atexit(server_close);
    int option = 1;
    setsockopt(_server_fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    if (_server_fd == -1) {
        LOG_ERROR("Failed to open server fd.\n");
        return -1;
    }

    struct sockaddr_in server_info;
    server_info.sin_family = AF_INET;
    server_info.sin_addr.s_addr = INADDR_ANY;
    server_info.sin_port = htons(PORT);
    
    LOG("Binding port(%d).\n", PORT);
    if (bind(_server_fd, (struct sockaddr*)&server_info, sizeof(server_info)) < 0) {
        LOG_ERROR("Failed to bind.\n");
        return -1;
    }

    LOG("Listening.\n");
    if (listen(_server_fd, 5) < 0) {
        LOG_ERROR("Failed to listen.\n");
        return -1;
    }
    
    LOG("Creating thread.\n");
    if (pthread_create(&_server_thread, NULL, _server_handler, NULL) != 0) {
        LOG_ERROR("Failed to create server thread.\n");
        return -1;
    }
    
    LOG("Done.\n");
    return 0;
}

void server_close() {
    close(_server_fd);
}
//-----

void *_server_handler(void *arg) {
    pthread_detach(pthread_self());
    
    struct sockaddr_in client_info;
    int addr_len = sizeof(client_info);
    while (1){
        int connection_fd = accept(_server_fd, (struct sockaddr*)&client_info, &addr_len);
        // Make a new mavlink connection struct.
        struct MAVLinkFile *connection = malloc(sizeof(struct MAVLinkFile));
        
        inet_ntop(AF_INET, &client_info.sin_addr, connection->name, INET_ADDRSTRLEN);
        connection->fd = connection_fd;
        connection->mav_channel = mavlink_ocupy_usable_channel();
        connection->on_begin = on_connect;
        connection->on_end = on_disconnect;
        connection->exit_on_error = true;
        pthread_t thread;
        scheduler_create_rt_thread(&thread, 5, mavlink_connection_handler, (void*)connection);
        usleep(10000);
    }    

    pthread_exit(NULL);
}

void on_connect() {
    _n_connection++;
}
void on_disconnect() {
    _n_connection--;
}