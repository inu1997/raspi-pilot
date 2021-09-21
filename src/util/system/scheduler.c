#include "scheduler.h"
#include "util/logger.h"
#include "util/debug.h"

#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/time.h>

void scheduler_unlock_memory() {
    LOG("Unlocking all memory.\n");
    munlockall();
}

/**
 * @brief Initate the Real Time process of self.
 * 
 * @return 0 if success else stderr.
 */
int scheduler_init_real_time() {
    LOG("Initiating Linux Real-Time utilities.\n");

    if (getuid() != 0) {
        LOG_ERROR("Not running as root.\n");
        return -1;
    }

    int ret;
    
    if ((ret = atexit(scheduler_unlock_memory)) != 0) {
        LOG_ERROR("Failed to register unlock atexit.\n");
        goto EXIT;
    }
    
    LOG("Locking memory.\n");
    if ((ret = mlockall(MCL_CURRENT)) != 0) {
        LOG_ERROR("Failed to lock memory.\n");
        goto EXIT;
    }
    

    struct sched_param param = {
        .sched_priority = 99
    };

    LOG("Setting top priority.\n");
    if ((ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param)) != 0) {
        LOG_ERROR("Failed to set scheduling parameter.\n");
        goto EXIT;
    }

    LOG("Done.\n");
    
    EXIT:
    return ret;
}

/**
 * @brief Same as pthread_create but with priority argument.
 * 
 * @param thread 
 *      Pthread handle.
 * @param priority
 *      Priority of thread. 
 * @param func 
 *      Function of thread.
 * @param arg 
 *      Argument of pthread_create(..., arg);
 * @return pthread_create return value.
 */
int scheduler_create(pthread_t *thread, int priority, void *(*func)(), void *arg) {
    int ret;
    pthread_attr_t attr;
    if ((ret = pthread_attr_init(&attr)) != 0) {
        LOG_ERROR("Failed to init attribute.\n");
        goto EXIT;
    }

    if ((ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO)) != 0) {
        LOG_ERROR("Failed to set policy.\n");
        goto EXIT;
    }

    struct sched_param param = {
        .sched_priority = priority
    };
    if ((ret = pthread_attr_setschedparam(&attr, &param)) != 0) {
        LOG_ERROR("Failed to set sched param.\n");
        goto EXIT;
    }

    if ((ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) != 0) {
        LOG_ERROR("Failed to set explicit.\n");
        goto EXIT;
    }

    if ((ret = pthread_create(thread, &attr, func, arg)) != 0) {
        LOG_ERROR("Failed to create thread.\n");
        goto EXIT;
    }
    
    EXIT:
    pthread_attr_destroy(&attr);
    return ret;
}