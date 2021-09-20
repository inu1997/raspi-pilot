#include "util/loop.h"
#include "measurement/measurement.h"
#include "pilot/pilot.h"
#include "mavlink/mavlink_main.h"

#include "util/parameter.h"
#include "util/logger.h"
#include "util/system/scheduler.h"
#include "util/system/signal_hanlder.h"

#include <stdio.h>
#include <stdlib.h>


#include <stdio.h>
#include <stdlib.h>

int init();

int main() {
    if (init() != 0) {
        LOG_ERROR("Failed to initiate.\n");
        return -1;
    }

    LOG("Entering control loop.\n");
    while (1) {
        loop_delay_control();

        parameter_lock_mutex();
        
        // measurement_update();
        
        // pilot_update();

        parameter_unlock_mutex();
        // LOG("Loop alive.\n");
    }
    return 0;
}

int init(){
    LOG("Initiating everything.\n");
    // Register exit functions.]
    if (signal_handler_init() != 0){
        LOG_ERROR("Failed to register signal handler.\n");
        return -1;
    }
    if (parameter_init() != 0) {
        return -1;
    }
    
    LOG("Initiating modules.\n");
    // Initiate modules
    // if (measurement_init() != 0){
    //     LOG_ERROR("Failed to initiate Measurement.\n");
    //     return -1;
    // }

    if (pilot_init() != 0){
        LOG_ERROR("Failed to initiate Pilot.\n");
        return -1;
    }

    // Initiate Loop
    if (mavlink_init() != 0) {
        LOG_ERROR("Failed to initiate Remote.\n");
        return -1;
    }

    if (scheduler_init_real_time() != 0) {
        LOG_ERROR("Failed to initiate Real Time.\n");
        return -1;
    }
    loop_init(100);

    LOG("Done.\n");
    return 0;
}