#include "battery.h"

#include "util/loop.h"
#include "util/logger.h"
#include "util/debug.h"

float _voltage;
float _current;
float _comsumed_current;
float _remain_time_sec;

int battery_init() {
    LOG("Initiating Battery measurement.\n");
    
    _voltage = 0;
    _current = 0;
    _comsumed_current = 0;
    _remain_time_sec = 0;

    LOG("Done.\n");
    return 0;
}

void battery_update() {
    
    _comsumed_current += _current * loop_get_interval();
}

float battery_get_voltage() {
    return _voltage;
}

float battery_get_current() {
    return _current;
}

float battery_get_comsumed_current() {
    return _comsumed_current;
}

float battery_get_remain_time_sec() {
    return _remain_time_sec;
}