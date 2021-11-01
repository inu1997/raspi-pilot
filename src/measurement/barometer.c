#include "barometer.h"
#include "util/loop.h"

#include "driver/ms5611.h"

#define TIME_DIFF loop_get_interval()

static float _pressure; // Pressure.
static float _pressure_diff; // Pressure differential.
static float _altitude; // Altitude.
static float _climb_rate; // Climb rate from altitude diff.
static float _temp; // Temperature.

/**
 * @brief Initiate Barometer.
 * 
 * @return 0 if success else -1.
 */
int barometer_init() {
    _pressure = 0;
    _altitude = 0;
    _climb_rate = 0;
    _pressure_diff = 0;
    _temp = 0;
    return 0;
}

/**
 * @brief Update barometer reading and compute altitude.
 * 
 */
void barometer_update() {
    float _prev_alt = _altitude;
    float _prev_pres = _pressure;
    // Update sensor reading

    // Filter

    // Convertion

    _climb_rate = (_prev_alt - _altitude) / TIME_DIFF;
    _pressure_diff = (_prev_pres - _pressure) / TIME_DIFF;
}

/**
 * @brief Get altitude according to pressure.
 * 
 * @return Altitude.
 */
float barometer_get_altitude() {
    return _altitude;
}

/**
 * @brief Get climb rate according to the altitude computed from pressure.
 * 
 * @return Climb rate.
 */
float barometer_get_climb_rate() {
    return _climb_rate;
}

/**
 * @brief Get the pressure reading.
 * 
 * @return Pressure.
 */
float barometer_get_pressure() {
    return _pressure;
}

/**
 * @brief Get the pressure differential.
 * 
 * @return Pressure differential.
 */
float barometer_get_pressure_diff() {
    return _pressure_diff;
}

float barometer_get_temperature() {
    return _temp;
}