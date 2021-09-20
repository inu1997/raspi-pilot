#include "parameter.h"

#include "pilot/pilot.h"

#include "util/logger.h"
#include "util/debug.h"

#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <json-c/json.h>

#define PARAM_FILE "parameter.json"

const char *parameter_keys[] = {
    "MTR_PWM_FREQ",
    "MTR_PWM_MAX",
    "MTR_PWM_MIN",
    "MANUAL_AX_MAX",
    "MANUAL_AY_MAX",
    "MANUAL_AVZ_MAX",
    "CALIB_OFFSET_GX",
    "CALIB_OFFSET_GY",
    "CALIB_OFFSET_GZ",
    "CALIB_OFFSET_MX",
    "CALIB_OFFSET_MY",
    "CALIB_OFFSET_MZ",
    "CALIB_SCALE_MX",
    "CALIB_SCALE_MY",
    "CALIB_SCALE_MZ",
    "PID_AX_P",
    "PID_AX_I",
    "PID_AX_D",
    "PID_AX_I_LIMIT",
    "PID_AX_O_LIMIT",
    "PID_AY_P",
    "PID_AY_I",
    "PID_AY_D",
    "PID_AY_I_LIMIT",
    "PID_AY_O_LIMIT",
    "PID_AZ_P",
    "PID_AZ_I",
    "PID_AZ_D",
    "PID_AZ_I_LIMIT",
    "PID_AZ_O_LIMIT",
    "PID_AVX_P",
    "PID_AVX_I",
    "PID_AVX_D",
    "PID_AVX_I_LIMIT",
    "PID_AVX_O_LIMIT",
    "PID_AVY_P",
    "PID_AVY_I",
    "PID_AVY_D",
    "PID_AVY_I_LIMIT",
    "PID_AVY_O_LIMIT",
    "PID_AVZ_P",
    "PID_AVZ_I",
    "PID_AVZ_D",
    "PID_AVZ_I_LIMIT",
    "PID_AVZ_O_LIMIT",
    "PID_VA_P",
    "PID_VA_I",
    "PID_VA_D",
    "PID_VA_I_LIMIT",
    "PID_VA_O_LIMIT",
    "PID_ALT_P",
    "PID_ALT_I",
    "PID_ALT_D",
    "PID_ALT_I_LIMIT",
    "PID_ALT_O_LIMIT"
};

pthread_mutex_t _parameter_mutex;

json_object *_param_json;

//-----

/**
 * @brief Load parameters from json file and register them to param_list.
 * 
 * @return 0 if success else -1.
 */
int parameter_init() {
    pthread_mutex_init(&_parameter_mutex, NULL);

    LOG("Loading parameters.\n");
    //-----
    _param_json = json_object_from_file(PARAM_FILE);

    LOG("Load complete.\n");
    DEBUG("Registed %d parameters.\n", parameter_get_count_no_mutex());
    return 0;
}

bool parameter_exist(const char *key) {
    json_object *param;
    return json_object_object_get_ex(_param_json, key, &param);
}

/**
 * @brief Get the number of parameters registered with mutex protection.
 * 
 * @return The number of parameters.
 */
int parameter_get_count() {
    parameter_lock_mutex();
    
    int ret = parameter_get_count_no_mutex();
    
    parameter_unlock_mutex();
    
    return ret;
}

/**
 * @brief Get the number of parameters registered without mutex protection.
 * 
 * @return The number of parameters.
 */
int parameter_get_count_no_mutex() {
    return json_object_object_length(_param_json);
}

/**
 * @brief Get the value of parameters registered with mutex protection.
 * 
 * @param key
 *      Key/name of parameter.
 * @param data
 *      Data cather.
 * @return 0 if found & valid, -1 if not found, -2 if not valid.
 */
int parameter_get_value(const char *key, void *data) {
    parameter_lock_mutex();
    
    int ret = parameter_get_value_no_mutex(key, data);
    
    parameter_unlock_mutex();
    
    return ret;
}

/**
 * @brief Get the value of parameters registered without mutex protection.
 * 
 * @param key
 *      Key/name of parameter.
 * @param data
 *      Data cather.
 * @return 0 if found & valid, -1 if not found, -2 if not valid, -3 if pilot is armed.
 */
int parameter_get_value_no_mutex(const char *key, void *data) {
    int ret = -1;
    json_object *param;
    
    if (json_object_object_get_ex(_param_json, key, &param)) {
        // Found parameter
        ret = 0;
        if (json_object_is_type(param, json_type_double)) {
            *(float*)data = (float)json_object_get_double(param);
        } else if (json_object_is_type(param, json_type_int)) {
            *(int32_t*)data = json_object_get_int(param);
        } else {
            ret = -2;
        }
    }

    return ret;
}

/**
 * @brief Set the value of parameters registered with mutex protection.
 * 
 * @param key
 *      Key/name of parameter.
 * @param data
 *      Data.
 * @param callback
 *      Do callback function if true.
 * @return 0 if found & valid, -1 if not found, -2 if not valid.
 */
int parameter_set_value(const char *key, void *data, bool save) {
    parameter_lock_mutex();
    
    int ret = parameter_set_value_no_mutex(key, data, save);
    
    parameter_unlock_mutex();
    
    return ret;
}

/**
 * @brief Set the value of parameters registered with mutex protection.
 * 
 * @param key
 *      Key/name of parameter.
 * @param data
 *      Data.
 * @param callback
 *      Do callback function if true.
 * @return 0 if found & valid, -1 if not found, -2 if not valid, -3 if pilot is armed.
 */
int parameter_set_value_no_mutex(const char *key, void *data, bool save) {
    if (pilot_is_armed()) {
        return -3;
    }
    int ret = -1;
    json_object *param;
    if (json_object_object_get_ex(_param_json, key, &param)) {
        ret = 0;
        if (json_object_is_type(param, json_type_double)) {
            json_object_set_double(param, *(float*)data);
        } else if (json_object_is_type(param, json_type_int)) {
            json_object_set_int(param, *(int32_t*)data);
        } else {
            ret = -2;
        }
    }
    if (ret == 0) {
        if (save) {
            // Update json.
            DEBUG("Saving File.\n");
            json_object_to_file_ext(PARAM_FILE, _param_json, JSON_C_TO_STRING_PRETTY);
        }
    } 
    return ret;
}

/**
 * @brief Get the type of parameter with mutex protection.
 * 
 * @param key 
 *      Key/Name of parameter.
 * @return -1 if not found else type of parameter.
 */
int parameter_get_type(const char *key) {
    parameter_lock_mutex();
    
    int ret = parameter_get_type_no_mutex(key);

    parameter_unlock_mutex();

    return ret;
}

/**
 * @brief Get the type of parameter without mutex protection.
 * 
 * @param key 
 *      Key/Name of parameter.
 * @return -1 if not found else type of parameter.
 */
int parameter_get_type_no_mutex(const char *key) {

    int ret = -1;
    json_object *param;
    if (json_object_object_get_ex(_param_json, key, &param)) {
        if (json_object_is_type(param, json_type_double)) {
            ret = PARAMETER_TYPE_FLOAT;
        } else if (json_object_is_type(param, json_type_int)) {
            ret = PARAMETER_TYPE_INT32;
        }
    }

    return ret;
}
 
/**
 * @brief Get the index of key with mutex protection.
 * 
 * @param key 
 *      Key/Name of parameter.
 * @return -1 if not exist in key list else index of the key.
 */
int parameter_get_index(const char *key) {
    parameter_lock_mutex();
    
    int ret = parameter_get_index_no_mutex(key);

    parameter_unlock_mutex();
    
    return ret;
}

/**
 * @brief Get the index of key without mutex protection.
 * 
 * @param key 
 *      Key/Name of parameter.
 * @return -1 if not exist in key list else index of the key.
 */
int parameter_get_index_no_mutex(const char *key) {
    int index = -1;
    int i;
    for (i = 0; i < sizeof(parameter_keys) / sizeof(char *); i++) {
        if (strcmp(key, parameter_keys[i]) == 0) {
            index = i;
            break;
        }
    }
    return index;
}

/**
 * @brief For multi-thread usage. Lock the parameters.
 * 
 */
void parameter_lock_mutex() {
    pthread_mutex_lock(&_parameter_mutex);
}

/**
 * @brief For multi-thread usage. Unlock the parameters.
 * 
 */
void parameter_unlock_mutex() {
    pthread_mutex_unlock(&_parameter_mutex);
}