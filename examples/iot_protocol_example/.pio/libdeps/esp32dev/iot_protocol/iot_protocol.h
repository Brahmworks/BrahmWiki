#ifndef IOT_PROTOCOL_H
#define IOT_PROTOCOL_H

#include <stdint.h>
#include "iot_protocol_defs.h"

// --- Configuration Structures ---

typedef struct {
    double warn_threshold;
    double fail_threshold;
} sensor_thresholds_t;

typedef struct {
    const char* id;
    sensor_thresholds_t thresholds;
} sensor_config_t;

typedef struct {
    const char* id;
} actuator_config_t;

typedef struct {
    const char* device_id;
    const char* device_type;
    const char* firmware_version;
    sensor_config_t* sensors;
    int num_sensors;
    actuator_config_t* actuators;
    int num_actuators;
} device_config_t;

// --- Runtime Data Structures ---

typedef struct {
    const char* id;
    double value;
} sensor_data_t;

typedef struct {
    const char* id;
    const char* value;
} actuator_data_t;

typedef struct {
    sensor_data_t* sensors;
    int num_sensors;
    actuator_data_t* actuators;
    int num_actuators;
} device_data_t;

// --- Function Prototypes ---

/**
 * @brief Generates a handshake request JSON string.
 * @param config Pointer to the device configuration.
 * @return char* Dynamically allocated JSON string. Caller must free.
 */
char* iot_protocol_get_handshake_request(const device_config_t* config);

/**
 * @brief Generates a health check response based on current data and configuration.
 * @param config Pointer to the device configuration.
 * @param data Pointer to the current device data.
 * @return char* Dynamically allocated JSON string. Caller must free.
 */
char* iot_protocol_get_health_check_response(const device_config_t* config, const device_data_t* data);

/**
 * @brief Processes an incoming JSON string and generates a response.
 * @param incoming_string The raw JSON string from the communication channel.
 * @param config Pointer to the device configuration.
 * @param data Pointer to the current device data.
 * @return char* Dynamically allocated JSON response string, or NULL if no response is needed. Caller must free.
 */
char* iot_protocol_process_incoming(const char* incoming_string, const device_config_t* config, const device_data_t* data);

#endif // IOT_PROTOCOL_H
