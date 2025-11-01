#ifndef IOT_PROTOCOL_DEFS_H
#define IOT_PROTOCOL_DEFS_H

// Message Types
#define MSG_TYPE_HANDSHAKE "handshake"
#define MSG_TYPE_HEALTH_CHECK "health_check"
#define MSG_TYPE_COMMAND "command"
#define MSG_TYPE_ERROR "error"

// Command Types
#define CMD_TYPE_CONTROL "control"
#define CMD_TYPE_CONFIG "configuration"
#define CMD_TYPE_FW_UPDATE "firmware_update"
#define CMD_TYPE_DATA_REQ "data_request"

// Device Types
#define DEV_TYPE_SENSOR "sensor"
#define DEV_TYPE_ACTUATOR "actuator"

// Sensor and Actuator IDs
#define SENSOR_1 "sensor_1"
#define SENSOR_2 "sensor_2"
#define ACTUATOR_1 "actuator_1"
#define ACTUATOR_2 "actuator_2"

#endif // IOT_PROTOCOL_DEFS_H
