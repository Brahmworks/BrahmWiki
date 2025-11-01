#ifndef IOT_PROTOCOL_DEFS_H
#define IOT_PROTOCOL_DEFS_H

// --- Enums for Type Safety ---

typedef enum {
    MSG_TYPE_HANDSHAKE,
    MSG_TYPE_HEALTH_CHECK,
    MSG_TYPE_COMMAND,
    MSG_TYPE_ERROR
} message_type_t;

typedef enum {
    CMD_TYPE_CONTROL,
    CMD_TYPE_CONFIG,
    CMD_TYPE_FW_UPDATE,
    CMD_TYPE_DATA_REQ
} command_type_t;

typedef enum {
    COMP_STATUS_PASS,
    COMP_STATUS_WARN,
    COMP_STATUS_FAIL
} component_status_t;

// --- String Representations for Enums ---
// Allows using enums internally and strings in JSON

extern const char* MSG_TYPE_STR[];
extern const char* CMD_TYPE_STR[];
extern const char* COMP_STATUS_STR[];

// --- JSON Error Codes ---
#define ERROR_CODE_INVALID_JSON "INVALID_JSON"
#define ERROR_CODE_MISSING_FIELD "MISSING_REQUIRED_FIELD"
#define ERROR_CODE_INVALID_FIELD_TYPE "INVALID_FIELD_TYPE"

// --- Status Codes ---

// Success Codes (2xx)
#define STATUS_OK 200
#define STATUS_CREATED 201
#define STATUS_ACCEPTED 202
#define STATUS_NO_CONTENT 204

// Client Error Codes (4xx)
#define STATUS_BAD_REQUEST 400
#define STATUS_UNAUTHORIZED 401
#define STATUS_FORBIDDEN 403
#define STATUS_NOT_FOUND 404
#define STATUS_REQUEST_TIMEOUT 408
#define STATUS_TOO_MANY_REQUESTS 429

// Server Error Codes (5xx)
#define STATUS_INTERNAL_SERVER_ERROR 500
#define STATUS_BAD_GATEWAY 502
#define STATUS_SERVICE_UNAVAILABLE 503
#define STATUS_GATEWAY_TIMEOUT 504

// Device-Specific Status Codes (6xx)
#define STATUS_DEVICE_OFFLINE 600
#define STATUS_LOW_BATTERY 601
#define STATUS_SENSOR_ERROR 602
#define STATUS_MEMORY_FULL 603
#define STATUS_INVALID_STATE 604

#endif // IOT_PROTOCOL_DEFS_H
