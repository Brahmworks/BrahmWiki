#include "iot_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "cJSON.h"
#include <stdio.h>

// --- String Representations for Enums ---
const char* MSG_TYPE_STR[] = { "handshake", "health_check", "command", "error" };
const char* CMD_TYPE_STR[] = { "control", "configuration", "firmware_update", "data_request" };
const char* COMP_STATUS_STR[] = { "pass", "warn", "fail" };

// --- Private Function Prototypes ---
static char* handle_health_check(const device_config_t* config, const device_data_t* data);
static char* handle_command(cJSON *root);
static char* create_error_response(int statusCode, const char* errorCode, const char* message);

char* iot_protocol_get_handshake_request(const device_config_t* config) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "messageType", MSG_TYPE_STR[MSG_TYPE_HANDSHAKE]);
    cJSON_AddStringToObject(root, "version", "1.0");
    
    cJSON *request = cJSON_CreateObject();
    cJSON_AddStringToObject(request, "deviceId", config->device_id);
    cJSON_AddStringToObject(request, "deviceType", config->device_type);
    cJSON_AddStringToObject(request, "firmwareVersion", config->firmware_version);
    cJSON_AddItemToObject(root, "request", request);

    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_string;
}

char* iot_protocol_process_incoming(const char* incoming_string, const device_config_t* config, const device_data_t* data) {
    cJSON *root = cJSON_Parse(incoming_string);
    if (root == NULL) {
        return create_error_response(STATUS_BAD_REQUEST, ERROR_CODE_INVALID_JSON, "Invalid JSON format.");
    }

    char* response_string = NULL;
    cJSON *messageTypeItem = cJSON_GetObjectItem(root, "messageType");
    if (!cJSON_IsString(messageTypeItem)) {
        cJSON_Delete(root);
        return create_error_response(STATUS_BAD_REQUEST, ERROR_CODE_MISSING_FIELD, "Missing or invalid 'messageType' field.");
    }

    const char* messageType = messageTypeItem->valuestring;
    if (strcmp(messageType, MSG_TYPE_STR[MSG_TYPE_HEALTH_CHECK]) == 0) {
        response_string = handle_health_check(config, data);
    } else if (strcmp(messageType, MSG_TYPE_STR[MSG_TYPE_COMMAND]) == 0) {
        response_string = handle_command(root);
    }

    cJSON_Delete(root);
    return response_string;
}

char* iot_protocol_get_health_check_response(const device_config_t* config, const device_data_t* data) {
    return handle_health_check(config, data);
}

static char* handle_health_check(const device_config_t* config, const device_data_t* data) {
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "messageType", MSG_TYPE_STR[MSG_TYPE_HEALTH_CHECK]);
    cJSON_AddStringToObject(response, "version", "1.0");
    cJSON_AddStringToObject(response, "deviceId", config->device_id);

    cJSON *checks = cJSON_CreateObject();
    component_status_t overall_status = COMP_STATUS_PASS;

    // Process sensors
    for (int i = 0; i < data->num_sensors; i++) {
        component_status_t status = COMP_STATUS_PASS;
        for (int j = 0; j < config->num_sensors; j++) {
            if (strcmp(data->sensors[i].id, config->sensors[j].id) == 0) {
                if (data->sensors[i].value >= config->sensors[j].thresholds.fail_threshold) {
                    status = COMP_STATUS_FAIL;
                } else if (data->sensors[i].value >= config->sensors[j].thresholds.warn_threshold) {
                    status = COMP_STATUS_WARN;
                }
                break;
            }
        }

        if (status == COMP_STATUS_FAIL) overall_status = COMP_STATUS_FAIL;
        else if (status == COMP_STATUS_WARN && overall_status != COMP_STATUS_FAIL) overall_status = COMP_STATUS_WARN;

        cJSON *sensor_array = cJSON_CreateArray();
        cJSON *sensor_item = cJSON_CreateObject();
        cJSON_AddStringToObject(sensor_item, "componentId", data->sensors[i].id);
        cJSON_AddStringToObject(sensor_item, "componentType", "sensor");
        cJSON_AddStringToObject(sensor_item, "status", COMP_STATUS_STR[status]);
        cJSON_AddNumberToObject(sensor_item, "observedValue", data->sensors[i].value);
        cJSON_AddItemToArray(sensor_array, sensor_item);
        
        char key[64];
        sprintf(key, "sensor:%s", data->sensors[i].id);
        cJSON_AddItemToObject(checks, key, sensor_array);
    }

    // Process actuators
    for (int i = 0; i < data->num_actuators; i++) {
        cJSON *actuator_array = cJSON_CreateArray();
        cJSON *actuator_item = cJSON_CreateObject();
        cJSON_AddStringToObject(actuator_item, "componentId", data->actuators[i].id);
        cJSON_AddStringToObject(actuator_item, "componentType", "actuator");
        cJSON_AddStringToObject(actuator_item, "status", COMP_STATUS_STR[COMP_STATUS_PASS]);
        cJSON_AddStringToObject(actuator_item, "observedValue", data->actuators[i].value);
        cJSON_AddItemToArray(actuator_array, actuator_item);

        char key[64];
        sprintf(key, "actuator:%s", data->actuators[i].id);
        cJSON_AddItemToObject(checks, key, actuator_array);
    }

    cJSON_AddStringToObject(response, "status", COMP_STATUS_STR[overall_status]);
    cJSON_AddNumberToObject(response, "statusCode", overall_status == COMP_STATUS_FAIL ? STATUS_INTERNAL_SERVER_ERROR : STATUS_OK);
    cJSON_AddItemToObject(response, "checks", checks);

    char *json_string = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    return json_string;
}

static char* handle_command(cJSON *root) {
    cJSON *commandTypeItem = cJSON_GetObjectItem(root, "commandType");
    if (!cJSON_IsString(commandTypeItem)) {
        return create_error_response(STATUS_BAD_REQUEST, ERROR_CODE_MISSING_FIELD, "Missing or invalid 'commandType' field.");
    }
    // Further command handling logic would go here
    return NULL; 
}

static char* create_error_response(int statusCode, const char* errorCode, const char* message) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "messageType", MSG_TYPE_STR[MSG_TYPE_ERROR]);
    cJSON_AddNumberToObject(root, "statusCode", statusCode);

    cJSON *error = cJSON_CreateObject();
    cJSON_AddStringToObject(error, "code", errorCode);
    cJSON_AddStringToObject(error, "message", message);
    cJSON_AddItemToObject(root, "error", error);

    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_string;
}
