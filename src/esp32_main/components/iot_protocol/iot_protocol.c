#include "iot_protocol.h"
#include "iot_protocol_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "cJSON.h"

static uint8_t current_uart_port;

void iot_protocol_init(uint8_t uart_port) {
    current_uart_port = uart_port;
}

void iot_protocol_handshake(void) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "messageType", MSG_TYPE_HANDSHAKE);
    cJSON_AddStringToObject(root, "version", "1.0");
    
    cJSON *request = cJSON_CreateObject();
    cJSON_AddStringToObject(request, "deviceId", "device_12345");
    cJSON_AddStringToObject(request, "deviceType", DEV_TYPE_SENSOR);
    cJSON_AddStringToObject(request, "firmwareVersion", "2.1.4");
    cJSON_AddItemToObject(root, "request", request);

    char *json_string = cJSON_PrintUnformatted(root);
    uart_write(current_uart_port, json_string);
    cJSON_free(json_string);
    cJSON_Delete(root);
}

void iot_protocol_send_health_check(void) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "messageType", MSG_TYPE_HEALTH_CHECK);
    cJSON_AddStringToObject(root, "version", "1.0");
    cJSON_AddStringToObject(root, "deviceId", "device_12345");
    cJSON_AddStringToObject(root, "status", "pass");
    cJSON_AddNumberToObject(root, "statusCode", 200);

    char *json_string = cJSON_PrintUnformatted(root);
    uart_write(current_uart_port, json_string);
    cJSON_free(json_string);
    cJSON_Delete(root);
}

void iot_protocol_process_incoming(void) {
    uint8_t buffer[256];
    int len = uart_read(current_uart_port, buffer, sizeof(buffer) - 1);
    if (len > 0) {
        buffer[len] = '\0';
        cJSON *root = cJSON_Parse((const char *)buffer);
        if (root != NULL) {
            cJSON *messageType = cJSON_GetObjectItem(root, "messageType");
            if (cJSON_IsString(messageType) && (strcmp(messageType->valuestring, MSG_TYPE_COMMAND) == 0)) {
                uart_write(current_uart_port, "Received command\n");
            }
            cJSON_Delete(root);
        }
    }
}
