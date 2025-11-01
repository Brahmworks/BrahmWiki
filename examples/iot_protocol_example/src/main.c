#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_standard.h"
#include "iot_protocol.h"
#include "device_settings.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// --- Device Configuration ---
static const sensor_config_t SENSOR_CONFIGS[] = {
    {SENSOR_ID_HEATER_TEMP, {HEATER_TEMP_WARN_THRESHOLD, HEATER_TEMP_FAIL_THRESHOLD}},
    {SENSOR_ID_WATER_TEMP, {WATER_TEMP_WARN_THRESHOLD, WATER_TEMP_FAIL_THRESHOLD}},
    {SENSOR_ID_ULTRASONIC, {ULTRASONIC_WARN_THRESHOLD, ULTRASONIC_FAIL_THRESHOLD}}
};

static const actuator_config_t ACTUATOR_CONFIGS[] = {
    {ACTUATOR_ID_HEATER},
    {ACTUATOR_ID_SOLENOID},
    {ACTUATOR_ID_PUMP}
};

static const device_config_t DEVICE_CONFIG = {
    .device_id = DEVICE_ID,
    .device_type = DEVICE_TYPE,
    .firmware_version = FIRMWARE_VERSION,
    .sensors = (sensor_config_t*)SENSOR_CONFIGS,
    .num_sensors = sizeof(SENSOR_CONFIGS) / sizeof(sensor_config_t),
    .actuators = (actuator_config_t*)ACTUATOR_CONFIGS,
    .num_actuators = sizeof(ACTUATOR_CONFIGS) / sizeof(actuator_config_t)
};

// --- Simulation Functions ---
double read_sensor(const char* id) {
    if (strcmp(id, SENSOR_ID_HEATER_TEMP) == 0) {
        int r = rand() % 100;
        if (r > 95) return 105.0;
        if (r > 85) return 85.0;
        return 75.0;
    }
    if (strcmp(id, SENSOR_ID_WATER_TEMP) == 0) return 25.0;
    if (strcmp(id, SENSOR_ID_ULTRASONIC) == 0) return 15.2;
    return 0.0;
}

const char* read_actuator(const char* id) {
    if (strcmp(id, ACTUATOR_ID_HEATER) == 0) return "off";
    if (strcmp(id, ACTUATOR_ID_SOLENOID) == 0) return "closed";
    if (strcmp(id, ACTUATOR_ID_PUMP) == 0) return "off";
    return "unknown";
}

void app_main()
{
    uart_init(UART_0_PROG, 115200);

    char* handshake_request = iot_protocol_get_handshake_request(&DEVICE_CONFIG);
    if (handshake_request != NULL) {
        uart_write(UART_0_PROG, "Sending Handshake:\n");
        uart_write(UART_0_PROG, handshake_request);
        uart_write(UART_0_PROG, "\n\n");
        free(handshake_request);
    }

    while(1) {
        uint8_t buffer[512];
        int len = uart_read(UART_0_PROG, buffer, sizeof(buffer) - 1);
        if (len > 0) {
            buffer[len] = '\0';
            uart_write(UART_0_PROG, "Received Data:\n");
            uart_write(UART_0_PROG, (const char*)buffer);
            uart_write(UART_0_PROG, "\n");

            sensor_data_t sensors[DEVICE_CONFIG.num_sensors];
            for (int i = 0; i < DEVICE_CONFIG.num_sensors; i++) {
                sensors[i] = (sensor_data_t){DEVICE_CONFIG.sensors[i].id, read_sensor(DEVICE_CONFIG.sensors[i].id)};
            }

            actuator_data_t actuators[DEVICE_CONFIG.num_actuators];
            for (int i = 0; i < DEVICE_CONFIG.num_actuators; i++) {
                actuators[i] = (actuator_data_t){DEVICE_CONFIG.actuators[i].id, read_actuator(DEVICE_CONFIG.actuators[i].id)};
            }

            device_data_t device_data = {
                .sensors = sensors,
                .num_sensors = DEVICE_CONFIG.num_sensors,
                .actuators = actuators,
                .num_actuators = DEVICE_CONFIG.num_actuators
            };

            char* response = iot_protocol_process_incoming((const char*)buffer, &DEVICE_CONFIG, &device_data);
            if (response != NULL) {
                uart_write(UART_0_PROG, "Sending Response:\n");
                uart_write(UART_0_PROG, response);
                uart_write(UART_0_PROG, "\n\n");
                free(response);
            }
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
