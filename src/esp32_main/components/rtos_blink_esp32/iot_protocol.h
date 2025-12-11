#ifndef IOT_PROTOCOL_H
#define IOT_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Ethernet and Pub/Sub Protocol for ESP32 Robot Controller
 * 
 * Supports the following endpoints:
 * - SUBSCRIBE /torso_cmd - Motor command for torso (angle, speed, accel)
 * - SUBSCRIBE /neck_cmd - Motor command for neck (angle, speed, accel)
 * - SUBSCRIBE /head_cmd - Motor command for head (angle, speed, accel)
 * - PUBLISH /touch - Touch sensor states (tap, double, up, down, long)
 * - PUBLISH /led - LED state control (listen, mute, loading, offline, connecting, low_battery)
 * - SUBSCRIBE /batt - Battery info (batt%, power_source, heartbeat counter)
 * - PUBLISH /reb_jet - Reboot signal to Jetson
 */

/**
 * @brief Callback function type for incoming pub/sub messages
 * 
 * @param topic Topic name (without leading '/')
 * @param payload JSON payload string
 * @param payload_len Length of payload
 */
typedef void (*iot_msg_callback_t)(const char *topic, const uint8_t *payload, uint32_t payload_len);

/**
 * @brief Initialize Ethernet and Pub/Sub Protocol
 * 
 * Initializes Ethernet interface with auto IP assignment (DHCP)
 * and sets up pub/sub message handler.
 * 
 * @param uart_port UART port number for debugging (optional, can be -1)
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t iot_protocol_init(int uart_port);

/**
 * @brief Perform handshake/connection with Windows host
 * 
 * Sends connection handshake to establish communication with Windows system.
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t iot_protocol_handshake(void);

/**
 * @brief Process incoming pub/sub messages
 * 
 * Checks for and processes any incoming messages from Windows host.
 * Should be called periodically in the main loop.
 * 
 * @return Number of messages processed (0 if no messages)
 */
int iot_protocol_process_incoming(void);

/**
 * @brief Publish a message to a topic
 * 
 * Sends a message to the specified topic (to be received by Windows host).
 * 
 * @param topic Topic name (without leading '/')
 * @param payload JSON payload as string
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t iot_protocol_publish(const char *topic, const char *payload);

/**
 * @brief Subscribe to a topic with callback
 * 
 * Registers a callback function for messages on a specific topic.
 * 
 * @param topic Topic name (without leading '/')
 * @param callback Function to call when message arrives on this topic
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t iot_protocol_subscribe(const char *topic, iot_msg_callback_t callback);

/**
 * @brief Get current network status
 * 
 * @return true if connected to network, false otherwise
 */
bool iot_protocol_is_connected(void);

/**
 * @brief Get IP address of ESP32
 * 
 * @param ip_str Buffer to store IP string (minimum 16 bytes)
 * @return true on success, false on error
 */
bool iot_protocol_get_ip_address(char *ip_str);

/**
 * @brief Get MAC address of ESP32
 * 
 * @param mac_str Buffer to store MAC string like "XX:XX:XX:XX:XX:XX" (minimum 18 bytes)
 * @return true on success, false on error
 */
bool iot_protocol_get_mac_address(char *mac_str);

/**
 * @brief Deinitialize protocol and close Ethernet connection
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t iot_protocol_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // IOT_PROTOCOL_H
