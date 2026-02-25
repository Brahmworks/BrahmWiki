/**
 * @file RobustUART_host.h
 * @brief Host functionality for RobustUART library
 * @author BrahmWiki
 * @version 1.0.0
 * @license MIT
 * 
 * Provides Host (master) functionality for initiating commands
 * and waiting for responses from Client devices.
 */

#ifndef ROBUST_UART_HOST_H
#define ROBUST_UART_HOST_H

#include "RobustUART.h"

// ============================================================================
// Command IDs (Application-specific)
// ============================================================================
#define ROBUST_UART_CMD_PING         0x01  // Ping command
#define ROBUST_UART_CMD_READ_SENSOR  0x02  // Read sensor data
#define ROBUST_UART_CMD_WRITE_ACTUATOR 0x03 // Write to actuator
#define ROBUST_UART_CMD_GET_STATUS   0x04  // Get device status
#define ROBUST_UART_CMD_RESET        0x05  // Reset device
#define ROBUST_UART_CMD_FIRMWARE    0x06  // Get firmware version

// ============================================================================
// RobustHost Class Template
// ============================================================================
template<typename UARTDriver>
class RobustHost : public RobustUART<UARTDriver> {
private:
    uint8_t response_buffer[ROBUST_UART_MAX_PAYLOAD];
    size_t last_response_len;
    uint32_t last_response_timeout;

public:
    RobustHost(UARTDriver* driver) : RobustUART<UARTDriver>(driver), 
        last_response_len(0), last_response_timeout(0) {}

    // ========================================================================
    // High-level Commands
    // ========================================================================

    /**
     * @brief Send a ping command and wait for response
     * @param timeout_ms Timeout in milliseconds
     * @return true if ping successful
     */
    bool ping(uint32_t timeout_ms = 500) {
        return this->send_cmd_with_response(ROBUST_UART_CMD_PING, 
            response_buffer, sizeof(response_buffer), timeout_ms);
    }

    /**
     * @brief Read sensor data from client
     * @param sensor_id Sensor identifier
     * @param data Buffer for sensor data
     * @param len Buffer length
     * @param timeout_ms Timeout in milliseconds
     * @return true if successful
     */
    bool read_sensor(uint8_t sensor_id, uint8_t* data, size_t& len, uint32_t timeout_ms = 500) {
        uint8_t payload[1] = { sensor_id };
        if (!this->send_frame(ROBUST_UART_CMD_READ_SENSOR, payload, 1)) {
            return false;
        }
        
        if (this->wait_response(response_buffer, sizeof(response_buffer), timeout_ms)) {
            len = last_response_len;
            memcpy(data, response_buffer, len);
            return true;
        }
        return false;
    }

    /**
     * @brief Write to an actuator
     * @param actuator_id Actuator identifier
     * @param value Value to write
     * @param value_len Value length
     * @param timeout_ms Timeout in milliseconds
     * @return true if successful
     */
    bool write_actuator(uint8_t actuator_id, const uint8_t* value, size_t value_len, uint32_t timeout_ms = 500) {
        uint8_t payload[ROBUST_UART_MAX_PAYLOAD];
        payload[0] = actuator_id;
        memcpy(&payload[1], value, value_len);
        
        return this->send_cmd_with_response(ROBUST_UART_CMD_WRITE_ACTUATOR, 
            payload, value_len + 1, timeout_ms);
    }

    /**
     * @brief Get device status
     * @param status Buffer for status data
     * @param len Buffer length (updated with actual length)
     * @param timeout_ms Timeout in milliseconds
     * @return true if successful
     */
    bool get_status(uint8_t* status, size_t& len, uint32_t timeout_ms = 500) {
        if (!this->send_cmd(ROBUST_UART_CMD_GET_STATUS)) {
            return false;
        }
        
        if (this->wait_response(response_buffer, sizeof(response_buffer), timeout_ms)) {
            len = last_response_len;
            memcpy(status, response_buffer, len);
            return true;
        }
        return false;
    }

    /**
     * @brief Reset the client device
     * @param timeout_ms Timeout in milliseconds
     * @return true if successful
     */
    bool reset(uint32_t timeout_ms = 500) {
        return this->send_cmd_with_response(ROBUST_UART_CMD_RESET, 
            response_buffer, sizeof(response_buffer), timeout_ms);
    }

    /**
     * @brief Get firmware version
     * @param version Buffer for version string
     * @param len Buffer length (updated with actual length)
     * @param timeout_ms Timeout in milliseconds
     * @return true if successful
     */
    bool get_firmware_version(uint8_t* version, size_t& len, uint32_t timeout_ms = 500) {
        if (!this->send_cmd(ROBUST_UART_CMD_FIRMWARE)) {
            return false;
        }
        
        if (this->wait_response(response_buffer, sizeof(response_buffer), timeout_ms)) {
            len = last_response_len;
            memcpy(version, response_buffer, len);
            return true;
        }
        return false;
    }

    /**
     * @brief Generic send command with response
     * @param cmd Command byte
     * @param data Optional payload data
     * @param len Payload length
     * @param response Buffer for response
     * @param resp_len Response buffer length (updated with actual length)
     * @param timeout_ms Timeout in milliseconds
     * @return true if successful
     */
    bool send_command(uint8_t cmd, const uint8_t* data, size_t len,
                      uint8_t* response, size_t& resp_len, uint32_t timeout_ms) {
        if (len > 0) {
            if (!this->send_frame(cmd, data, len)) {
                return false;
            }
        } else {
            if (!this->send_cmd(cmd)) {
                return false;
            }
        }
        
        if (this->wait_response(response, resp_len, timeout_ms)) {
            resp_len = last_response_len;
            return true;
        }
        return false;
    }

    // ========================================================================
    // Response Access
    // ========================================================================

    const uint8_t* get_response() const { return response_buffer; }
    size_t get_response_length() const { return last_response_len; }

    /**
     * @brief Set callback for responses
     */
    void on_response(void (*callback)(const uint8_t* data, size_t len)) {
        this->on_rx_frame([callback](uint8_t cmd, uint8_t* data, size_t len) {
            callback(data, len);
        });
    }
};

#endif // ROBUST_UART_HOST_H
