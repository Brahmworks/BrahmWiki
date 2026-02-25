/**
 * @file RobustUART_client.h
 * @brief Client functionality for RobustUART library
 * @author BrahmWiki
 * @version 1.0.0
 * @license MIT
 * 
 * Provides Client (slave) functionality for responding to commands
 * from a Host device.
 */

#ifndef ROBUST_UART_CLIENT_H
#define ROBUST_UART_CLIENT_H

#include "RobustUART.h"

// ============================================================================
// RobustClient Class Template
// ============================================================================
template<typename UARTDriver>
class RobustClient : public RobustUART<UARTDriver> {
private:
    // Command handler function pointer type
    typedef void (*CommandHandler)(uint8_t cmd, const uint8_t* data, size_t len, 
                                   RobustClient* client);
    
    // Command handler table
    struct HandlerEntry {
        uint8_t cmd;
        CommandHandler handler;
    };
    
    static const size_t MAX_HANDLERS = 16;
    HandlerEntry handlers[MAX_HANDLERS];
    size_t handler_count;
    
    // Response data
    uint8_t response_buffer[ROBUST_UART_MAX_PAYLOAD];
    size_t response_len;

public:
    RobustClient(UARTDriver* driver) : RobustUART<UARTDriver>(driver), 
        handler_count(0), response_len(0) {
        // Register default command handlers
        register_default_handlers();
    }

    // ========================================================================
    // Command Handler Registration
    // ========================================================================

    /**
     * @brief Register a handler for a specific command
     * @param cmd Command byte
     * @param handler Handler function
     * @return true if registered successfully
     */
    bool register_handler(uint8_t cmd, void (*handler)(uint8_t cmd, uint8_t* data, size_t len)) {
        if (handler_count >= MAX_HANDLERS) {
            return false;
        }
        
        handlers[handler_count].cmd = cmd;
        handlers[handler_count].handler = handler;
        handler_count++;
        return true;
    }

    /**
     * @brief Unregister a handler for a specific command
     * @param cmd Command byte
     * @return true if unregistered successfully
     */
    bool unregister_handler(uint8_t cmd) {
        for (size_t i = 0; i < handler_count; i++) {
            if (handlers[i].cmd == cmd) {
                // Shift remaining handlers
                for (size_t j = i; j < handler_count - 1; j++) {
                    handlers[j] = handlers[j + 1];
                }
                handler_count--;
                return true;
            }
        }
        return false;
    }

    // ========================================================================
    // Response Methods
    // ========================================================================

    /**
     * @brief Set response data to send back to host
     * @param data Response data
     * @param len Response data length
     */
    void set_response(const uint8_t* data, size_t len) {
        if (len > ROBUST_UART_MAX_PAYLOAD) {
            len = ROBUST_UART_MAX_PAYLOAD;
        }
        memcpy(response_buffer, data, len);
        response_len = len;
    }

    /**
     * @brief Set string response
     * @param str String response (null-terminated)
     */
    void set_response_str(const char* str) {
        response_len = strlen(str);
        if (response_len > ROBUST_UART_MAX_PAYLOAD) {
            response_len = ROBUST_UART_MAX_PAYLOAD;
        }
        memcpy(response_buffer, str, response_len);
    }

    /**
     * @brief Send response to host (call after processing command)
     */
    void send_response() {
        this->send_frame(0x00, response_buffer, response_len);
    }

    /**
     * @brief Send success response
     */
    void send_success() {
        const uint8_t success[] = { 0x01 }; // 1 = success
        this->send_frame(0x00, success, 1);
    }

    /**
     * @brief Send error response
     * @param error_code Error code
     */
    void send_error(uint8_t error_code) {
        this->send_frame(0x00, &error_code, 1);
    }

    // ========================================================================
    // Main Loop
    // ========================================================================

    /**
     * @brief Process incoming commands (call this in loop)
     * This handles the protocol state machine and dispatches commands
     */
    void process() {
        this->tick();
    }

    /**
     * @brief Start listening for commands
     * Call this in setup()
     */
    void begin(uint32_t baud, int rx_pin = -1, int tx_pin = -1) {
        RobustUART<UARTDriver>::begin(baud, rx_pin, tx_pin);
        
        // Set up default frame callback
        this->on_rx_frame([this](uint8_t cmd, uint8_t* data, size_t len) {
            this->handle_command(cmd, data, len);
        });
    }

    // ========================================================================
    // Default Handlers (can be overridden)
    // ========================================================================

    /**
     * @brief Handle ping command (default implementation)
     */
    virtual void handle_ping(uint8_t* data, size_t len) {
        const char* response = "PONG";
        set_response_str(response);
        send_response();
    }

    /**
     * @brief Handle read sensor command (default implementation)
     * Override this in subclass for custom sensor handling
     */
    virtual void handle_read_sensor(uint8_t sensor_id, uint8_t* data, size_t len) {
        // Default: return dummy data
        // Override in subclass for real implementation
        uint8_t response[] = { sensor_id, 0x00 };
        set_response(response, 2);
        send_response();
    }

    /**
     * @brief Handle write actuator command (default implementation)
     * Override this in subclass for custom actuator handling
     */
    virtual void handle_write_actuator(uint8_t actuator_id, uint8_t* data, size_t len) {
        // Default: acknowledge
        uint8_t response[] = { actuator_id, 0x01 }; // 0x01 = acknowledged
        set_response(response, 2);
        send_response();
    }

    /**
     * @brief Handle get status command (default implementation)
     * Override this in subclass for custom status
     */
    virtual void handle_get_status(uint8_t* data, size_t len) {
        // Default: return basic status
        uint8_t status[] = { 0x01, 0x00 }; // status OK, no errors
        set_response(status, sizeof(status));
        send_response();
    }

    /**
     * @brief Handle reset command (default implementation)
     */
    virtual void handle_reset(uint8_t* data, size_t len) {
        send_success();
        // Note: Actual reset should be done by calling wdt_disable() and jump to bootloader
        // or software reset depending on platform
    }

    /**
     * @brief Handle firmware version command (default implementation)
     */
    virtual void handle_firmware(uint8_t* data, size_t len) {
        const char* version = "1.0.0";
        set_response_str(version);
        send_response();
    }

protected:
    /**
     * @brief Dispatch command to appropriate handler
     */
    void handle_command(uint8_t cmd, uint8_t* data, size_t len) {
        // Check registered handlers first
        for (size_t i = 0; i < handler_count; i++) {
            if (handlers[i].cmd == cmd) {
                if (handlers[i].handler) {
                    handlers[i].handler(cmd, data, len, this);
                }
                return;
            }
        }
        
        // Fall back to default handlers
        switch (cmd) {
            case ROBUST_UART_CMD_PING:
                handle_ping(data, len);
                break;
            case ROBUST_UART_CMD_READ_SENSOR:
                if (len > 0) {
                    handle_read_sensor(data[0], data + 1, len - 1);
                }
                break;
            case ROBUST_UART_CMD_WRITE_ACTUATOR:
                if (len > 0) {
                    handle_write_actuator(data[0], data + 1, len - 1);
                }
                break;
            case ROBUST_UART_CMD_GET_STATUS:
                handle_get_status(data, len);
                break;
            case ROBUST_UART_CMD_RESET:
                handle_reset(data, len);
                break;
            case ROBUST_UART_CMD_FIRMWARE:
                handle_firmware(data, len);
                break;
            default:
                // Unknown command
                send_error(0xFF);
                break;
        }
    }

private:
    /**
     * @brief Register default command handlers
     */
    void register_default_handlers() {
        // Default handlers are handled in handle_command
        // This space is reserved for custom handlers
    }
};

#endif // ROBUST_UART_CLIENT_H
