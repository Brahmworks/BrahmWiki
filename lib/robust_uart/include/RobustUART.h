/**
 * @file RobustUART.h
 * @brief Core API for RobustUART - A cross-platform packetized UART communication library
 * @author BrahmWiki
 * @version 1.0.0
 * @license MIT
 * 
 * Supports: ESP32 (ESP-IDF/Arduino), ATmega328P (Arduino), and any PlatformIO-supported MCU
 * 
 * Protocol: SOH[LEN][CMD][PAYLOAD...][CHKSUM]EOT with ACK/NAK responses
 */

#ifndef ROBUST_UART_H
#define ROBUST_UART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// ============================================================================
// Platform Detection
// ============================================================================
#if defined(ESP_PLATFORM)
    // ESP-IDF
    #include "driver/uart.h"
    #define ROBUST_UART_PLATFORM_ESP_IDF 1
#elif defined(ARDUINO)
    // Arduino framework (including ESP32 Arduino, ATmega Arduino)
    #if defined(ESP32)
        #define ROBUST_UART_PLATFORM_ESP32_ARDUINO 1
    #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega2560__)
        #define ROBUST_UART_PLATFORM_AVR 1
    #else
        #define ROBUST_UART_PLATFORM_ARDUINO_GENERIC 1
    #endif
#else
    // Generic platform (user provides UART implementation)
    #define ROBUST_UART_PLATFORM_GENERIC 1
#endif

// ============================================================================
// Debug Configuration
// ============================================================================
#ifdef DEBUG_UART_LOG
    #if defined(ARDUINO)
        #include <Arduino.h>
        #define RUART_DEBUG(...) Serial.printf(__VA_ARGS__)
    #elif defined(ESP_PLATFORM)
        #include "esp_log.h"
        #define RUART_DEBUG(...) ESP_LOGI("RobustUART", __VA_ARGS__)
    #else
        #include <stdio.h>
        #define RUART_DEBUG(...) printf(__VA_ARGS__)
    #endif
#else
    #define RUART_DEBUG(...)
#endif

// ============================================================================
// Protocol Constants
// ============================================================================
#define ROBUST_UART_SOH     0x01  // Start of Header
#define ROBUST_UART_EOT     0x04  // End of Transmission
#define ROBUST_UART_ACK     0x06  // Acknowledge
#define ROBUST_UART_NAK     0x15  // Negative Acknowledge
#define ROBUST_UART_CAN     0x18  // Cancel

#define ROBUST_UART_CMD_HEARTBEAT  0x00  // Heartbeat command
#define ROBUST_UART_MAX_PAYLOAD    256   // Maximum payload size
#define ROBUST_UART_TIMEOUT_MS    200    // Frame timeout
#define ROBUST_UART_MAX_RETRIES    3      // Maximum retry attempts

// ============================================================================
// Error Codes
// ============================================================================
typedef enum {
    ROBUST_UART_OK = 0,
    ROBUST_UART_ERR_CRC_FAIL,
    ROBUST_UART_ERR_TIMEOUT,
    ROBUST_UART_ERR_FRAME,
    ROBUST_UART_ERR_BUF_OVERFLOW,
    ROBUST_UART_ERR_INVALID_PARAM,
    ROBUST_UART_ERR_NO_DATA,
    ROBUST_UART_ERR_BUSY,
    ROBUST_UART_ERR_NAK,
    ROBUST_UART_ERR_MAX_RETRIES
} robust_uart_error_t;

// ============================================================================
// Frame States (State Machine)
// ============================================================================
typedef enum {
    FRAME_STATE_IDLE = 0,
    FRAME_STATE_SOH_WAIT,
    FRAME_STATE_LEN,
    FRAME_STATE_CMD,
    FRAME_STATE_PAYLOAD,
    FRAME_STATE_CHK,
    FRAME_STATE_EOT,
    FRAME_STATE_VERIFY,
    FRAME_STATE_ACK_SEND,
    FRAME_STATE_COMPLETE
} frame_state_t;

// ============================================================================
// Configuration Structure
// ============================================================================
typedef struct {
    uint32_t baud_rate;
    int8_t rx_pin;
    int8_t tx_pin;
    uint8_t uart_num;       // UART port number
    size_t rx_buffer_size;  // RX ring buffer size
    size_t tx_buffer_size;  // TX ring buffer size
    uint32_t frame_timeout_ms;
    uint8_t max_retries;
    bool enable_heartbeat;
    uint32_t heartbeat_interval_ms;
} robust_uart_config_t;

// Default configuration
static inline robust_uart_config_t robust_uart_default_config() {
    robust_uart_config_t cfg;
    cfg.baud_rate = 115200;
    cfg.rx_pin = -1;
    cfg.tx_pin = -1;
    cfg.uart_num = 0;
    cfg.rx_buffer_size = 512;
    cfg.tx_buffer_size = 512;
    cfg.frame_timeout_ms = 200;
    cfg.max_retries = 3;
    cfg.enable_heartbeat = true;
    cfg.heartbeat_interval_ms = 5000;
    return cfg;
}

#define ROBUST_UART_DEFAULT_CONFIG robust_uart_default_config()

// ============================================================================
// Statistics Structure
// ============================================================================
typedef struct {
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_error;
    uint32_t retries;
    uint32_t crc_errors;
    uint32_t timeouts;
} robust_uart_stats_t;

// ============================================================================
// Ring Buffer Template Class
// ============================================================================
template<size_t N>
class RingBuffer {
private:
    uint8_t buffer[N];
    volatile size_t head;
    volatile size_t tail;
    size_t count;

public:
    RingBuffer() : head(0), tail(0), count(0) {}

    inline bool isEmpty() const { return count == 0; }
    inline bool isFull() const { return count == N; }
    inline size_t available() const { return count; }
    inline size_t capacity() const { return N; }

    bool write(uint8_t byte) {
        if (isFull()) return false;
        buffer[head] = byte;
        head = (head + 1) % N;
        count++;
        return true;
    }

    bool read(uint8_t& byte) {
        if (isEmpty()) return false;
        byte = buffer[tail];
        tail = (tail + 1) % N;
        count--;
        return true;
    }

    bool peek(uint8_t& byte) const {
        if (isEmpty()) return false;
        byte = buffer[tail];
        return true;
    }

    void clear() {
        head = tail = count = 0;
    }

    size_t write(const uint8_t* data, size_t len) {
        size_t written = 0;
        for (size_t i = 0; i < len && !isFull(); i++) {
            write(data[i]);
            written++;
        }
        return written;
    }

    size_t read(uint8_t* data, size_t len) {
        size_t read_count = 0;
        for (size_t i = 0; i < len && !isEmpty(); i++) {
            if (read(data[i])) {
                read_count++;
            }
        }
        return read_count;
    }
};

// ============================================================================
// Abstract UART Interface (Virtual Base Class)
// ============================================================================
class RobustUARTBase {
public:
    robust_uart_config_t config;
    robust_uart_stats_t stats;
    frame_state_t state;
    uint32_t last_activity_ms;

    RobustUARTBase() : state(FRAME_STATE_IDLE), last_activity_ms(0) {
        config = (robust_uart_config_t)ROBUST_UART_DEFAULT_CONFIG;
        memset(&stats, 0, sizeof(stats));
    }

    virtual ~RobustUARTBase() {}

    // Virtual UART interface - must be implemented by platform-specific classes
    virtual void uart_init(uint32_t baud, int rx_pin, int tx_pin, uint8_t uart_num) = 0;
    virtual size_t uart_write(const uint8_t* data, size_t len) = 0;
    virtual size_t uart_write_byte(uint8_t b) = 0;
    virtual int uart_available() = 0;
    virtual int uart_read() = 0;
    virtual uint32_t millis() = 0;

    // Utility functions
    static uint8_t calculate_xor(const uint8_t* data, size_t len) {
        uint8_t chk = 0;
        for (size_t i = 0; i < len; i++) {
            chk ^= data[i];
        }
        return chk;
    }

    uint32_t elapsed_ms(uint32_t start) {
        uint32_t now = millis();
        return (now >= start) ? (now - start) : (0xFFFFFFFF - start + now);
    }

    void update_activity() {
        last_activity_ms = millis();
    }

    bool check_timeout(uint32_t timeout_ms) {
        return elapsed_ms(last_activity_ms) > timeout_ms;
    }
};

// ============================================================================
// Main RobustUART Class Template
// ============================================================================
template<typename UARTDriver>
class RobustUART : public RobustUARTBase {
private:
    UARTDriver* uart_driver;
    
    // RX/TX buffers
    RingBuffer<512> rx_ring;
    RingBuffer<512> tx_ring;
    
    // Frame parsing variables
    uint8_t frame_buffer[ROBUST_UART_MAX_PAYLOAD + 6]; // SOH + LEN + CMD + PAYLOAD + CHK + EOT
    size_t frame_pos;
    size_t expected_len;
    uint8_t received_cmd;
    uint8_t calculated_chk;

protected:
    // Callbacks (to be overridden or set)
    void (*rx_callback)(uint8_t cmd, uint8_t* data, size_t len);
    void (*tx_complete_callback)();
    void (*error_callback)(robust_uart_error_t err);

public:
    RobustUART(UARTDriver* driver) : 
        RobustUARTBase(), 
        uart_driver(driver),
        frame_pos(0),
        expected_len(0),
        rx_callback(nullptr),
        tx_complete_callback(nullptr),
        error_callback(nullptr) {}

    virtual ~RobustUART() {}

    // Initialize UART with configuration
    void begin(uint32_t baud, int rx_pin = -1, int tx_pin = -1) {
        config.baud_rate = baud;
        config.rx_pin = rx_pin;
        config.tx_pin = tx_pin;
        
        uart_init(baud, rx_pin, tx_pin, config.uart_num);
        state = FRAME_STATE_IDLE;
        rx_ring.clear();
        tx_ring.clear();
        last_activity_ms = millis();
    }

    // Platform-specific implementations
#if defined(ROBUST_UART_PLATFORM_ESP32_ARDUINO) || defined(ROBUST_UART_PLATFORM_ESP_IDF)
    void uart_init(uint32_t baud, int rx_pin, int tx_pin, uint8_t uart_num) override {
        #if defined(ROBUST_UART_PLATFORM_ESP_IDF)
            uart_config_t uart_config = {
                .baud_rate = baud,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .rx_flow_ctrl_thresh = 0,
                .source_clk = UART_SCLK_APB
            };
            uart_param_config(uart_num, &uart_config);
            if (rx_pin >= 0 && tx_pin >= 0) {
                uart_set_pin(uart_num, tx_pin, rx_pin, -1, -1);
            }
            uart_driver_install(uart_num, config.rx_buffer_size, config.tx_buffer_size, 0, NULL, 0);
        #else
            // Arduino ESP32
            if (uart_num == 0) {
                if (rx_pin >= 0 && tx_pin >= 0) {
                    ((HardwareSerial*)uart_driver)->begin(baud, SERIAL_8N1, rx_pin, tx_pin);
                } else {
                    ((HardwareSerial*)uart_driver)->begin(baud);
                }
            }
        #endif
    }

    size_t uart_write(const uint8_t* data, size_t len) override {
        #if defined(ROBUST_UART_PLATFORM_ESP_IDF)
            return uart_write_bytes(uart_num, (const char*)data, len);
        #else
            return ((HardwareSerial*)uart_driver)->write(data, len);
        #endif
    }

    size_t uart_write_byte(uint8_t b) override {
        #if defined(ROBUST_UART_PLATFORM_ESP_IDF)
            return uart_write_bytes(uart_num, (const char*)&b, 1);
        #else
            return ((HardwareSerial*)uart_driver)->write(b);
        #endif
    }

    int uart_available() override {
        #if defined(ROBUST_UART_PLATFORM_ESP_IDF)
            size_t available;
            uart_get_buffered_data_len(uart_num, &available);
            return available;
        #else
            return ((HardwareSerial*)uart_driver)->available();
        #endif
    }

    int uart_read() override {
        #if defined(ROBUST_UART_PLATFORM_ESP_IDF)
            uint8_t byte;
            if (uart_read_bytes(uart_num, &byte, 1, 0) > 0) {
                return byte;
            }
            return -1;
        #else
            return ((HardwareSerial*)uart_driver)->read();
        #endif
    }

    uint32_t millis() override {
        #if defined(ARDUINO)
            return ::millis();
        #elif defined(ESP_PLATFORM)
            return (uint32_t)(esp_timer_get_time() / 1000);
        #else
            return 0;
        #endif
    }

#elif defined(ROBUST_UART_PLATFORM_AVR)
    void uart_init(uint32_t baud, int rx_pin, int tx_pin, uint8_t uart_num) override {
        // AVR uses HardwareSerial - rx_pin and tx_pin are ignored (hardwired)
        ((HardwareSerial*)uart_driver)->begin(baud);
    }

    size_t uart_write(const uint8_t* data, size_t len) override {
        return ((HardwareSerial*)uart_driver)->write(data, len);
    }

    size_t uart_write_byte(uint8_t b) override {
        return ((HardwareSerial*)uart_driver)->write(b);
    }

    int uart_available() override {
        return ((HardwareSerial*)uart_driver)->available();
    }

    int uart_read() override {
        return ((HardwareSerial*)uart_driver)->read();
    }

    uint32_t millis() override {
        return ::millis();
    }

// ============================================================================
// SoftwareSerial Support for AVR
// ============================================================================
#elif defined(ARDUINO) && defined(__AVR__)
    // Include SoftwareSerial if available
    #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H)
        #include <SoftwareSerial.h>
        
        void uart_init(uint32_t baud, int rx_pin, int tx_pin, uint8_t uart_num) override {
            // SoftwareSerial: rx_pin, tx_pin
            if (rx_pin >= 0 && tx_pin >= 0) {
                ((SoftwareSerial*)uart_driver)->begin(baud);
                ((SoftwareSerial*)uart_driver)->setRX(rx_pin);
                ((SoftwareSerial*)uart_driver)->setTX(tx_pin);
                ((SoftwareSerial*)uart_driver)->enableRx(true);
                ((SoftwareSerial*)uart_driver)->enableTx(true);
            }
        }

        size_t uart_write(const uint8_t* data, size_t len) override {
            return ((SoftwareSerial*)uart_driver)->write(data, len);
        }

        size_t uart_write_byte(uint8_t b) override {
            return ((SoftwareSerial*)uart_driver)->write(b);
        }

        int uart_available() override {
            return ((SoftwareSerial*)uart_driver)->available();
        }

        int uart_read() override {
            return ((SoftwareSerial*)uart_driver)->read();
        }

        uint32_t millis() override {
            return ::millis();
        }
    #else
        // No SoftwareSerial available
        void uart_init(uint32_t baud, int rx_pin, int tx_pin, uint8_t uart_num) override {
            RUART_DEBUG("SoftwareSerial not available");
        }
        size_t uart_write(const uint8_t* data, size_t len) override { return 0; }
        size_t uart_write_byte(uint8_t b) override { return 0; }
        int uart_available() override { return 0; }
        int uart_read() override { return -1; }
        uint32_t millis() override { return ::millis(); }
    #endif

#else
    // Generic platform - must provide implementations
    void uart_init(uint32_t baud, int rx_pin, int tx_pin, uint8_t uart_num) override {
        RUART_DEBUG("Generic UART init: baud=%u, rx=%d, tx=%d, uart=%d", baud, rx_pin, tx_pin, uart_num);
    }

    size_t uart_write(const uint8_t* data, size_t len) override { return 0; }
    size_t uart_write_byte(uint8_t b) override { return 0; }
    int uart_available() override { return 0; }
    int uart_read() override { return -1; }
    uint32_t millis() override { return 0; }
#endif

    // ========================================================================
    // Protocol Send Functions
    // ========================================================================
    
    /**
     * @brief Send a packetized frame
     * @param cmd Command byte
     * @param data Payload data
     * @param len Payload length
     * @return true if sent successfully
     */
    bool send_frame(uint8_t cmd, const uint8_t* data, size_t len) {
        if (len > ROBUST_UART_MAX_PAYLOAD) {
            if (error_callback) error_callback(ROBUST_UART_ERR_INVALID_PARAM);
            return false;
        }

        // Build frame: SOH LEN CMD PAYLOAD... CHK EOT
        uint8_t frame[ROBUST_UART_MAX_PAYLOAD + 6];
        size_t pos = 0;
        
        frame[pos++] = ROBUST_UART_SOH;
        frame[pos++] = (uint8_t)len;
        frame[pos++] = cmd;
        
        for (size_t i = 0; i < len; i++) {
            frame[pos++] = data[i];
        }
        
        // Calculate XOR checksum
        uint8_t chk_data[] = { (uint8_t)len, cmd };
        uint8_t chk = calculate_xor(chk_data, 2);
        chk = calculate_xor_contiguous(chk, data, len);
        frame[pos++] = chk;
        
        frame[pos++] = ROBUST_UART_EOT;

        // Send with retries
        for (uint8_t retry = 0; retry < config.max_retries; retry++) {
            // Send frame
            size_t written = uart_write(frame, pos);
            if (written != pos) {
                stats.packets_error++;
                continue;
            }
            
            stats.packets_sent++;
            update_activity();
            
            // Wait for ACK
            uint32_t start = millis();
            while (elapsed_ms(start) < config.frame_timeout_ms) {
                int c = uart_read();
                if (c >= 0) {
                    if (c == ROBUST_UART_ACK) {
                        if (tx_complete_callback) tx_complete_callback();
                        return true;
                    } else if (c == ROBUST_UART_NAK) {
                        stats.retries++;
                        break; // Retry
                    }
                }
            }
            
            stats.retries++;
            RUART_DEBUG("Frame send retry %d", retry + 1);
        }

        stats.packets_error++;
        if (error_callback) error_callback(ROBUST_UART_ERR_MAX_RETRIES);
        return false;
    }

    /**
     * @brief Send command without payload
     */
    bool send_cmd(uint8_t cmd) {
        return send_frame(cmd, nullptr, 0);
    }

    /**
     * @brief Send command with response
     */
    bool send_cmd_with_response(uint8_t cmd, uint8_t* response, size_t resp_len, uint32_t timeout_ms) {
        if (!send_frame(cmd, nullptr, 0)) {
            return false;
        }
        
        // Wait for response
        return wait_response(response, resp_len, timeout_ms);
    }

    /**
     * @brief Wait for response frame
     */
    bool wait_response(uint8_t* response, size_t max_len, uint32_t timeout_ms) {
        uint32_t start = millis();
        
        while (elapsed_ms(start) < timeout_ms) {
            if (process_incoming()) {
                // Got a valid frame
                size_t payload_len = frame_pos > 4 ? frame_pos - 4 : 0;
                if (payload_len <= max_len && response) {
                    memcpy(response, &frame_buffer[3], payload_len);
                }
                
                // Send ACK
                uart_write_byte(ROBUST_UART_ACK);
                return true;
            }
        }
        
        stats.timeouts++;
        if (error_callback) error_callback(ROBUST_UART_ERR_TIMEOUT);
        return false;
    }

    // ========================================================================
    // Protocol Receive Functions
    // ========================================================================
    
    /**
     * @brief Process incoming bytes and parse frames
     * @return true if a complete valid frame was received
     */
    bool process_incoming() {
        while (uart_available() > 0) {
            int c = uart_read();
            if (c < 0) break;
            
            uint8_t byte = (uint8_t)c;
            update_activity();
            
            switch (state) {
                case FRAME_STATE_IDLE:
                    if (byte == ROBUST_UART_SOH) {
                        state = FRAME_STATE_SOH_WAIT;
                        frame_pos = 0;
                        frame_buffer[frame_pos++] = byte;
                    }
                    break;

                case FRAME_STATE_SOH_WAIT:
                    if (byte == ROBUST_UART_SOH) {
                        // Restart
                        frame_pos = 0;
                    }
                    expected_len = byte;
                    if (expected_len > ROBUST_UART_MAX_PAYLOAD) {
                        state = FRAME_STATE_IDLE;
                        stats.crc_errors++;
                        if (error_callback) error_callback(ROBUST_UART_ERR_FRAME);
                        break;
                    }
                    frame_buffer[frame_pos++] = byte;
                    state = FRAME_STATE_LEN;
                    break;

                case FRAME_STATE_LEN:
                    received_cmd = byte;
                    frame_buffer[frame_pos++] = byte;
                    state = FRAME_STATE_CMD;
                    break;

                case FRAME_STATE_CMD:
                    frame_buffer[frame_pos++] = byte;
                    if (frame_pos - 3 >= expected_len) {
                        // All payload received, next is checksum
                        state = FRAME_STATE_PAYLOAD;
                    }
                    break;

                case FRAME_STATE_PAYLOAD:
                    frame_buffer[frame_pos++] = byte;
                    if (frame_pos - 3 >= expected_len) {
                        state = FRAME_STATE_CHK;
                    }
                    break;

                case FRAME_STATE_CHK:
                    calculated_chk = byte;
                    state = FRAME_STATE_EOT;
                    break;

                case FRAME_STATE_EOT:
                    if (byte != ROBUST_UART_EOT) {
                        // Invalid frame
                        state = FRAME_STATE_IDLE;
                        stats.crc_errors++;
                        if (error_callback) error_callback(ROBUST_UART_ERR_FRAME);
                        break;
                    }
                    frame_buffer[frame_pos++] = byte;
                    
                    // Verify checksum
                    {
                        uint8_t chk_data[ROBUST_UART_MAX_PAYLOAD + 2];
                        chk_data[0] = expected_len;
                        chk_data[1] = received_cmd;
                        memcpy(&chk_data[2], &frame_buffer[3], expected_len);
                        uint8_t chk = calculate_xor(chk_data, expected_len + 2);
                        
                        if (chk == calculated_chk) {
                            state = FRAME_STATE_COMPLETE;
                            stats.packets_received++;
                            return true;
                        } else {
                            stats.crc_errors++;
                            if (error_callback) error_callback(ROBUST_UART_ERR_CRC_FAIL);
                            state = FRAME_STATE_IDLE;
                        }
                    }
                    break;

                default:
                    state = FRAME_STATE_IDLE;
                    break;
            }
            
            // Check for timeout
            if (check_timeout(config.frame_timeout_ms)) {
                state = FRAME_STATE_IDLE;
                stats.timeouts++;
                if (error_callback) error_callback(ROBUST_UART_ERR_TIMEOUT);
            }
        }
        
        return false;
    }

    /**
     * @brief Get the last received command and payload
     */
    bool get_last_frame(uint8_t& cmd, uint8_t* data, size_t& len) {
        if (state != FRAME_STATE_COMPLETE) return false;
        
        cmd = frame_buffer[2]; // CMD is at position 2 after SOH and LEN
        len = frame_pos > 4 ? frame_pos - 4 : 0; // SOH, LEN, CMD, CHK, EOT
        if (data && len > 0) {
            memcpy(data, &frame_buffer[3], len);
        }
        
        state = FRAME_STATE_IDLE;
        return true;
    }

    /**
     * @brief Send ACK
     */
    void send_ack() {
        uart_write_byte(ROBUST_UART_ACK);
    }

    /**
     * @brief Send NAK
     */
    void send_nak() {
        uart_write_byte(ROBUST_UART_NAK);
    }

    // ========================================================================
    // Heartbeat
    // ========================================================================
    
    void send_heartbeat() {
        send_cmd(ROBUST_UART_CMD_HEARTBEAT);
    }

    bool should_send_heartbeat() {
        return config.enable_heartbeat && 
               elapsed_ms(last_activity_ms) > config.heartbeat_interval_ms;
    }

    // ========================================================================
    // Main tick function for state machine
    // ========================================================================
    
    void tick() {
        // Process incoming data
        if (process_incoming()) {
            uint8_t cmd;
            uint8_t data[ROBUST_UART_MAX_PAYLOAD];
            size_t len;
            
            if (get_last_frame(cmd, data, len)) {
                // Handle heartbeat
                if (cmd == ROBUST_UART_CMD_HEARTBEAT) {
                    send_ack();
                    return;
                }
                
                // Call user callback
                if (rx_callback) {
                    rx_callback(cmd, data, len);
                }
            }
        }
        
        // Send heartbeat if needed
        if (should_send_heartbeat()) {
            send_heartbeat();
        }
    }

    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void on_rx_frame(void (*callback)(uint8_t cmd, uint8_t* data, size_t len)) {
        rx_callback = callback;
    }

    void on_tx_complete(void (*callback)()) {
        tx_complete_callback = callback;
    }

    void on_error(void (*callback)(robust_uart_error_t err)) {
        error_callback = callback;
    }

    // ========================================================================
    // Statistics
    // ========================================================================
    
    robust_uart_stats_t get_stats() const { return stats; }
    void reset_stats() { memset((robust_uart_stats_t*)&stats, 0, sizeof(stats)); }

private:
    // Helper for continuous XOR calculation
    static uint8_t calculate_xor_contiguous(uint8_t initial, const uint8_t* data, size_t len) {
        uint8_t result = initial;
        for (size_t i = 0; i < len; i++) {
            result ^= data[i];
        }
        return result;
    }
};

#endif // ROBUST_UART_H
