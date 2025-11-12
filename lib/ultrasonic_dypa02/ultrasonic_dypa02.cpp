/*
 * ultrasonic_dypa02.cpp
 *
 * Cross-platform implementation (Arduino/ESP-IDF)
 * for DYP-A02 UART ultrasonic sensor.
 *
 * MODULAR/STRUCTURED/ROBUST VERSION
 * - Uses a dyp_sensor_t "handle" to support multiple sensor instances.
 * - Each instance is thread-safe on ESP-IDF.
 * - Uses a dyp_err_t enum for clear error handling.
 * - Stores last error in the sensor handle.
 */

#include "ultrasonic_dypa02.h"
#include <string.h> // for memset

#if defined(ESP_PLATFORM)
  /* ESP-IDF */
  #include "esp_log.h"
  #include "esp_timer.h"
  #include "freertos/task.h" // For vTaskDelay
  static const char *TAG_DYP = "dyp_a02";
#elif defined(ARDUINO)
  /* Arduino */
  #include <Arduino.h>
  // Stream.h and SoftwareSerial.h are included via dyp_a02_uart.h
#else
  #error "Either ESP_PLATFORM (ESP-IDF) or ARDUINO must be defined"
#endif


/* ---------- Internal state (from .h) ---------- */

#if defined(ESP_PLATFORM)
    // Internal modes
    #define MODE_NONE 0
    #define MODE_HW_ESP 1
#elif defined(ARDUINO)
    // Internal modes
    #define MODE_NONE 0
    #define MODE_STREAM_ARDUINO 2
    #define MODE_SW_ARDUINO 3
#endif


/* ---------- Mutex Helpers (ESP-IDF) ---------- */

#if defined(ESP_PLATFORM)
    static bool dyp_mutex_take(dyp_sensor_t *sensor) {
        if (!sensor || !sensor->mutex) return false;
        return (xSemaphoreTakeRecursive(sensor->mutex, portMAX_DELAY) == pdTRUE);
    }
    static void dyp_mutex_give(dyp_sensor_t *sensor) {
        if (sensor && sensor->mutex) {
            xSemaphoreGiveRecursive(sensor->mutex);
        }
    }
    static dyp_err_t dyp_init_mutex(dyp_sensor_t *sensor) {
        if (sensor->mutex == NULL) {
            sensor->mutex = xSemaphoreCreateRecursiveMutex();
            if (sensor->mutex == NULL) {
                return DYP_ERR_MUTEX_FAIL;
            }
        }
        return DYP_OK;
    }
    static void dyp_deinit_mutex(dyp_sensor_t *sensor) {
        if (sensor->mutex != NULL) {
            vSemaphoreDelete(sensor->mutex);
            sensor->mutex = NULL;
        }
    }
#else
    // On Arduino, these macros do nothing.
    #define dyp_mutex_take(sensor) (true)
    #define dyp_mutex_give(sensor)
    #define dyp_init_mutex(sensor) (DYP_OK)
    #define dyp_deinit_mutex(sensor)
#endif


/* ---------- Internal Helpers ---------- */

/**
 * @brief Internal: Get current time in milliseconds.
 */
static uint64_t dyp_get_time_ms(void) {
#if defined(ESP_PLATFORM)
    return esp_timer_get_time() / 1000ULL;
#elif defined(ARDUINO)
    return millis();
#endif
}

/**
 * @brief Internal: Yield the task.
 * THIS IS CRITICAL on ESP-IDF to allow the UART ISR to run.
 */
static void dyp_task_yield(void) {
#if defined(ESP_PLATFORM)
    // This delay (1ms or 1 tick) is critical to allow
    // the background UART ISR to run and fill its buffer.
    vTaskDelay(pdMS_TO_TICKS(1)); 
#elif defined(ARDUINO)
    // On Arduino, delay(0) or yield() can be used
    delay(1);
#endif
}


/**
 * @brief Internal: Read an exact number of bytes from the active peripheral.
 * @note This function MUST be called *after* taking the mutex.
 */
static dyp_err_t read_bytes_internal(dyp_sensor_t *sensor, uint8_t *buf, size_t len, uint32_t timeout_ms) {
    if (sensor->mode == MODE_NONE || !buf) {
        return DYP_ERR_NOT_INITIALIZED;
    }

#if defined(ESP_PLATFORM)
    if (sensor->mode == MODE_HW_ESP) {
        size_t received = 0;
        uint64_t start_time = dyp_get_time_ms();

        while (received < len) {
            int bytes_read = uart_read_bytes(sensor->uart_num, buf + received, len - received, pdMS_TO_TICKS(timeout_ms));
            
            if (bytes_read > 0) {
                received += bytes_read;
                continue; // Continue reading immediately
            }

            // Check for timeout
            if ((dyp_get_time_ms() - start_time) > timeout_ms) {
                return DYP_ERR_TIMEOUT;
            }
            
            // --- CRITICAL ---
            // Add a small yield to allow the UART ISR to fill the
            // background buffer, just like in the user's original code.
            dyp_task_yield();
        }
        return DYP_OK;
    }
#elif defined(ARDUINO)
    if (sensor->mode == MODE_STREAM_ARDUINO || sensor->mode == MODE_SW_ARDUINO) {
        if (!sensor->stream) return DYP_ERR_NOT_INITIALIZED;
        
        sensor->stream->setTimeout(timeout_ms);
        size_t bytes_read = sensor->stream->readBytes(buf, len);
        
        if (bytes_read == len) {
            return DYP_OK;
        } else {
            return DYP_ERR_TIMEOUT;
        }
    }
#endif
    return DYP_ERR_NOT_SUPPORTED;
}

/**
 * @brief Internal: Read a single packet.
 * @note This function MUST be called *after* taking the mutex.
 */
static dyp_err_t read_packet_internal(dyp_sensor_t *sensor, uint16_t *distance_mm_out, uint32_t timeout_ms) {
    uint8_t b;
    uint8_t payload[3];
    dyp_err_t err;
    
    uint64_t start_time_ms = dyp_get_time_ms();

    // 1. Find the start byte 0xFF
    while (1) {
        err = read_bytes_internal(sensor, &b, 1, timeout_ms);
        if (err == DYP_OK) {
            if (b == 0xFF) {
                break; // Found start byte
            }
            // Got a byte, but not the header. Loop continues.
        } else {
             // read_bytes_internal failed (e.g., timeout)
            return err;
        }

        // Check overall timeout
        if (dyp_get_time_ms() - start_time_ms > timeout_ms) {
            return DYP_ERR_TIMEOUT; // Timeout waiting for start byte
        }
        
        // --- CRITICAL ---
        // Add a small yield to prevent this loop from starving
        // the UART ISR if the buffer is full of garbage.
        dyp_task_yield();
    }

    // 2. Read the 3-byte payload
    err = read_bytes_internal(sensor, payload, 3, timeout_ms);
    if (err != DYP_OK) {
        return err; // Timeout reading payload
    }

    // 3. Parse and verify
    uint8_t data_h = payload[0];
    uint8_t data_l = payload[1];
    uint8_t checksum = payload[2];

    uint16_t distance_mm = ((uint16_t)data_h << 8) | (uint16_t)data_l;
    uint8_t calculated_cs = (uint8_t)((0xFF + data_h + data_l) & 0xFF);

    if (checksum != calculated_cs) {
        #if defined(ESP_PLATFORM)
        ESP_LOGW(TAG_DYP, "Checksum mismatch: calc=0x%02X recv=0x%02X", calculated_cs, checksum);
        #endif
        return DYP_ERR_CHECKSUM;
    }

    // Per datasheet, 0 is not a valid distance.
    if (distance_mm == 0) {
        return DYP_ERR_INVALID_READING;
    }

    *distance_mm_out = distance_mm;
    return DYP_OK;
}

/* ---------- Public API ---------- */

void dyp_a02_deinit(dyp_sensor_t *sensor) {
    if (!sensor || !sensor->is_initialized) {
        return;
    }

    // Now we check the return value. On Arduino, this is just `if(true)`.
    // On ESP-IDF, this adds safety by not proceeding if the mutex is locked.
    if (!dyp_mutex_take(sensor)) {
        return; // Failed to get lock, cannot deinit
    }

#if defined(ESP_PLATFORM)
    if (sensor->mode == MODE_HW_ESP && sensor->uart_num != UART_NUM_MAX) {
        uart_driver_delete(sensor->uart_num);
        sensor->uart_num = UART_NUM_MAX;
    }
#elif defined(ARDUINO)
    if (sensor->mode == MODE_SW_ARDUINO && sensor->sw_serial_inst) {
        // We own this instance, so we delete it.
        // The [-Wdelete-non-virtual-dtor] warning here is from the
        // Arduino core library (SoftwareSerial) and is safe to ignore.
        delete sensor->sw_serial_inst;
        sensor->sw_serial_inst = NULL;
    }
    sensor->stream = NULL;
#endif

    sensor->mode = MODE_NONE;
    sensor->is_initialized = false;
    
    // Give mutex before deleting it
    dyp_mutex_give(sensor);
    dyp_deinit_mutex(sensor);
}

dyp_err_t dyp_a02_init(dyp_sensor_t *sensor, int rx_pin, int tx_pin) {
    if (!sensor) return DYP_ERR_INVALID_ARG;

    dyp_a02_deinit(sensor); // Clean up previous state

#if defined(ESP_PLATFORM)
    // On ESP-IDF, default init calls the HW init
    return dyp_a02_init_esp_hw(sensor, DYP_A02_DEFAULT_UART_PORT, rx_pin, tx_pin);

#elif defined(ARDUINO)
    // On Arduino, default init creates a new SoftwareSerial
    dyp_err_t err = dyp_init_mutex(sensor);
    if (err != DYP_OK) return err;

    if (!dyp_mutex_take(sensor)) return DYP_ERR_MUTEX_FAIL;

    sensor->sw_serial_inst = new SoftwareSerial(rx_pin, tx_pin);
    if (!sensor->sw_serial_inst) {
        dyp_mutex_give(sensor);
        return DYP_ERR_NO_MEM; // Out of memory
    }
    sensor->stream = sensor->sw_serial_inst;
    sensor->sw_serial_inst->begin(DYP_A02_DEFAULT_BAUD);
    sensor->mode = MODE_SW_ARDUINO;
    sensor->is_initialized = true;
    
    dyp_mutex_give(sensor);
    return DYP_OK;
#endif
}

dyp_err_t dyp_a02_init_stream(dyp_sensor_t *sensor, void *stream_ptr) {
#if defined(ESP_PLATFORM)
    if (!sensor) return DYP_ERR_INVALID_ARG;
    sensor->last_error = DYP_ERR_NOT_SUPPORTED;
    return DYP_ERR_NOT_SUPPORTED;
#elif defined(ARDUINO)
    if (!sensor || !stream_ptr) return DYP_ERR_INVALID_ARG;
    
    dyp_a02_deinit(sensor);
    
    dyp_err_t err = dyp_init_mutex(sensor);
    if (err != DYP_OK) return err;

    if (!dyp_mutex_take(sensor)) return DYP_ERR_MUTEX_FAIL;

    sensor->stream = (Stream*)stream_ptr;
    // We assume the user has already called .begin() on the stream
    sensor->mode = MODE_STREAM_ARDUINO;
    sensor->is_initialized = true;

    dyp_mutex_give(sensor);
    return DYP_OK;
#endif
}

dyp_err_t dyp_a02_init_esp_hw(dyp_sensor_t *sensor, int uart_port_num, int rx_pin, int tx_pin) {
#if defined(ESP_PLATFORM)
    if (!sensor) return DYP_ERR_INVALID_ARG;

    dyp_a02_deinit(sensor);

    dyp_err_t err = dyp_init_mutex(sensor);
    if (err != DYP_OK) return err;

    if (!dyp_mutex_take(sensor)) return DYP_ERR_MUTEX_FAIL;

    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config)); // Zero initialize
    uart_config.baud_rate = DYP_A02_DEFAULT_BAUD;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;
    
    esp_err_t ret = uart_param_config((uart_port_t)uart_port_num, &uart_config);
    if (ret != ESP_OK) {
        sensor->last_error = DYP_ERR_UART_CONFIG;
        dyp_mutex_give(sensor);
        return DYP_ERR_UART_CONFIG;
    }

    ret = uart_set_pin((uart_port_t)uart_port_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        sensor->last_error = DYP_ERR_UART_PIN;
        dyp_mutex_give(sensor);
        return DYP_ERR_UART_PIN;
    }

    // Install driver
    ret = uart_driver_install((uart_port_t)uart_port_num, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        sensor->last_error = DYP_ERR_UART_INSTALL;
        dyp_mutex_give(sensor);
        return DYP_ERR_UART_INSTALL;
    }

    uart_flush((uart_port_t)uart_port_num);
    sensor->uart_num = (uart_port_t)uart_port_num;
    sensor->mode = MODE_HW_ESP;
    sensor->is_initialized = true;
    sensor->last_error = DYP_OK;

    dyp_mutex_give(sensor);
    return DYP_OK;

#elif defined(ARDUINO)
    if (!sensor) return DYP_ERR_INVALID_ARG;
    sensor->last_error = DYP_ERR_NOT_SUPPORTED;
    return DYP_ERR_NOT_SUPPORTED;
#endif
}

bool dyp_a02_read_packet(dyp_sensor_t *sensor, uint16_t *distance_mm_out, uint32_t timeout_ms) {
    if (!sensor || !distance_mm_out) {
        if(sensor) sensor->last_error = DYP_ERR_INVALID_ARG;
        return false;
    }
    if (!dyp_mutex_take(sensor)) {
        sensor->last_error = DYP_ERR_MUTEX_FAIL;
        return false;
    }

    dyp_err_t ret = read_packet_internal(sensor, distance_mm_out, timeout_ms);
    sensor->last_error = ret;
    
    dyp_mutex_give(sensor);
    return (ret == DYP_OK);
}

long dyp_a02_read_distance_int(dyp_sensor_t *sensor, uint32_t timeout_ms) {
    uint16_t dist;
    
    if (!sensor) return DYP_A02_READ_ERROR;
    if (!dyp_mutex_take(sensor)) {
        sensor->last_error = DYP_ERR_MUTEX_FAIL;
        return DYP_A02_READ_ERROR;
    }

    dyp_err_t ok = read_packet_internal(sensor, &dist, timeout_ms);
    sensor->last_error = ok;
    
    dyp_mutex_give(sensor);
    
    if (ok == DYP_OK) {
        return (long)dist;
    } else {
        return DYP_A02_READ_ERROR;
    }
}

long dyp_a02_read_average_int(dyp_sensor_t *sensor, int num_samples, uint32_t per_read_timeout_ms) {
    if (!sensor || num_samples < 1) {
        if(sensor) sensor->last_error = DYP_ERR_INVALID_ARG;
        return DYP_A02_READ_ERROR;
    }
    
    uint32_t sum = 0;
    int valid_samples = 0;
    
    if (!dyp_mutex_take(sensor)) {
        sensor->last_error = DYP_ERR_MUTEX_FAIL;
        return DYP_A02_READ_ERROR;
    }
    
    while (valid_samples < num_samples) {
        uint16_t dist;
        dyp_err_t ok = read_packet_internal(sensor, &dist, per_read_timeout_ms);
        if (ok == DYP_OK) {
            sum += dist;
            valid_samples++;
        } else {
            // Store the first error we see
            if (sensor->last_error == DYP_OK) {
                sensor->last_error = ok;
            }
            // Break on error to avoid endless loop
            break;
        }
    }
    
    dyp_mutex_give(sensor);

    if (valid_samples == num_samples) {
        sensor->last_error = DYP_OK;
        return (long)(sum / num_samples);
    } else {
        // We failed, sensor->last_error is already set
        return DYP_A02_READ_ERROR;
    }
}

long dyp_a02_read_average_filtered_int(dyp_sensor_t *sensor,
                                       int num_samples,
                                       uint16_t min_mm,
                                       uint16_t max_mm,
                                       uint32_t per_read_timeout_ms,
                                       uint32_t overall_timeout_ms)
{
    if (!sensor || num_samples < 1) {
        if(sensor) sensor->last_error = DYP_ERR_INVALID_ARG;
        return DYP_A02_READ_ERROR;
    }

    uint32_t sum = 0;
    int valid_samples = 0;
    uint64_t start_time = dyp_get_time_ms();

    if (!dyp_mutex_take(sensor)) {
        sensor->last_error = DYP_ERR_MUTEX_FAIL;
        return DYP_A02_READ_ERROR;
    }

    // Clear last error for this operation
    sensor->last_error = DYP_OK;

    while (valid_samples < num_samples) {
        // Check overall timeout
        if ((dyp_get_time_ms() - start_time) > overall_timeout_ms) {
            sensor->last_error = DYP_ERR_TIMEOUT_OVERALL;
            break;
        }

        uint16_t dist;
        dyp_err_t err = read_packet_internal(sensor, &dist, per_read_timeout_ms);

        if (err == DYP_OK) {
            // Packet is valid, check range
            if (dist >= min_mm && dist <= max_mm) {
                sum += dist;
                valid_samples++;
            } else {
                // Out of range, ignore sample
                #if defined(ESP_PLATFORM)
                ESP_LOGD(TAG_DYP, "Filtered out-of-range: %u mm", (unsigned)dist);
                #endif
            }
        } else {
            // A read error occurred (e.g., checksum, timeout)
            // Store the first error we see
            if (sensor->last_error == DYP_OK) {
                sensor->last_error = err;
            }
            // Don't break, just keep trying until overall timeout
            
            // --- CRITICAL ---
            // Add a yield here to prevent a tight spin-loop on
            // continuous read failures. This was the missing piece.
            dyp_task_yield();
        }
    }

    dyp_mutex_give(sensor);

    if (valid_samples == num_samples) {
        sensor->last_error = DYP_OK;
        return (long)(sum / num_samples);
    }

    // Failure, set error if not already set
    if (sensor->last_error == DYP_OK) {
        sensor->last_error = DYP_ERR_TIMEOUT_OVERALL;
    }
    return DYP_A02_READ_ERROR;
}


/**
 * @brief NEW ROBUST FUNCTION: Gets a stable average distance.
 */
long dyp_a02_get_stable_distance_mm(dyp_sensor_t *sensor,
                                    int num_samples_to_try,
                                    int min_required_samples,
                                    uint32_t per_read_timeout_ms,
                                    uint32_t overall_timeout_ms)
{
    if (!sensor || num_samples_to_try < 1 || min_required_samples < 1) {
        if(sensor) sensor->last_error = DYP_ERR_INVALID_ARG;
        return DYP_A02_READ_ERROR;
    }

    uint32_t sum = 0;
    int valid_samples = 0;
    int tried_samples = 0;
    uint64_t start_time = dyp_get_time_ms();

    if (!dyp_mutex_take(sensor)) {
        sensor->last_error = DYP_ERR_MUTEX_FAIL;
        return DYP_A02_READ_ERROR;
    }

    // Clear last error for this operation
    sensor->last_error = DYP_OK;

    while (tried_samples < num_samples_to_try) {
        // Check overall timeout
        if ((dyp_get_time_ms() - start_time) > overall_timeout_ms) {
            sensor->last_error = DYP_ERR_TIMEOUT_OVERALL;
            break;
        }

        tried_samples++;
        uint16_t dist;
        dyp_err_t err = read_packet_internal(sensor, &dist, per_read_timeout_ms);

        if (err == DYP_OK) {
            // Hardware read was good (checksum ok, not 0mm)
            sum += dist;
            valid_samples++;
        } else {
            // A hardware error occurred (checksum, timeout)
            // Store the first *hardware* error we see
            if (sensor->last_error == DYP_OK) {
                sensor->last_error = err;
            }
            // This is not a fatal error, we just try again.
            // We must yield to prevent a spin-loop.
            dyp_task_yield();
        }
    }

    dyp_mutex_give(sensor);

    // Now, check if we got *enough* valid samples
    if (valid_samples >= min_required_samples) {
        sensor->last_error = DYP_OK;
        return (long)(sum / valid_samples); // Return the average
    }

    // Failure: We timed out or finished all tries
    // and didn't get the minimum number of samples.
    if (sensor->last_error == DYP_OK) {
        // If no hardware error was logged, log this one.
        sensor->last_error = DYP_ERR_NOT_ENOUGH_SAMPLES;
    }
    return DYP_A02_READ_ERROR;
}


bool dyp_a02_is_object_detected(long distance_mm, uint16_t threshold_mm) {
    if (distance_mm == DYP_A02_READ_ERROR) {
        return false;
    }
    // Check for > 0 because 0 is an invalid reading
    return (distance_mm > 0 && distance_mm <= (long)threshold_mm);
}

dyp_err_t dyp_a02_get_last_error(dyp_sensor_t *sensor) {
    if (!sensor) return DYP_ERR_INVALID_ARG;

    // This read is not mutex-protected, but dyp_err_t
    // should be an atomic read on most platforms.
    // For true safety, we can wrap it.
    if (!dyp_mutex_take(sensor)) return DYP_ERR_MUTEX_FAIL;
    dyp_err_t err = sensor->last_error;
    dyp_mutex_give(sensor);
    
    return err;
}

const char* dyp_a02_strerror(dyp_err_t err) {
    switch(err) {
        case DYP_OK: return "Success";
        case DYP_ERR_INVALID_ARG: return "Invalid argument";
        case DYP_ERR_NOT_INITIALIZED: return "Sensor not initialized";
        case DYP_ERR_MUTEX_FAIL: return "Mutex operation failed";
        case DYP_ERR_UART_CONFIG: return "UART configuration failed";
        case DYP_ERR_UART_PIN: return "UART pin setup failed";
        case DYP_ERR_UART_INSTALL: return "UART driver install failed";
        case DYP_ERR_NO_MEM: return "Out of memory";
        case DYP_ERR_NOT_SUPPORTED: return "Function not supported";
        case DYP_ERR_TIMEOUT: return "Read timeout (no bytes)";
        case DYP_ERR_CHECKSUM: return "Packet checksum mismatch";
        case DYP_ERR_INVALID_READING: return "Invalid reading (e.g., 0mm)";
        case DYP_ERR_TIMEOUT_OVERALL: return "Filtered average timed out";
        case DYP_ERR_NOT_ENOUGH_SAMPLES: return "Not enough good samples";
        default: return "Unknown error";
    }
}