#ifndef ULTRASONIC_DYPA02_H
#define ULTRASONIC_DYPA02_H

#ifdef __cplusplus
// No extern "C" here yet
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * DYP-A02 (or compatible) UART Ultrasonic Sensor
 * cross-platform C API (ESP-IDF & Arduino)
 *
 * MODULAR/STRUCTURED/ROBUST VERSION
 * - Uses a dyp_sensor_t "handle" to support multiple sensor instances.
 * - Each instance is thread-safe on ESP-IDF.
 * - Uses a dyp_err_t enum for clear error handling.
 * - Stores last error in the sensor handle.
 */


// --- C-Linkage block for C/C++ compatibility ---
// NO extern "C" here yet.

// Platform-specific includes for handle
#if defined(ESP_PLATFORM)
  #include "driver/uart.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/semphr.h"
#elif defined(ARDUINO) && defined(__cplusplus)
    // For Arduino C++, we need these types
    #include <Stream.h>
    #include <SoftwareSerial.h>
#endif

/**
 * @brief Error codes for all library functions.
 * MOVED OUTSIDE extern "C" to be visible to C++
 */
typedef enum {
    DYP_OK = 0,                 // Success
    DYP_ERR_INVALID_ARG,        // A null pointer or invalid argument was passed
    DYP_ERR_NOT_INITIALIZED,    // Sensor struct is not initialized
    DYP_ERR_MUTEX_FAIL,         // (ESP-IDF) Failed to create/take mutex
    DYP_ERR_UART_CONFIG,        // (ESP-IDF) Failed to configure UART
    DYP_ERR_UART_PIN,           // (ESP-IDF) Failed to set UART pins
    DYP_ERR_UART_INSTALL,       // (ESP-IDF) Failed to install UART driver
    DYP_ERR_NO_MEM,             // (Arduino) Failed to allocate SoftwareSerial
    DYP_ERR_NOT_SUPPORTED,      // Function not supported on this platform
    DYP_ERR_TIMEOUT,            // Timed out waiting for packet/bytes
    DYP_ERR_CHECKSUM,           // Packet checksum mismatch
    DYP_ERR_INVALID_READING,    // Reading was 0 or otherwise invalid
    DYP_ERR_TIMEOUT_OVERALL,    // (Filtered Read) Failed to get samples in time
    DYP_ERR_NOT_ENOUGH_SAMPLES  // (Stable Read) Got < minimum required samples
} dyp_err_t;

/**
 * @brief Main sensor handle structure.
 * MOVED OUTSIDE extern "C" to be visible to C++ and use C++ types.
 */
typedef struct {
    // Internal state
    int mode; // 0=none, 1=hw_esp, 2=stream_arduino, 3=sw_arduino
    bool is_initialized;
    dyp_err_t last_error; // <-- Last known error for this sensor

#if defined(ESP_PLATFORM)
    uart_port_t uart_num;
    SemaphoreHandle_t mutex; // Thread-safety mutex for this instance
#elif defined(ARDUINO)
    // These types are now C++ classes
    Stream *stream;
    SoftwareSerial *sw_serial_inst; 
#endif
} dyp_sensor_t;

/**
 * @brief Use this to initialize or reset your sensor struct.
 * e.g. dyp_sensor_t my_sensor = DYP_SENSOR_INITIALIZER;
 * or
 * my_sensor = DYP_SENSOR_INITIALIZER;
 * (This compound literal works for both initialization and assignment)
 */
#define DYP_SENSOR_INITIALIZER (dyp_sensor_t){0}


/**
 * @brief Error code returned by integer functions on read failure.
 */
#define DYP_A02_READ_ERROR ((long)0x80000000L)

/**
 * @brief Default baud rate for the DYP-A02 sensor.
 */
#define DYP_A02_DEFAULT_BAUD 9600

/**
 * @brief (ESP-IDF only) Default UART port to use.
 */
#define DYP_A02_DEFAULT_UART_PORT 2


// --- C-Linkage block for C/C++ compatibility ---
// ALL function prototypes go inside here
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Initialize using Software Serial (Arduino) or default HW UART (ESP-IDF).
 *
 * @param sensor Pointer to your dyp_sensor_t struct.
 * @param rx_pin MCU RX pin (connected to Sensor TX).
 * @param tx_pin MCU TX pin (connected to Sensor RX).
 * @return DYP_OK on success, or a dyp_err_t code on failure.
 */
dyp_err_t dyp_a02_init(dyp_sensor_t *sensor, int rx_pin, int tx_pin);

/**
 * @brief Initialize using an existing Stream (Arduino only).
 *
 * @param sensor Pointer to your dyp_sensor_t struct.
 * @param stream_ptr A pointer to the Stream object (e.g., &Serial2).
 * @return DYP_OK on success, or a dyp_err_t code on failure.
 */
dyp_err_t dyp_a02_init_stream(dyp_sensor_t *sensor, void *stream_ptr);

/**
 * @brief Initialize using a specific HW UART port (ESP-IDF only).
 *
 * @param sensor Pointer to your dyp_sensor_t struct.
 * @param uart_port_num The UART port to use (e.g., UART_NUM_1, UART_NUM_2).
 * @param rx_pin MCU RX pin (connected to Sensor TX).
 * @param tx_pin MCU TX pin (connected to Sensor RX).
 * @return DYP_OK on success, or a dyp_err_t code on failure.
 */
dyp_err_t dyp_a02_init_esp_hw(dyp_sensor_t *sensor, int uart_port_num, int rx_pin, int tx_pin);

/**
 * @brief Deinitialize/cleanup the sensor interface.
 *
 * @param sensor Pointer to your dyp_sensor_t struct.
 */
void dyp_a02_deinit(dyp_sensor_t *sensor);

/**
 * @brief Read a single, raw packet from the sensor.
 * This function will set the sensor's last_error field on failure.
 *
 * @param sensor Pointer to your dyp_sensor_t struct.
 * @param distance_mm_out Pointer to store the read distance in millimeters.
 * @param timeout_ms Timeout for waiting for a complete packet.
 * @return true on success, false on error (check with dyp_a02_get_last_error).
 */
bool dyp_a02_read_packet(dyp_sensor_t *sensor, uint16_t *distance_mm_out, uint32_t timeout_ms);

/**
 * @brief Read a single distance measurement.
 * This function will set the sensor's last_error field on failure.
 *
 * @param sensor Pointer to your dyp_sensor_t struct.
 * @param timeout_ms Timeout for waiting for a complete packet.
 * @return Distance in mm, or DYP_A02_READ_ERROR (check with dyp_a02_get_last_error).
 */
long dyp_a02_read_distance_int(dyp_sensor_t *sensor, uint32_t timeout_ms);

/**
 * @brief Read and average multiple samples.
 * This function will set the sensor's last_error field on failure.
 *
 * @param sensor Pointer to your dyp_sensor_t struct.
 * @param num_samples Number of valid samples to average.
 * @param per_read_timeout_ms Timeout for *each* packet read attempt.
 * @return Average distance in mm, or DYP_A02_READ_ERROR (check with dyp_a02_get_last_error).
 */
long dyp_a02_read_average_int(dyp_sensor_t *sensor, int num_samples, uint32_t per_read_timeout_ms);

/**
 * @brief [DEPRECATED] Read and average multiple samples with filtering.
 * This function is deprecated as it treats "out of range" as an error.
 * Use dyp_a02_get_stable_distance_mm instead.
 *
 * @param sensor Pointer to your dyp_sensor_t struct.
 * @param num_samples Number of *valid* samples to collect.
 * @param min_mm Minimum accepted distance (e.g., 1).
 * @param max_mm Maximum accepted distance (e.g., 1000).
 * @param per_read_timeout_ms Timeout for a single packet read attempt.
 * @param overall_timeout_ms Total time to wait before giving up.
 * @return Average distance in mm, or DYP_A02_READ_ERROR (check with dyp_a02_get_last_error).
 */
long dyp_a02_read_average_filtered_int(dyp_sensor_t *sensor,
                                       int num_samples,
                                       uint16_t min_mm,
                                       uint16_t max_mm,
                                       uint32_t per_read_timeout_ms,
                                       uint32_t overall_timeout_ms);
                                       
/**
 * @brief NEW ROBUST FUNCTION: Gets a stable average distance.
 * This function is designed for production. It tries to get multiple
 * hardware-valid samples (ignoring checksum/timeout errors) and averages them.
 * It does NOT filter by distance, so "no cup" (e.g., 2000mm) is a
 * successful read, not an error.
 *
 * @param sensor Pointer to your dyp_sensor_t struct.
 * @param num_samples_to_try How many read attempts (e.g., 20).
 * @param min_required_samples Minimum good samples needed for an average (e.g., 5).
 * @param per_read_timeout_ms Timeout for a single packet read attempt.
 * @param overall_timeout_ms Total time to wait before giving up.
 * @return Average distance in mm, or DYP_A02_READ_ERROR on hardware failure.
 */
long dyp_a02_get_stable_distance_mm(dyp_sensor_t *sensor,
                                    int num_samples_to_try,
                                    int min_required_samples,
                                    uint32_t per_read_timeout_ms,
                                    uint32_t overall_timeout_ms);

/**
 * @brief Helper function to check for object detection.
 *
 * @param distance_mm The distance reading (e.g., from one of the read functions).
 * @param threshold_mm The threshold for detection (e.g., 80).
 * @return true if (distance_mm > 0 && distance_mm <= threshold_mm),
 * false otherwise (including if distance_mm == DYP_A02_READ_ERROR).
 */
bool dyp_a02_is_object_detected(long distance_mm, uint16_t threshold_mm);

/**
 * @brief NEW: Get the last error recorded in the sensor handle.
 *
 * @param sensor Pointer to your dyp_sensor_t struct.
 * @return The last dyp_err_t code.
 */
dyp_err_t dyp_a02_get_last_error(dyp_sensor_t *sensor);

/**
 * @brief NEW: Convert a dyp_err_t code into a human-readable string.
 *
 * @param err The error code.
 * @return A constant string describing the error.
 */
const char* dyp_a02_strerror(dyp_err_t err);


#ifdef __cplusplus
} // End extern "C"
#endif

#endif /* DYP_A02_UART_H */