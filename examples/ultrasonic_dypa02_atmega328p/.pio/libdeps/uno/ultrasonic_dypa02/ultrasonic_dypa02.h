#ifndef DYP_UART_H
#define DYP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#define DYP_UART_BAUD_DEFAULT 9600
#define DYP_UART_ERROR       (-1)
#define DYP_UART_OK           0

/* Unit conversion constants */
#define DYP_CM_TO_INCH  0.393701f
#define DYP_CM_TO_FEET  0.0328084f
#define DYP_CM_TO_METER 0.01f

/* Initialize the UART-based DYP sensor.
   - rx_pin, tx_pin: platform pin numbers (Arduino digital/Ax macros or ESP32 GPIO numbers)
   - baud: baudrate (pass 0 to use default 9600)
   Returns 0 on success, negative on error. */
int dyp_uart_init(int rx_pin, int tx_pin, int baud);

/* Deinitialize / free resources */
int dyp_uart_deinit(void);

/* Blocking read: wait up to timeout_ms for a valid frame and parse distance.
   If timeout_ms == 0 -> uses default internal timeout (100 ms).
   Returns:
     0 on success and writes distance in the specified unit to *out_distance (float)
    -1 on timeout/no-frame
    -2 on invalid args or not initialized
*/

/* Read distance in centimeters (float) */
int dyp_uart_read_distance_cm_float(float *out_cm, unsigned timeout_ms);

/* Read distance in inches (float) */
int dyp_uart_read_distance_inch_float(float *out_inch, unsigned timeout_ms);

/* Read distance in feet (float) */
int dyp_uart_read_distance_feet_float(float *out_feet, unsigned timeout_ms);

/* Read distance in meters (float) */
int dyp_uart_read_distance_meter_float(float *out_meter, unsigned timeout_ms);

/* Integer variants: returns DYP_UART_ERROR on error, otherwise integer distance (rounded) */
long dyp_uart_read_distance_cm_int(unsigned timeout_ms);
long dyp_uart_read_distance_inch_int(unsigned timeout_ms);
long dyp_uart_read_distance_feet_int(unsigned timeout_ms);
long dyp_uart_read_distance_meter_int(unsigned timeout_ms);

/* Convenience: returns true if last read call timed out (no valid frame) */
bool dyp_uart_last_timeout(void);

/* Convenience high-level: object detection against threshold (blocking read),
   returns 1 if object present (distance <= threshold), 0 if not, negative on error. */
int dyp_uart_object_detected_cm(float threshold_cm, unsigned timeout_ms);
int dyp_uart_object_detected_inch(float threshold_inch, unsigned timeout_ms);
int dyp_uart_object_detected_feet(float threshold_feet, unsigned timeout_ms);
int dyp_uart_object_detected_meter(float threshold_meter, unsigned timeout_ms);
int dyp_uart_flush_buffer(void);
    

/* ============================================================
   ADD TO HEADER FILE (ultrasonic_dypa02.h)
   ============================================================ */

// Add this constant with the other conversion constants:
#define DYP_CM_TO_MM    10.0f

// Add these function declarations:

/* Read distance in millimeters (float) */
int dyp_uart_read_distance_mm_float(float *out_mm, unsigned timeout_ms);

/* Read distance in millimeters (integer) */
long dyp_uart_read_distance_mm_int(unsigned timeout_ms);

/* Object detection with millimeter threshold */
int dyp_uart_object_detected_mm(float threshold_mm, unsigned timeout_ms);

/* Change baud (reconfigure underlying UART). Returns 0 on success. */
int dyp_uart_set_baud(int baud);

#ifdef __cplusplus
}
#endif

#endif /* DYP_UART_H */