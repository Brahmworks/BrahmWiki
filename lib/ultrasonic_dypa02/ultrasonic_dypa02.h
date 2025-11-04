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
     0 on success and writes distance in centimeters to *out_cm (float)
    -1 on timeout/no-frame
    -2 on invalid args or not initialized
*/
int dyp_uart_read_distance_cm_float(float *out_cm, unsigned timeout_ms);

/* Integer variant: returns DYP_UART_ERROR on error, otherwise integer cm (rounded) */
long dyp_uart_read_distance_cm_int(unsigned timeout_ms);

/* Convenience: returns true if last read call timed out (no valid frame) */
bool dyp_uart_last_timeout(void);

/* Convenience high-level: object detection against threshold (blocking read),
   returns 1 if object present (distance <= threshold_cm), 0 if not, negative on error. */
int dyp_uart_object_detected(float threshold_cm, unsigned timeout_ms);

/* Change baud (reconfigure underlying UART). Returns 0 on success. */
int dyp_uart_set_baud(int baud);

#ifdef __cplusplus
}
#endif

#endif /* DYP_UART_H */
