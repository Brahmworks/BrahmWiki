#ifndef MAX6675_H
#define MAX6675_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * MAX6675 cross-platform C API
 *
 * - Two-file library: max6675.h and max6675.cpp
 * - Works on Arduino (UNO/MEGA) and ESP-IDF (ESP32)
 *
 * Error/edge cases:
 * - On hardware/communication error or thermocouple open/fault:
 *     - float functions return NAN
 *     - integer functions return MAX6675_ERROR
 */

#define MAX6675_ERROR ((long)0x80000000L)

/* Initialize using hardware SPI.
 * cs_pin: chip-select (GPIO)
 * clock_hz: desired SPI clock in Hz (Arduino: passed to SPISettings; ESP-IDF: used in device config)
 * Returns 0 on success, negative on error.
 */
int max6675_init_hw(int cs_pin, uint32_t clock_hz);

/* Initialize using software (bit-banged) SPI.
 * cs_pin: chip-select (GPIO)
 * sck_pin: clock pin (GPIO)
 * miso_pin: data out (SO) pin from MAX6675 to MCU (GPIO)
 * Returns 0 on success, negative on error.
 */
int max6675_init_sw(int cs_pin, int sck_pin, int miso_pin);

/* Deinitialize/cleanup; safe to call repeatedly. */
void max6675_deinit(void);

/* Low-level read: read full 16-bit raw word from MAX6675.
 * out_raw: pointer to receive the 16-bit value.
 * Returns 0 on success, negative on error.
 */
int max6675_read_raw(uint16_t *out_raw);

/* Higher level temperature reads.
 * decimals: 0..2
 * - For decimals == 0: integer degrees (e.g. 25)
 * - For decimals == 1: scaled by 10 (e.g. 252 means 25.2)
 * - For decimals == 2: scaled by 100 (e.g. 2525 means 25.25)
 *
 * Float returns:
 * - returns temperature in °C/°F on success
 * - returns NAN on error (thermocouple open or comms error)
 */
float max6675_read_celsius_float(unsigned decimals);
float max6675_read_fahrenheit_float(unsigned decimals);

/* Integer returns (long):
 * - returns MAX6675_ERROR on error
 * - otherwise returns integer (possibly scaled as above)
 */
long max6675_read_celsius_int(unsigned decimals);
long max6675_read_fahrenheit_int(unsigned decimals);

/* True if thermocouple is open/faulted (based on MAX6675 D2 bit) */
bool max6675_is_open(void);

#ifdef __cplusplus
}
#endif

#endif /* MAX6675_H */
