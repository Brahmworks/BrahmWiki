#ifndef MAX31865_H
#define MAX31865_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * MAX31865 cross-platform C API (Arduino + ESP-IDF)
 *
 * Note: This variant uses a single global device instance internally
 * (s_spi_dev). Use the `_hw` / `_sw` init functions to select hardware
 * or software SPI; the other APIs operate on the global device.
 */

/* Error sentinel for integer reads */
#define MAX31865_ERROR ((long)0x80000000L)

/* Supported wire modes */
#define MAX31865_WIRES_2 2
#define MAX31865_WIRES_3 3
#define MAX31865_WIRES_4 4

/* Public API (single-device style) */

/* Initialize using hardware SPI and the given CS pin.
 * cs_pin: chip-select GPIO
 * clock_hz: desired SPI clock in Hz (0 => use safe default 1MHz)
 * Returns 0 on success, negative on error.
 */
int max31865_init_hw(int cs_pin, uint32_t clock_hz);

/* Initialize using software (bit-banged) SPI.
 * Returns 0 on success.
 */
int max31865_init_sw(int cs_pin, int sck_pin, int mosi_pin, int miso_pin);

/* Deinitialize/cleanup. Safe to call repeatedly. */
void max31865_deinit(void);

/* Configure RTD sensor type and reference resistor */
void max31865_set_rtd_nominal(float rtd_nominal_ohms, float ref_resistor_ohms);

/* Set 2/3/4-wire mode (use constants above). Returns 0 on success. */
int max31865_set_wire_mode(int wires);

/* Set 50Hz filter (true) or 60Hz (false). Returns 0 on success. */
int max31865_set_filter_50hz(bool enable);

/* Enable/disable bias (VBIAS) for one-shot style */
int max31865_enable_bias(bool enable);

/* Start a one-shot conversion. Returns 0 on success. */
int max31865_start_one_shot(void);

/* Clear fault status (set fault clear bit). */
int max31865_clear_faults(void);

/* Read raw RTD register (15-bit value raw). Returns 0 on success. */
int max31865_read_raw(uint16_t *out_raw);

/* Read RTD resistance in ohms (float). Returns NAN on error/fault. */
float max31865_read_resistance_float(void);

/* Temperature reads (Celsius) */
float max31865_read_temperature_c_float(unsigned decimals);
long  max31865_read_temperature_c_int(unsigned decimals);

/* Read fault status register (0x07). Returns negative on comms error. */
int max31865_read_fault_status(uint8_t *out_fault);

/* Return last cached fault status (0 if none). */
uint8_t max31865_last_fault(void);
bool    max31865_has_fault(void);

/* Debug verbosity (0 = quiet). */
void max31865_set_debug(int verbosity);


/* --- Functions restored from old library --- */

/* Debug helper: read two raw register bytes (msb, lsb). */
int max31865_read_raw_bytes(uint8_t out[2]);

/* Read RTD resistance as scaled integer:
 * decimals 0..2: decimals==0 -> integer ohms (rounded)
 * decimals==1 -> ohms * 10 (e.g. 25.2 => 252)
 * decimals==2 -> ohms * 100 (e.g. 25.25 => 2525)
 * Returns MAX31865_ERROR on fault.
 */
long max31865_read_resistance_int(unsigned decimals);

/* Temperature reads (Fahrenheit) */
float max31865_read_temperature_f_float(unsigned decimals);
long  max31865_read_temperature_f_int(unsigned decimals);


/* --- Functions added for robustness / continuous mode --- */
/* Put the internal global device into continuous conversion mode (bias ON).
 * enable = true turns on continuous conversion + bias, false turns them off.
 * Returns 0 on success, negative on error.
 */
int max31865_set_continuous_mode(bool enable);

/* Safe read wrapper:
 * - Returns a spike-filtered temperature in °C (float).
 * - On transient failure returns last known-good temperature (or NAN if none).
 * - Matches the implementation in max31865.cpp (no device handle required).
 */
float max31865_read_temperature_safe(void);

/* Return last-good temperature cached by the safe-read wrapper (NAN if none). */
float max31865_get_last_good(void);

#ifdef __cplusplus
}
#endif

#endif /* MAX31865_H */