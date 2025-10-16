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
 * CAUTION / IMPORTANT (copy into your project README):
 * - On ESP-IDF avoid SPI_DEVICE_HALFDUPLEX with DMA + MOSI+MISO transactions.
 * - By default this library registers the device as full-duplex (devcfg.flags = 0)
 *   and uses polling small-transfer path for 1-3 byte transfers. If you see zero
 *   reads with your toolchain set MAX31865_ESP_SAFE_SPI to 1 (in max31865.cpp)
 *   to force the safe heap-buffer path for all transfers.
 *
 * See max31865.cpp for additional guidance about wiring, filter timing and
 * conversion delays.
 */

/* Error sentinel for integer reads */
#define MAX31865_ERROR ((long)0x80000000L)

/* Supported wire modes */
#define MAX31865_WIRES_2 2
#define MAX31865_WIRES_3 3
#define MAX31865_WIRES_4 4

/* Function prototypes */

/* Hardware SPI init:
 * cs_pin: chip-select GPIO
 * clock_hz: desired SPI clock in Hz (0 => use safe default 1MHz)
 * Returns 0 on success, negative on error.
 *
 * On ESP-IDF the library will attempt to initialize HSPI_HOST with common default
 * pins if spi_bus isn't already initialized. Prefer to initialize the bus in your
 * app (see examples) and then call max31865_init_hw() to only add the device.
 */
int max31865_init_hw(int cs_pin, uint32_t clock_hz);

/* Software SPI init (bit-banged).
 * cs_pin: CS GPIO
 * sck_pin: clock GPIO
 * mosi_pin: MOSI GPIO (master out - internal to MAX31865)
 * miso_pin: MISO GPIO
 * Returns 0 on success.
 */
int max31865_init_sw(int cs_pin, int sck_pin, int mosi_pin, int miso_pin);

/* Deinitialize and cleanup. Safe to call repeatedly. */
void max31865_deinit(void);

/* Configure RTD sensor type and reference resistor:
 * rtd_nominal_ohms: typical R0 for RTD (100.0 for PT100, 1000.0 for PT1000)
 * ref_resistor_ohms: resistor value used on board as R_ref (e.g., 430.0 or 4300.0)
 */
void max31865_set_rtd_nominal(float rtd_nominal_ohms, float ref_resistor_ohms);

/* Set 2/3/4-wire mode (use constants above). Returns 0 on success. */
int max31865_set_wire_mode(int wires);

/* Set filter to 50Hz (true) or 60Hz (false). Returns 0 on success. */
int max31865_set_filter_50hz(bool enable);

/* Enable/disable VBIAS (power for the RTD bridge). */
int max31865_enable_bias(bool enable);

/* Start a one-shot conversion. The conversion completes after one-shot delay.
 * Use read_* after waiting recommended conversion delay (see comments).
 * Returns 0 on success.
 */
int max31865_start_one_shot(void);

/* Clear fault status (sets fault clear bit). */
int max31865_clear_faults(void);

/* Read raw RTD register (15-bit value raw). out_raw receives raw 16-bit value
 * where only top 15 bits are used; returns 0 on success.
 */
int max31865_read_raw(uint16_t *out_raw);

/* Debug helper: read two raw register bytes (msb, lsb). */
int max31865_read_raw_bytes(uint8_t out[2]);

/* Read RTD resistance in ohms (float). Returns NAN on error/fault. */
float max31865_read_resistance_float(void);

/* Read RTD resistance as scaled integer:
 * decimals 0..2: decimals==0 -> integer ohms (rounded)
 * decimals==1 -> ohms * 10 (e.g. 25.2 => 252)
 * decimals==2 -> ohms * 100 (e.g. 25.25 => 2525)
 * Returns MAX31865_ERROR on fault.
 */
long max31865_read_resistance_int(unsigned decimals);

/* Temperature reads (Celsius) */
float max31865_read_temperature_c_float(unsigned decimals);
long  max31865_read_temperature_c_int(unsigned decimals);

/* Temperature reads (Fahrenheit) */
float max31865_read_temperature_f_float(unsigned decimals);
long  max31865_read_temperature_f_int(unsigned decimals);

/* Read the 8-bit fault status register (0x07). Returns negative on comms error.
 * Use last_fault() to get cached fault from last read if needed.
 */
int max31865_read_fault_status(uint8_t *out_fault);

/* Return the last observed fault status cached by the library (0 if none). */
uint8_t max31865_last_fault(void);

/* Check if last read indicates a fault (or read returns errors). */
bool max31865_has_fault(void);

/* Set/get debug verbosity (0 = quiet, 1 = log debug via Serial or ESP_LOGI).
 * Only useful for debugging and prints low-volume messages.
 */
void max31865_set_debug(int verbosity);

#ifdef __cplusplus
}
#endif

#endif /* MAX31865_H */
