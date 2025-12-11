#ifndef UART_STANDARD_H
#define UART_STANDARD_H

#include <stdint.h>

// UART Port Definitions
#define UART_0_PROG 0
#define UART_1_COMM 1
#define UART_2_DEBUG 2
#define UART_3_COMM2 3

void uart_init(uint8_t uart_num, uint32_t baud_rate);
void uart_write(uint8_t uart_num, const char *str);
int uart_read(uint8_t uart_num, uint8_t *buffer, uint32_t len);

#endif // UART_STANDARD_H
