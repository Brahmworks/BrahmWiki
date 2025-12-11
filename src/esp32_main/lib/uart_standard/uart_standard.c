#include "uart_standard.h"

#if defined(ESP32)
// ESP32 Implementation
#include "driver/uart.h"
#include "string.h"

void uart_init(uint8_t uart_num, uint32_t baud_rate) {
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(uart_num, &uart_config);
    uart_driver_install(uart_num, 256, 0, 0, NULL, 0);
}

void uart_write(uint8_t uart_num, const char *str) {
    uart_write_bytes(uart_num, str, strlen(str));
}

int uart_read(uint8_t uart_num, uint8_t *buffer, uint32_t len) {
    return uart_read_bytes(uart_num, buffer, len, 100 / portTICK_PERIOD_MS);
}

#elif defined(__AVR__)
// AVR Implementation
#include <avr/io.h>

void uart_init(uint8_t uart_num, uint32_t baud_rate) {
    uint16_t ubrr = (F_CPU / 16 / baud_rate) - 1;
    switch (uart_num) {
        case 0:
            UBRR0H = (unsigned char)(ubrr >> 8);
            UBRR0L = (unsigned char)ubrr;
            UCSR0B = (1 << RXEN0) | (1 << TXEN0);
            UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
            break;
#if defined(__AVR_ATmega2560__)
        case 1:
            UBRR1H = (unsigned char)(ubrr >> 8);
            UBRR1L = (unsigned char)ubrr;
            UCSR1B = (1 << RXEN1) | (1 << TXEN1);
            UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
            break;
        case 2:
            UBRR2H = (unsigned char)(ubrr >> 8);
            UBRR2L = (unsigned char)ubrr;
            UCSR2B = (1 << RXEN2) | (1 << TXEN2);
            UCSR2C = (1 << UCSZ21) | (1 << UCSZ20);
            break;
        case 3:
            UBRR3H = (unsigned char)(ubrr >> 8);
            UBRR3L = (unsigned char)ubrr;
            UCSR3B = (1 << RXEN3) | (1 << TXEN3);
            UCSR3C = (1 << UCSZ31) | (1 << UCSZ30);
            break;
#endif
    }
}

void uart_write(uint8_t uart_num, const char *str) {
    while (*str) {
        switch (uart_num) {
            case 0:
                while (!(UCSR0A & (1 << UDRE0)));
                UDR0 = *str;
                break;
#if defined(__AVR_ATmega2560__)
            case 1:
                while (!(UCSR1A & (1 << UDRE1)));
                UDR1 = *str;
                break;
            case 2:
                while (!(UCSR2A & (1 << UDRE2)));
                UDR2 = *str;
                break;
            case 3:
                while (!(UCSR3A & (1 << UDRE3)));
                UDR3 = *str;
                break;
#endif
        }
        str++;
    }
}

int uart_read(uint8_t uart_num, uint8_t *buffer, uint32_t len) {
    // Placeholder for AVR UART read
    return -1;
}

#else
// Unsupported architecture
#warning "UART_STANDARD library does not support this architecture."

void uart_init(uint8_t uart_num, uint32_t baud_rate) {}
void uart_write(uint8_t uart_num, const char *str) {}
int uart_read(uint8_t uart_num, uint8_t *buffer, uint32_t len) { return -1; }

#endif
