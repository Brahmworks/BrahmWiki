#ifndef IOT_PROTOCOL_H
#define IOT_PROTOCOL_H

#include "uart_standard.h"

// Initialize the IoT protocol component
void iot_protocol_init(uint8_t uart_port);

// Perform the device handshake
void iot_protocol_handshake(void);

// Send a health check message
void iot_protocol_send_health_check(void);

// Process incoming data from the server
void iot_protocol_process_incoming(void);

#endif // IOT_PROTOCOL_H
