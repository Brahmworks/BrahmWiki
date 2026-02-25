# RobustUART

A cross-platform C++ library for robust UART communication using a packetized protocol.

## Features

- **Packet Protocol**: `SOH[LEN][CMD][PAYLOAD][CHKSUM]EOT` with ACK/NAK responses
- **Cross-Platform**: ESP32 (ESP-IDF/Arduino), ATmega328P, ATmega2560, and any PlatformIO-supported MCU
- **Error Handling**: XOR checksum, 200ms timeout, 3 retries
- **Heartbeat**: Automatic heartbeat command (CMD=0x00)
- **Non-blocking**: Callbacks for rx_frame, tx_complete, error
- **Zero Dependencies**: <2KB flash footprint
- **ISR-Safe**: Ring buffer implementation

## Protocol

```
Frame Format:
[SOH][LEN][CMD][PAYLOAD...][CHKSUM][EOT]

- SOH (0x01): Start of Header
- LEN: Payload length (0-256)
- CMD: Command byte
- PAYLOAD: Data bytes
- CHKSUM: XOR checksum of LEN + CMD + PAYLOAD
- EOT (0x04): End of Transmission

Responses:
- ACK (0x06): Acknowledged
- NAK (0x15): Not Acknowledged (retry)
```

## Installation

### PlatformIO

```ini
# platformio.ini
lib_deps = 
    https://github.com/Brahmworks/BrahmWiki.git#lib/robust_uart
```

Or use as a local library by copying to `lib/` folder.

## Usage

### Host (ESP32)

```cpp
#include <RobustUART.h>
#include <RobustUART_host.h>

RobustHost<HardwareSerial> host(&Serial2);

void setup() {
    // Initialize: baud, rx_pin, tx_pin
    host.begin(115200, 16, 17);
}

void loop() {
    // Send ping with 500ms timeout
    if (host.ping(500)) {
        Serial.println("Ping OK!");
    }
    
    // Send command with response
    uint8_t response[32];
    size_t len = sizeof(response);
    if (host.get_status(response, len, 500)) {
        // Process response
    }
}
```

### Client (ATmega)

```cpp
#include <RobustUART.h>
#include <RobustUART_client.h>

RobustClient<HardwareSerial> client(&Serial);

void setup() {
    client.begin(115200);
}

void loop() {
    client.process();
}
```

## API

### RobustUART (Base Class)

| Method | Description |
|--------|-------------|
| `begin(baud, rx_pin, tx_pin)` | Initialize UART |
| `send_frame(cmd, data, len)` | Send packetized frame |
| `send_cmd(cmd)` | Send command without payload |
| `process_incoming()` | Process received bytes |
| `tick()` | Main state machine tick |
| `on_rx_frame(callback)` | Set RX callback |
| `on_error(callback)` | Set error callback |
| `get_stats()` | Get communication statistics |

### RobustHost

| Method | Description |
|--------|-------------|
| `ping(timeout_ms)` | Send ping and wait for response |
| `read_sensor(id, data, len, timeout)` | Read sensor from client |
| `write_actuator(id, value, len, timeout)` | Write to actuator |
| `get_status(data, len, timeout)` | Get device status |
| `get_firmware_version(data, len, timeout)` | Get firmware version |

### RobustClient

| Method | Description |
|--------|-------------|
| `begin(baud)` | Start listening for commands |
| `process()` | Process incoming commands |
| `register_handler(cmd, callback)` | Register custom command handler |
| `set_response(data, len)` | Set response data |
| `send_response()` | Send response to host |

## Configuration

Default configuration (can be modified via `config` member):

```cpp
robust_uart_config_t config = {
    .baud_rate = 115200,
    .rx_pin = -1,
    .tx_pin = -1,
    .uart_num = 0,
    .rx_buffer_size = 512,
    .tx_buffer_size = 512,
    .frame_timeout_ms = 200,
    .max_retries = 3,
    .enable_heartbeat = true,
    .heartbeat_interval_ms = 5000
};
```

## Debugging

Enable debug output:

```cpp
#define DEBUG_UART_LOG
#include <RobustUART.h>
```

## Performance

- Protocol overhead: ~6 bytes per frame
- Typical latency: <10ms at 115200 baud
- Flash footprint: <2KB

## License

MIT License
