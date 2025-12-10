# ESP32 ENC28J60 Ethernet Communication Example

This project demonstrates reliable and high-performance UDP communication between an ESP32 and a PC using an ENC28J60 Ethernet module.

## Features

-   **High Performance**: Tuned for optimal throughput with 4 MHz SPI clock and full MTU buffering (1514 bytes).
-   **Reliable Connection**: Uses a custom PHY driver to bypass strict Chip ID checks, supporting a wider range of ENC28J60 modules (including clones).
-   **Direct PC Connection**: Configured with a Link-Local (APIPA) Static IP to allow direct cable connection to a Windows PC without requiring a router or DHCP server.
-   **UDP Echo Server**: Echos back any received data for latency and integrity testing.
-   **Python Client**: Includes a helper script for easy testing from Windows.

## Hardware Connection (VSPI)

Connect the ENC28J60 module to the ESP32 as follows:

| ENC28J60 Pin | ESP32 GPIO | Description |
| :--- | :--- | :--- |
| **MISO** | 19 | SPI Data In |
| **MOSI** | 23 | SPI Data Out |
| **SCK** | 18 | SPI Clock |
| **CS** | 5 | Chip Select |
| **INT** | N/C | Interrupt (Not Used/Polled) |
| **VCC** | 3.3V | Power |
| **GND** | GND | Ground |

> **Note**: The SPI Clock is set to **4 MHz** in `src/main.c`. If you experience instability (e.g., "Ethernet Link Down" or logs stopping), try lowering `SPI_CLOCK_MHZ` back to `1` in `src/main.c`.

## Network Configuration

The ESP32 is configured with a **Static IP Address**:
-   **IP**: `169.254.1.200`
-   **Mask**: `255.255.0.0`
-   **Gateway**: `169.254.1.1`

This IP range is automatically used by Windows when no DHCP server is found (Auto IP / APIPA). This ensures you can plug the Ethernet cable directly from the ESP32 to your PC and communicate immediately.

## How to Run

1.  **Build and Flash**:
    Use PlatformIO to build and upload the firmware to your ESP32.
    ```bash
    pio run -t upload
    pio device monitor
    ```

2.  **Verify ESP32 Status**:
    Watch the serial monitor. You should see:
    -   `Setting MAC address to ...`
    -   `Ethernet Started`
    -   `Ethernet Link Up`
    -   `Ethernet Got IP Address` -> `ETHIP:169.254.1.200`
    -   `UDP Server Task Started`

3.  **Run Python Client**:
    Open a terminal in the project directory and run:
    ```bash
    python client.py
    ```
    -   The script will detect your PC's IP.
    -   It defaults to the ESP32's IP (`169.254.1.200`).
    -   It will send a "Hello" packet and measure the round-trip time.

## Performance Tuning Details

-   **SPI Clock**: Increased to **4 MHz** (up from 1 MHz) for faster transfers.
-   **RX Buffer**: Increased to **1514 bytes** to handle full Ethernet frames without truncation.
-   **Logging**: Data logging in `udp_server_task` is optimized to avoid printing large payloads to the serial console, which slows down processing. Only packet length and source are logged for large packets.
-   **Custom Driver**: `src/my_enc28j60_phy.c` implements a permissive initialization routine that logs warnings instead of aborting if the PHY ID doesn't strictly match Microchip defaults.

## Troubleshooting

-   **No Response**: Check if your PC has an IP in the `169.254.x.x` range (`ipconfig`). If it's using WiFi (`192.168.x.x`), you may need to disable WiFi briefly or manually set your Ethernet adapter IP to static `169.254.1.10`.
-   **Wrong Chip ID**: The logs might show `PHY ID Read: ID1=0x0000...`. This is fine; the custom driver allows it to proceed. If it fails completely, check your wiring.
