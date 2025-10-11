# ESP32

The ESP32 is a powerful, feature-rich microcontroller with integrated Wi-Fi and Bluetooth. It is an excellent choice for IoT projects.

## Pinout



## Pin Reference

### Digital Pins

| Pin | Function |
| --- | -------- |
| GPIO0 | Digital I/O, Bootstrapping |
| GPIO1 | Digital I/O, UART0 TX |
| GPIO2 | Digital I/O, Bootstrapping |
| GPIO3 | Digital I/O, UART0 RX |
| GPIO4 | Digital I/O |
| GPIO5 | Digital I/O, VSPI SS |
| ... | ... |

### Analog Pins (ADC)

| Pin | ADC Channel |
| --- | ----------- |
| GPIO36 | ADC1_CH0 |
| GPIO37 | ADC1_CH1 |
| GPIO38 | ADC1_CH2 |
| GPIO39 | ADC1_CH3 |
| ... | ... |

### PWM Pins

Any GPIO can be configured to be a PWM pin.

### Bootstrapping Pins

| Pin | State for Flashing | State for Normal Boot |
| --- | ------------------ | --------------------- |
| GPIO0 | LOW | HIGH |
| GPIO2 | HIGH | LOW |
