# ESP32 with DHT11/DHT22 Temperature and Humidity Sensor

This tutorial shows how to use the DHT11 and DHT22 temperature and humidity sensors with the ESP32 using Arduino IDE. We’ll go through a quick introduction to these sensors, pinout, wiring diagram, and finally the Arduino sketch.

## DHT11 and DHT22 Temperature and Humidity Sensors

The DHT11 and DHT22 sensors are used to measure temperature and relative humidity. These are very popular among makers and electronics hobbyists.

These sensors contain a chip that does analog to digital conversion and spit out a digital signal with the temperature and humidity. This makes them very easy to use with any microcontroller.

### DHT11 vs DHT22

The DHT11 and DHT22 are very similar, but differ in their specifications. The following table compares some of the most important specifications of the DHT11 and DHT22 temperature and humidity sensors.

| Specification | DHT11 | DHT22 |
|---|---|---|
| **Temperature range** | 0 to 50 ºC +/-2 ºC | -40 to 80 ºC +/-0.5ºC |
| **Humidity range** | 20 to 90% +/-5% | 0 to 100% +/-2% |
| **Resolution** | Humidity: 1%, Temperature: 1ºC | Humidity: 0.1%, Temperature: 0.1ºC |
| **Operating voltage** | 3 – 5.5 V DC | 3 – 6 V DC |
| **Current supply** | 0.5 – 2.5 mA | 1 – 1.5 mA |
| **Sampling period** | 1 second | 2 seconds |

The DHT22 sensor has a better resolution and a wider temperature and humidity measurement range. However, it is a bit more expensive, and you can only request readings with 2 seconds interval.

The DHT11 has a smaller range and it’s less accurate. However, you can request sensor readings every second. It’s also a bit cheaper.

## DHT Pinout

DHT sensors have four pins. When the sensor is facing you, pin numbering starts at 1 from left to right.

| DHT pin | Connect to |
|---|---|
| **1** | 3.3V |
| **2** | Any digital GPIO; also connect a 10k Ohm pull-up resistor |
| **3** | Don’t connect |
| **4** | GND |

## Parts Required

*   ESP32
*   DHT11 or DHT22 temperature and humidity sensor
*   10k Ohm resistor
*   Breadboard
*   Jumper wires

## Schematic Diagram

Wire the DHT22 or DHT11 sensor to the ESP32 development board as shown in the following schematic diagram. In this example, we’re connecting the DHT data pin to GPIO 4.

## Installing Libraries

To read from the DHT sensor, we’ll use the [DHT library from Adafruit](https://github.com/adafruit/DHT-sensor-library). To use this library you also need to install the [Adafruit Unified Sensor library](https://github.com/adafruit/Adafruit_Sensor).

1.  Open your Arduino IDE and go to **Sketch** > **Include Library** > **Manage Libraries**.
2.  Search for “**DHT**” and install the DHT library from Adafruit.
3.  Search for “**Adafruit Unified Sensor**” and install it.
4.  After installing the libraries, restart your Arduino IDE.

## ESP32 Reading Temperature and Humidity Sketch

```cpp
// Example testing sketch for various DHT humidity/temperature sensors written by ladyada
// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"

#define DHTPIN 4     // Digital pin connected to the DHT sensor

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));

  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));
}
```

## Troubleshooting

If you get an error message “Failed to read from DHT sensor!” or the DHT readings return “Nan”, try the following:

*   **Wiring:** Double-check the wiring and pin assignment.
*   **Power:** The DHT sensor has an operating range of 3V to 5.5V (DHT11) or 3V to 6V (DHT22). If you’re powering the sensor from the ESP32 3.3V pin, in some cases powering the DHT with 5V solves the problem.
*   **Sensor type:** Double-check that you’ve uncommented the right sensor in your code.
*   **Sampling rate:** The DHT sensor is very slow. Increasing the time between readings can sometimes solve the problem.
