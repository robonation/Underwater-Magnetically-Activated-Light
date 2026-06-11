# Underwater Magnetically Activated Light

ESP32-C3 firmware for the Underwater Magnetically Activated Light test unit used to simulate competition pipeline indicator behavior. The device creates its own Wi-Fi access point, serves a configuration page, reads a TLV493D 3-axis magnetometer, drives a WS2812 RGB LED ring, and monitors a battery-area 10K NTC thermistor for over-temperature protection.

## Current behavior

- Starts in **solid red** when armed.
- Reads magnetic field magnitude from the TLV493D magnetometer.
- If magnetic magnitude is above the configured threshold for at least **250 ms**, the light changes to **solid green**.
- If the magnet is removed before the configured hold time, the light returns to **solid red**.
- If the magnet remains above threshold for the configured hold time, the LED ring starts a red/green/blue flash sequence.
- The flash sequence continues for the configured total duration even if the magnet is removed.
- After the flash duration ends, the device returns to solid red and re-arms.
- If the battery-area thermistor reaches **45 °C**, the LED turns off and magnetic triggering is disabled.
- The battery temperature fault clears after the measured temperature drops below **43 °C**.
- If the battery thermistor reading fails, the trigger is disabled and the LED ring blinks red at **500 ms on / 500 ms off**.
- In Field Mode, if the TLV493D magnetometer read fails, the LED ring blinks red at **250 ms on / 250 ms off**.

## Hardware

Tested target hardware:

- ESP32-C3 Mini / Super Mini
- Adafruit TLV493D 3D magnetometer
- WS2812 RGB LED ring, 16 LEDs
- DFRobot 18650 power bank or other 5 V supply for the LED ring
- 10K NTC thermistor, Beta 3950
- 10K fixed resistor for thermistor divider
- 470 ohm resistor in series with WS2812 data input
- 1000 uF capacitor across LED ring power and ground

## Wiring

### TLV493D magnetometer

| TLV493D pin | ESP32-C3 connection |
| --- | --- |
| SDA | GPIO6 |
| SCL | GPIO7 |
| VIN | 3V3 |
| GND | GND |

### WS2812 LED ring

| LED ring pin | ESP32-C3 / power connection |
| --- | --- |
| DIN | GPIO10 through 470 ohm resistor |
| 5V | 5 V from power bank |
| GND | Common GND with ESP32-C3 |

1000uF capacitor across the LED ring 5 V and GND terminals.

### Battery-area thermistor

Voltage divider wiring:

3V3 -> 10K fixed resistor -> ADC0 / GPIO0 / A0 -> 10K NTC thermistor -> GND

The firmware uses `analogReadMilliVolts()` and a Beta 3950 thermistor calculation. Thermistor sampling is non-blocking during normal operation.

## Wi-Fi configuration portal

On boot, the ESP32 creates this access point:

| Setting | Value |
| --- | --- |
| SSID | `MagSensorLight` |
| Password | `configure123` |
| Configuration URL | `http://192.168.4.1` |

Settings are saved in ESP32 non-volatile preferences and survive power cycling.

## Configurable settings

| Setting | Default | Range / notes |
| --- | ---: | --- |
| Magnetic magnitude threshold | 1.5 mT | 0.05 to 200 mT |
| Magnet hold time | 2000 ms | Minimum 250 ms, maximum 60000 ms |
| Total flash duration | 120000 ms | 1000 ms to 3600000 ms |
| Brightness | 204 | 1 to 255 |
| Flash sequence mode | 0 | Mode 0 is red/green/blue; modes 1 and 2 are reserved |
| Operating mode | Setup / Lab Mode | Field Mode or Setup / Lab Mode |

## Operating modes

### Field Mode

Field Mode hides the live magnetometer status block and does not poll the `/status` endpoint from the web page. This mode is intended for field testing, demonstrations, or deployments where the web UI is not being actively watched. If the TLV493D magnetometer cannot be read in Field Mode, the LED ring blinks red at 250 ms on / 250 ms off.

### Setup / Lab Mode

Setup / Lab Mode displays live magnetometer readings, magnitude, threshold state, LED state, and battery-area temperature. The status block updates using the `/status` JSON endpoint. Invalid thermistor values are shown as `--` with a read-error status instead of displaying a misleading zero value.

## Battery temperature protection

The battery-area thermistor logic is intended to reduce risk during enclosed operation.

| Temperature | Behavior |
| --- | --- |
| Below 40 °C | Normal |
| 40 °C to below 45 °C | Warning shown in the web UI |
| 45 °C or higher | LED off, trigger disabled, fault state active |
| Below 43 °C after fault | Fault clears and device re-arms |
| Thermistor read error | Red blink, 500 ms on / 500 ms off; trigger disabled until reads recover |

## Arduino IDE setup

Install the ESP32 board package and required libraries:

- ESP32 board support package
- `WiFi.h`, `WebServer.h`, and `Preferences.h` from ESP32 Arduino core
- Adafruit NeoPixel library
- Infineon/XENSIV TLx493D library that provides `TLx493D_inc.hpp`

Select the appropriate ESP32-C3 board for your hardware and upload the sketch.

## Known release notes

Hardcoded AP password
Use at own risk
