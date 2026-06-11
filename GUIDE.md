# Underwater Magnetically Activated Light Usage and Modification Guide

This guide explains how to use the Mag Sensor Light firmware and where to make common code changes.

## Basic use

1. Power the ESP32-C3 and LED ring.
2. On a phone, tablet, or computer, connect to the Wi-Fi network named `MagSensorLight`.
3. Use password `configure123`.
4. Open `http://192.168.4.1` in a browser.
5. Use Setup / Lab Mode to view live magnetic readings and tune the magnetic threshold.
6. Switch to Field Mode when the device is ready for demonstration or deployment.

## Recommended setup workflow

1. Start in **Setup / Lab Mode**.
2. Observe the normal background magnetic magnitude with no magnet near the device.
3. Place the expected triggering magnet at the expected activation distance.
4. Note the displayed magnitude.
5. Set the magnetic threshold above the no-magnet background level and below the intended trigger level.
6. Test the required hold time.
7. Confirm the LED behavior:
   - Red when idle.
   - Green after the magnet is held above threshold for 250 ms.
   - Flashing after the configured hold time.
   - Back to red after the flash duration ends.
8. Switch to **Field Mode** when live status is no longer needed.

## Configuration values

The web page can modify these values without reflashing the ESP32:

| Web setting | Code variable | Default |
| --- | --- | ---: |
| Magnetic Magnitude Threshold | `magThresholdMt` | 1.5 mT |
| Magnet Hold Time | `successConfirmMs` | 2000 ms |
| Total Flash Duration | `successFlashTotalMs` | 120000 ms |
| Brightness | `ledBrightness` | 204 |
| Flash Sequence | `flashSequenceMode` | 0 |
| Operating Mode | `operationMode` | 1 |

The values are stored using ESP32 `Preferences`, so they remain saved after power cycling.

## Important code sections

### Pin assignments

Modify these constants near the top of the sketch if the wiring changes:

```cpp
#define LED_PIN   10
#define NUM_LEDS  16
#define I2C_SDA   6
#define I2C_SCL   7
#define BATTERY_THERMISTOR_PIN  0
```

### Wi-Fi access point name and password

Modify these values if you want a different AP name or password:

```cpp
const char* AP_SSID = "MagSensorLight";
const char* AP_PASSWORD = "configure123";
```

The password must be at least 8 characters.

### Thermistor settings

The thermistor constants are in the **Battery-Area Thermistor Settings** section:

```cpp
const float THERMISTOR_SUPPLY_MV = 3300.0;
const float THERMISTOR_FIXED_RESISTOR_OHMS = 10000.0;
const float THERMISTOR_NOMINAL_OHMS = 10000.0;
const float THERMISTOR_NOMINAL_TEMP_C = 25.0;
const float THERMISTOR_BETA = 3950.0;
```

Update `THERMISTOR_SUPPLY_MV` if the measured 3.3 V rail is significantly different from 3300 mV. Update `THERMISTOR_BETA` if a different thermistor is used.

### Battery temperature limits

The warning, fault, and fault-clear temperatures are here:

```cpp
const float BATTERY_TEMP_WARNING_C = 40.0;
const float BATTERY_TEMP_FAULT_C = 45.0;
const float BATTERY_TEMP_FAULT_CLEAR_C = 43.0;
```

The clear temperature is intentionally lower than the fault temperature to provide hysteresis and prevent rapid fault cycling.

### Flash timing

The current flash pattern is 1 second on and 1 second off:

```cpp
const unsigned long FLASH_ON_MS = 1000;
const unsigned long FLASH_OFF_MS = 1000;
```

The total flash duration is user-configurable from the web UI.

## Modifying the flash sequence

The flash color behavior is controlled by `setFlashColor(int colorIndex)`.

The current mode 0 sequence is:

1. Red
2. Green
3. Blue

To create a new sequence, modify case 1 or case 2 inside `setFlashColor()`.

Example idea for a white-only flash mode:

```cpp
case 1:
  setRingColor(255, 255, 255);
  break;
```

Then update the HTML labels in `handleRoot()` so the web page describes the new sequence accurately.

## Modifying the web page

The configuration page is built in `handleRoot()` using Arduino `String` concatenation.

Common edits:

- Change labels in the **Edit Settings Block**.
- Add status fields in the **Magnetometer Status Block**.
- Update JavaScript polling in the **Setup / Lab Mode JavaScript Polling** section.
- Add new JSON fields in `handleStatus()`.

When adding a new web setting, update all of these locations:

1. Add or modify the global setting variable.
2. Add defaults and loading in `loadConfig()`.
3. Add saving in `saveConfig()`.
4. Add validation in `applyConfigGuardrails()`.
5. Add the HTML input in `handleRoot()`.
6. Read the POST argument in `handleSave()`.
7. Add the value to `handleResetDefaults()` if needed.

## Code review notes and issues to consider

### 1. Thermistor reads are non-blocking

`BatteryThermistor::update(now)` now takes one ADC sample at a time using `millis()` timing instead of using `delay()`. The default behavior still averages 50 samples spaced 2 ms apart, but the main loop remains responsive while the averaging window is collected.

The key constants are:

```cpp
const int THERMISTOR_SAMPLE_COUNT = 50;
const unsigned long THERMISTOR_SAMPLE_SPACING_MS = 2;
const unsigned long THERMISTOR_READ_INTERVAL_MS = 500;
```

Reduce `THERMISTOR_SAMPLE_COUNT` for faster temperature updates, or increase it for more smoothing.

### 2. Invalid thermistor readings are displayed safely

The `/status` endpoint still returns `null` for invalid battery temperature and resistance values. The web JavaScript now checks `s.batteryTempReadOk` before formatting those values, so invalid readings display as `--` and the battery temperature status shows `READ ERROR`.

### 3. Access point is always enabled

The ESP32 access point and web server remain active in both Field Mode and Setup / Lab Mode. Field Mode only hides live status polling from the page.

This is useful for field configuration, but for production or public events, consider whether the AP should be disabled, renamed, or given a unique password.

### 4. Hardcoded Wi-Fi password

The AP password is hardcoded in the sketch. For public GitHub repos, this is acceptable for a demo password, but deployed units should use a unique password or a documented process for changing it.

### 5. Web page is built with dynamic `String` objects

The page is generated by repeatedly appending to a `String`. This is simple and readable, but long-term repeated page requests can contribute to heap fragmentation on small microcontrollers.

For a more robust production version, consider storing static HTML in `PROGMEM` or serving a simpler page.

### 6. Field Mode magnetometer read failure alert

If the TLV493D read fails in Field Mode, the LED ring blinks red at 250 ms on / 250 ms off. The firmware keeps attempting magnetometer reads while blinking, so the fault automatically clears and returns to `IDLE_RED` once reads recover.

In Setup / Lab Mode, the web status block reports the TLV493D read error instead of forcing the visible LED fault blink.

### 7. Thermistor read error fault behavior

If the battery thermistor reading is invalid, the trigger is disabled and the LED ring blinks red at 500 ms on / 500 ms off. The firmware keeps attempting thermistor reads while blinking, so the fault automatically clears and returns to `IDLE_RED` once thermistor readings recover.

A true over-temperature fault still has higher priority: at 45 °C or higher, the LED ring turns off and the trigger remains disabled until the temperature drops below 43 °C.

## Suggested pre-release tests

- Confirm the sketch compiles with the selected ESP32-C3 board package and TLx493D library version.
- Verify the AP starts and the configuration page loads at `http://192.168.4.1`.
- Confirm settings save, survive power cycling, and reset to defaults.
- Measure normal background magnetic magnitude in the final enclosure.
- Tune threshold using the final magnet and final enclosure.
- Confirm 250 ms green transition.
- Confirm configured hold time triggers the flash sequence.
- Confirm the flash sequence continues after magnet removal.
- Confirm the flash duration ends and the unit re-arms.
- Heat-test the thermistor location and confirm LED-off fault at 45 °C.
- Confirm the fault clears below 43 °C.
- Disconnect or force an invalid thermistor reading and confirm red blink at 500 ms on / 500 ms off.
- Force or simulate a TLV493D read failure in Field Mode and confirm red blink at 250 ms on / 250 ms off.
- Test with the LED ring at maximum brightness and confirm power stability.

## Troubleshooting

### Cannot see the Wi-Fi network

- Confirm the ESP32-C3 is powered and running.
- Open the serial monitor at 115200 baud and look for the AP startup messages.
- Confirm the sketch uploaded successfully.

### Web page does not load

- Confirm the computer or phone is connected to `MagSensorLight`.
- Open `http://192.168.4.1` directly.
- Some phones may try to switch back to cellular data because the ESP32 AP has no internet access. Disable mobile data temporarily if needed.

### Magnet readings do not update

- Confirm TLV493D SDA is on GPIO6 and SCL is on GPIO7.
- Confirm the sensor is powered from 3.3 V, not 5 V.
- Confirm the correct TLx493D library is installed.
- Check the serial monitor for `TLV493D read error` messages.

### LED ring does not light

- Confirm the LED ring has 5 V power.
- Confirm LED ground and ESP32 ground are common.
- Confirm DIN is connected to GPIO10 through the 470 ohm resistor.
- Confirm `NUM_LEDS` matches the LED ring.

### Battery temperature looks wrong

- Confirm the divider wiring order:

```text
3V3 -> 10K fixed resistor -> ADC0/GPIO0/A0 -> thermistor -> GND
```

- Confirm the thermistor is 10K nominal at 25 °C.
- Confirm the thermistor Beta value is 3950 or update `THERMISTOR_BETA`.
- Measure the ESP32 3.3 V rail and update `THERMISTOR_SUPPLY_MV` if needed.
