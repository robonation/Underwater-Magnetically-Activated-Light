/*
  RoboNation RoboSUb and RobotX Pipeline Light/Sensor Demo Test System
  Firmware to test a mock unit closely replicating the devices used in competitions

  ESP32-C3 Magnetic Trigger Light
  Hardware:
  - ESP32-C3 Mini / Super Mini
  - Adafruit TLV493D 3D Magnetometer
  - WS2812 RGB LED Ring, 16 LEDs
  - DFRobot 18650 power bank supplying 5VDC

  Working pin assignment:
  - TLV493D SDA -> GPIO6
  - TLV493D SCL -> GPIO7
  - TLV493D VIN -> 3V3
  - TLV493D GND -> GND

  - WS2812 DIN -> GPIO10 through 470 ohm resistor
  - WS2812 5V  -> 5V from DFRobot 18650 power bank
  - WS2812 GND -> common GND

  - 1000 uF capacitor across LED ring + and -

  - Access point 
  - Internal temperature 10K NTC thermistor divider:
    3V3 -> 10K fixed resistor -> ADC0/GPIO0/A0 -> thermistor -> GND
    Uses analogReadMilliVolts() for ESP32-C3 ADC accuracy

  Behavior:
  - Initial LED state: solid red
  - If magnetic field magnitude >= threshold for at least 250 ms: solid green
  - If magnet moves away before trigger hold time: return to solid red
  - If magnitude remains >= threshold for trigger hold time: start flash sequence
  - Flash sequence continues for configured duration whether magnet remains or is removed
  - After flash duration completes: return to solid red and re-arm
  - Battery temperature >= 45 C: LED off and trigger disabled
  - Battery thermistor read error: blink red 500 ms on / 500 ms off and trigger disabled
  - Field Mode magnetometer read error: blink red 250 ms on / 250 ms off

  Configuration:
  - ESP32 creates Wi-Fi access point
  - Connect to SSID: MagSensorLight
  - Password: configure123
  - Open: http://192.168.4.1

  Operating Modes:
  - Field Mode: no live web status polling
  - Setup / Lab Mode: status block updates using /status JSON endpoint
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include "TLx493D_inc.hpp"  // XENSIV TLx493D

using namespace ifx::tlx493d;

// -------------------- Pin Settings --------------------

#define LED_PIN   10
#define NUM_LEDS  16

#define I2C_SDA   6
#define I2C_SCL   7

#define BATTERY_THERMISTOR_PIN  0  // A0 / GPIO0 on ESP32-C3 Super Mini

// -------------------- Wi-Fi Access Point Settings --------------------

const char* AP_SSID = "MagSensorLight";
const char* AP_PASSWORD = "configure123";  // Must be at least 8 characters

WebServer server(80);
Preferences preferences;

// -------------------- Configurable Settings --------------------

double magThresholdMt = 1.5;
unsigned long successConfirmMs = 2000;
unsigned long successFlashTotalMs = 120000;
uint8_t ledBrightness = 204;  // 80% of 255
int flashSequenceMode = 0;

// 0 = Field Mode
// 1 = Setup / Lab Mode
int operationMode = 1;

// -------------------- Fixed Behavior Settings --------------------

const unsigned long GREEN_CONFIRM_MS = 250;
const unsigned long MAG_READ_INTERVAL_MS = 50;

const unsigned long FLASH_ON_MS = 1000;
const unsigned long FLASH_OFF_MS = 1000;

const unsigned long STATUS_POLL_INTERVAL_MS = 250;

// -------------------- Battery-Area Thermistor Settings --------------------

const float THERMISTOR_SUPPLY_MV = 3300.0;        // Update if measured 3V3 rail differs
const float THERMISTOR_FIXED_RESISTOR_OHMS = 10000.0;
const float THERMISTOR_NOMINAL_OHMS = 10000.0;   // 10K NTC at 25C
const float THERMISTOR_NOMINAL_TEMP_C = 25.0;
const float THERMISTOR_BETA = 3950.0;
const int THERMISTOR_SAMPLE_COUNT = 50;
const unsigned long THERMISTOR_SAMPLE_SPACING_MS = 2;
const unsigned long THERMISTOR_READ_INTERVAL_MS = 500;

const unsigned long MAG_READ_ERROR_BLINK_INTERVAL_MS = 250;
const unsigned long THERMISTOR_ERROR_BLINK_INTERVAL_MS = 500;

const float BATTERY_TEMP_WARNING_C = 40.0;
const float BATTERY_TEMP_FAULT_C = 45.0;
const float BATTERY_TEMP_FAULT_CLEAR_C = 43.0;  // Hysteresis to avoid rapid fault cycling


// -------------------- Battery-Area Thermistor Class --------------------

class BatteryThermistor {
public:
  BatteryThermistor(int pin,
                    float supplyMv,
                    float fixedResistorOhms,
                    float nominalOhms,
                    float nominalTempC,
                    float beta,
                    int sampleCount,
                    unsigned long sampleSpacingMs)
    : _pin(pin),
      _supplyMv(supplyMv),
      _fixedResistorOhms(fixedResistorOhms),
      _nominalOhms(nominalOhms),
      _nominalTempC(nominalTempC),
      _beta(beta),
      _sampleCount(sampleCount),
      _sampleSpacingMs(sampleSpacingMs) {
  }

  void begin() {
    analogReadResolution(12);
    analogSetPinAttenuation(_pin, ADC_11db);
    startSampling(0);
  }

  // Non-blocking update.
  // Returns true only when a new averaged thermistor reading has been completed.
  bool update(unsigned long now) {
    if (!_sampling) {
      startSampling(now);
    }

    if (now - _lastSampleMs < _sampleSpacingMs) {
      return false;
    }

    _lastSampleMs = now;

    _rawTotal += analogRead(_pin);
    _mvTotal += analogReadMilliVolts(_pin);
    _samplesTaken++;

    if (_samplesTaken < _sampleCount) {
      return false;
    }

    finishSampling();
    _sampling = false;
    return true;
  }

  bool isValid() const {
    return _valid;
  }

  bool isSampling() const {
    return _sampling;
  }

  float rawAdc() const {
    return _rawAdc;
  }

  float voltageMv() const {
    return _voltageMv;
  }

  float resistanceOhms() const {
    return _resistanceOhms;
  }

  float tempC() const {
    return _tempC;
  }

  float tempF() const {
    return _tempF;
  }

private:
  void startSampling(unsigned long now) {
    _sampling = true;
    _samplesTaken = 0;
    _rawTotal = 0;
    _mvTotal = 0;
    _lastSampleMs = now - _sampleSpacingMs;
  }

  void finishSampling() {
    _rawAdc = (float)_rawTotal / _sampleCount;
    _voltageMv = (float)_mvTotal / _sampleCount;

    if (_voltageMv <= 0.0 || _voltageMv >= _supplyMv) {
      _valid = false;
      _resistanceOhms = NAN;
      _tempC = NAN;
      _tempF = NAN;
      return;
    }

    // Wiring:
    // 3.3V -> fixed resistor -> ADC pin -> NTC thermistor -> GND
    _resistanceOhms = _fixedResistorOhms * _voltageMv / (_supplyMv - _voltageMv);

    float nominalTempK = _nominalTempC + 273.15;

    float tempK = 1.0 / (
      (1.0 / nominalTempK) +
      (1.0 / _beta) * log(_resistanceOhms / _nominalOhms)
    );

    _tempC = tempK - 273.15;
    _tempF = (_tempC * 9.0 / 5.0) + 32.0;
    _valid = true;
  }

  int _pin;
  float _supplyMv;
  float _fixedResistorOhms;
  float _nominalOhms;
  float _nominalTempC;
  float _beta;
  int _sampleCount;
  unsigned long _sampleSpacingMs;

  bool _sampling = false;
  bool _valid = false;
  int _samplesTaken = 0;
  long _rawTotal = 0;
  long _mvTotal = 0;
  unsigned long _lastSampleMs = 0;

  float _rawAdc = 0.0;
  float _voltageMv = 0.0;
  float _resistanceOhms = NAN;
  float _tempC = NAN;
  float _tempF = NAN;
};

// -------------------- Hardware Objects --------------------

Adafruit_NeoPixel ring(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

BatteryThermistor batteryThermistor(
  BATTERY_THERMISTOR_PIN,
  THERMISTOR_SUPPLY_MV,
  THERMISTOR_FIXED_RESISTOR_OHMS,
  THERMISTOR_NOMINAL_OHMS,
  THERMISTOR_NOMINAL_TEMP_C,
  THERMISTOR_BETA,
  THERMISTOR_SAMPLE_COUNT,
  THERMISTOR_SAMPLE_SPACING_MS
);

// This matches your working magnetometer setup
TLx493D_A1B6 magnetometer(Wire, TLx493D_IIC_ADDR_A0_e);

// -------------------- State Machine --------------------

enum LedState {
  IDLE_RED,
  HELD_GREEN,
  SUCCESS_FLASH,
  BATTERY_TEMP_FAULT,
  BATTERY_THERMISTOR_ERROR,
  MAG_READ_ERROR
};

LedState currentState = IDLE_RED;

// -------------------- Timing Variables --------------------

unsigned long lastMagReadMs = 0;

bool magnetCurrentlyAboveThreshold = false;
unsigned long magnetAboveStartMs = 0;

unsigned long successFlashStartMs = 0;
unsigned long lastFlashChangeMs = 0;
bool flashLedOn = false;
int flashColorIndex = 0;

unsigned long lastThermistorReadStartMs = 0;
bool batteryTempFaultActive = false;
bool batteryThermistorErrorActive = false;
bool fieldMagReadErrorActive = false;

unsigned long lastFaultBlinkChangeMs = 0;
bool faultBlinkLedOn = false;

// -------------------- Magnetometer Status Values --------------------

double latestMagX = 0.0;
double latestMagY = 0.0;
double latestMagZ = 0.0;
double latestMagMagnitude = 0.0;
double latestTemperature = 0.0;

bool latestMagReadOk = false;
bool latestAboveThreshold = false;
unsigned long latestMagReadTimeMs = 0;

// -------------------- Battery-Area Thermistor Status Values --------------------

bool latestBatteryTempReadOk = false;
float latestBatteryTempC = NAN;
float latestBatteryTempF = NAN;
float latestBatteryThermistorRawAdc = 0.0;
float latestBatteryThermistorMv = 0.0;
float latestBatteryThermistorOhms = NAN;
unsigned long latestBatteryTempReadTimeMs = 0;

// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("ESP32-C3 Magnetic Trigger Light");

  loadConfig();

  ring.begin();
  ring.setBrightness(ledBrightness);
  ring.clear();
  ring.show();

  setRingColor(255, 0, 0);  // Initial state: Red

  batteryThermistor.begin();
  while (!batteryThermistor.update(millis())) {
    delay(1);
  }
  processCompletedBatteryThermistorRead(millis());

  setupConfigPortal();

  Serial.println("TLV493D SDA = GPIO6, SCL = GPIO7");
  Serial.println("WS2812 DIN = GPIO10");
  Serial.println("Battery thermistor ADC = A0 / GPIO0");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Serial.println("Starting TLV493D...");
  magnetometer.begin();
  magnetometer.setSensitivity(TLx493D_FULL_RANGE_e);

  delay(250);

  updateMagnetStatusOnly(millis());

  if (latestMagReadOk) {
    Serial.println("TLV493D first read OK.");
  } else {
    Serial.println("TLV493D first read failed.");
  }

  Serial.println("System ready.");
}

// -------------------- Main Loop --------------------

void loop() {
  server.handleClient();

  unsigned long now = millis();

  if (updateBatteryThermistor(now)) {
    processCompletedBatteryThermistorRead(now);
  }

  // Battery-area over-temperature fault disables the trigger and turns off the LED.
  // The web server keeps running so the status can still be checked in Setup / Lab Mode.
  if (batteryTempFaultActive) {
    return;
  }

  // A thermistor read error disables the trigger and blinks red at 500 ms on / 500 ms off.
  if (batteryThermistorErrorActive) {
    updateFaultBlink(now, THERMISTOR_ERROR_BLINK_INTERVAL_MS);
    return;
  }

  // If the user switches from Field Mode to Setup / Lab Mode while a field-only
  // magnetometer LED fault is active, stop the visible blink and let the web UI
  // report the read status instead.
  if (fieldMagReadErrorActive && operationMode != 0) {
    clearMagReadError(now);
  }

  if (now - lastMagReadMs >= MAG_READ_INTERVAL_MS) {
    lastMagReadMs = now;

    if (currentState == SUCCESS_FLASH) {
      // During flash sequence, keep reading magnetometer for web status,
      // but do not let magnet state cancel or restart the flash sequence.
      updateMagnetStatusOnly(now);
    } else {
      updateMagnetLogic(now);
    }
  }

  // In Field Mode, a TLV493D read failure blinks red at 250 ms on / 250 ms off.
  // Continue attempting magnetometer reads so the fault can automatically clear.
  if (fieldMagReadErrorActive && operationMode == 0) {
    updateFaultBlink(now, MAG_READ_ERROR_BLINK_INTERVAL_MS);
    return;
  }

  if (currentState == SUCCESS_FLASH) {
    updateSuccessFlash(now);
  }
}


// -------------------- Battery-Area Thermistor Logic --------------------

bool updateBatteryThermistor(unsigned long now) {
  if (!batteryThermistor.isSampling() && (now - lastThermistorReadStartMs < THERMISTOR_READ_INTERVAL_MS)) {
    return false;
  }

  if (!batteryThermistor.isSampling()) {
    lastThermistorReadStartMs = now;
  }

  return batteryThermistor.update(now);
}

void processCompletedBatteryThermistorRead(unsigned long now) {
  latestBatteryTempReadOk = batteryThermistor.isValid();
  latestBatteryTempReadTimeMs = now;

  latestBatteryThermistorRawAdc = batteryThermistor.rawAdc();
  latestBatteryThermistorMv = batteryThermistor.voltageMv();
  latestBatteryThermistorOhms = batteryThermistor.resistanceOhms();
  latestBatteryTempC = batteryThermistor.tempC();
  latestBatteryTempF = batteryThermistor.tempF();

  if (!latestBatteryTempReadOk) {
    Serial.println("Battery thermistor read error. Trigger disabled; blinking red 500 ms on / 500 ms off.");
    enterBatteryThermistorError(now);
    return;
  }

  if (batteryThermistorErrorActive) {
    clearBatteryThermistorError(now);
  }

  if (!batteryTempFaultActive && latestBatteryTempC >= BATTERY_TEMP_FAULT_C) {
    enterBatteryTempFault(now);
    return;
  }

  if (batteryTempFaultActive && latestBatteryTempC < BATTERY_TEMP_FAULT_CLEAR_C) {
    clearBatteryTempFault(now);
  }
}

void enterBatteryTempFault(unsigned long now) {
  batteryTempFaultActive = true;
  batteryThermistorErrorActive = false;
  fieldMagReadErrorActive = false;

  magnetCurrentlyAboveThreshold = false;
  magnetAboveStartMs = 0;

  successFlashStartMs = 0;
  lastFlashChangeMs = 0;
  flashLedOn = false;

  Serial.print("BATTERY TEMP FAULT: ");
  Serial.print(latestBatteryTempC, 2);
  Serial.print(" C / ");
  Serial.print(latestBatteryTempF, 2);
  Serial.println(" F. LED off and trigger disabled.");

  setState(BATTERY_TEMP_FAULT, now);
}

void clearBatteryTempFault(unsigned long now) {
  batteryTempFaultActive = false;

  magnetCurrentlyAboveThreshold = false;
  magnetAboveStartMs = 0;

  Serial.print("Battery temperature fault cleared: ");
  Serial.print(latestBatteryTempC, 2);
  Serial.print(" C / ");
  Serial.print(latestBatteryTempF, 2);
  Serial.println(" F. Returning to IDLE_RED.");

  setState(IDLE_RED, now);
}

void enterBatteryThermistorError(unsigned long now) {
  if (batteryThermistorErrorActive) {
    return;
  }

  batteryThermistorErrorActive = true;
  fieldMagReadErrorActive = false;

  magnetCurrentlyAboveThreshold = false;
  magnetAboveStartMs = 0;

  successFlashStartMs = 0;
  lastFlashChangeMs = 0;
  flashLedOn = false;

  faultBlinkLedOn = true;
  lastFaultBlinkChangeMs = now;

  setState(BATTERY_THERMISTOR_ERROR, now);
  setRingColor(255, 0, 0);
}

void clearBatteryThermistorError(unsigned long now) {
  if (!batteryThermistorErrorActive) {
    return;
  }

  batteryThermistorErrorActive = false;

  Serial.println("Battery thermistor read recovered. Returning to IDLE_RED.");
  setState(IDLE_RED, now);
}

const char* getBatteryTempStatusText() {
  if (!latestBatteryTempReadOk) {
    return "READ ERROR";
  }

  if (batteryTempFaultActive || latestBatteryTempC >= BATTERY_TEMP_FAULT_C) {
    return "FAULT";
  }

  if (latestBatteryTempC >= BATTERY_TEMP_WARNING_C) {
    return "WARNING";
  }

  return "NORMAL";
}

// -------------------- Magnet Logic --------------------

void updateMagnetLogic(unsigned long now) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double temperature = 0.0;

  bool readOk = magnetometer.getMagneticFieldAndTemperature(&x, &y, &z, &temperature);

  latestMagReadOk = readOk;
  latestMagReadTimeMs = now;

  if (!readOk) {
    Serial.println("TLV493D read error");
    enterMagReadError(now);
    return;
  }

  if (fieldMagReadErrorActive) {
    clearMagReadError(now);
  }

  double magnitude = sqrt((x * x) + (y * y) + (z * z));
  bool aboveThreshold = magnitude >= magThresholdMt;

  latestMagX = x;
  latestMagY = y;
  latestMagZ = z;
  latestTemperature = temperature;
  latestMagMagnitude = magnitude;
  latestAboveThreshold = aboveThreshold;

  Serial.print("Mag: ");
  Serial.print(magnitude, 3);
  Serial.print(" mT, Threshold: ");
  Serial.print(magThresholdMt, 3);
  Serial.print(" mT, State: ");
  printStateName(currentState);
  Serial.println();

  if (aboveThreshold) {
    if (!magnetCurrentlyAboveThreshold) {
      magnetCurrentlyAboveThreshold = true;
      magnetAboveStartMs = now;

      Serial.println("Magnet detected above threshold. Starting hold timer.");
    }

    unsigned long heldMs = now - magnetAboveStartMs;

    if (heldMs >= successConfirmMs) {
      setState(SUCCESS_FLASH, now);
    } else if (heldMs >= GREEN_CONFIRM_MS && currentState == IDLE_RED) {
      setState(HELD_GREEN, now);
    }
  } else {
    magnetCurrentlyAboveThreshold = false;
    magnetAboveStartMs = 0;

    if (currentState != IDLE_RED) {
      setState(IDLE_RED, now);
    }
  }
}

void updateMagnetStatusOnly(unsigned long now) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double temperature = 0.0;

  bool readOk = magnetometer.getMagneticFieldAndTemperature(&x, &y, &z, &temperature);

  latestMagReadOk = readOk;
  latestMagReadTimeMs = now;

  if (!readOk) {
    enterMagReadError(now);
    return;
  }

  if (fieldMagReadErrorActive) {
    clearMagReadError(now);
  }

  double magnitude = sqrt((x * x) + (y * y) + (z * z));

  latestMagX = x;
  latestMagY = y;
  latestMagZ = z;
  latestTemperature = temperature;
  latestMagMagnitude = magnitude;
  latestAboveThreshold = magnitude >= magThresholdMt;
}

// -------------------- State Handling --------------------

void setState(LedState newState, unsigned long now) {
  if (newState == currentState) {
    return;
  }

  currentState = newState;

  switch (currentState) {
    case IDLE_RED:
      Serial.println("State changed: IDLE_RED");

      magnetCurrentlyAboveThreshold = false;
      magnetAboveStartMs = 0;

      setRingColor(255, 0, 0);
      break;

    case HELD_GREEN:
      Serial.println("State changed: HELD_GREEN");
      setRingColor(0, 255, 0);
      break;

    case SUCCESS_FLASH:
      Serial.println("State changed: SUCCESS_FLASH");
      Serial.print("Starting flash cycle for ");
      Serial.print(successFlashTotalMs);
      Serial.println(" ms.");

      successFlashStartMs = now;

      flashLedOn = true;
      flashColorIndex = 0;
      lastFlashChangeMs = now;

      setFlashColor(flashColorIndex);
      break;

    case BATTERY_TEMP_FAULT:
      Serial.println("State changed: BATTERY_TEMP_FAULT");
      setRingColor(0, 0, 0);
      break;

    case BATTERY_THERMISTOR_ERROR:
      Serial.println("State changed: BATTERY_THERMISTOR_ERROR");
      break;

    case MAG_READ_ERROR:
      Serial.println("State changed: MAG_READ_ERROR");
      break;
  }
}

// -------------------- Fault Blink Handling --------------------

void enterMagReadError(unsigned long now) {
  if (operationMode != 0 || fieldMagReadErrorActive || batteryTempFaultActive || batteryThermistorErrorActive) {
    return;
  }

  fieldMagReadErrorActive = true;
  magnetCurrentlyAboveThreshold = false;
  magnetAboveStartMs = 0;

  faultBlinkLedOn = true;
  lastFaultBlinkChangeMs = now;

  setState(MAG_READ_ERROR, now);
  setRingColor(255, 0, 0);
}

void clearMagReadError(unsigned long now) {
  if (!fieldMagReadErrorActive) {
    return;
  }

  fieldMagReadErrorActive = false;

  Serial.println("TLV493D read recovered. Returning to IDLE_RED.");
  setState(IDLE_RED, now);
}

void updateFaultBlink(unsigned long now, unsigned long blinkIntervalMs) {
  if (now - lastFaultBlinkChangeMs >= blinkIntervalMs) {
    lastFaultBlinkChangeMs = now;
    faultBlinkLedOn = !faultBlinkLedOn;

    if (faultBlinkLedOn) {
      setRingColor(255, 0, 0);
    } else {
      setRingColor(0, 0, 0);
    }
  }
}

// -------------------- Flashing Pattern --------------------

void updateSuccessFlash(unsigned long now) {
  if (now - successFlashStartMs >= successFlashTotalMs) {
    Serial.println("Flash cycle complete. Returning to IDLE_RED.");
    setState(IDLE_RED, now);
    return;
  }

  unsigned long interval = flashLedOn ? FLASH_ON_MS : FLASH_OFF_MS;

  if (now - lastFlashChangeMs >= interval) {
    lastFlashChangeMs = now;
    flashLedOn = !flashLedOn;

    if (flashLedOn) {
      flashColorIndex++;

      if (flashColorIndex > 2) {
        flashColorIndex = 0;
      }

      setFlashColor(flashColorIndex);
    } else {
      setRingColor(0, 0, 0);
    }
  }
}

void setFlashColor(int colorIndex) {
  switch (flashSequenceMode) {
    case 0:
    default:
      // Sequence 0: Red, Green, Blue
      switch (colorIndex) {
        case 0:
          setRingColor(255, 0, 0);
          break;

        case 1:
          setRingColor(0, 255, 0);
          break;

        case 2:
          setRingColor(0, 0, 255);
          break;
      }
      break;

    case 1:
      // Reserved future sequence 1
      // For now, use Red, Green, Blue
      switch (colorIndex) {
        case 0:
          setRingColor(255, 0, 0);
          break;

        case 1:
          setRingColor(0, 255, 0);
          break;

        case 2:
          setRingColor(0, 0, 255);
          break;
      }
      break;

    case 2:
      // Reserved future sequence 2
      // For now, use Red, Green, Blue
      switch (colorIndex) {
        case 0:
          setRingColor(255, 0, 0);
          break;

        case 1:
          setRingColor(0, 255, 0);
          break;

        case 2:
          setRingColor(0, 0, 255);
          break;
      }
      break;
  }
}

// -------------------- LED Helper --------------------

void setRingColor(uint8_t red, uint8_t green, uint8_t blue) {
  ring.setBrightness(ledBrightness);

  for (int i = 0; i < NUM_LEDS; i++) {
    ring.setPixelColor(i, ring.Color(red, green, blue));
  }

  ring.show();
}

// -------------------- Configuration Storage --------------------

void loadConfig() {
  preferences.begin("maglight", true);  // read-only

  magThresholdMt = preferences.getDouble("threshold", 1.5);
  successConfirmMs = preferences.getULong("holdMs", 2000);
  successFlashTotalMs = preferences.getULong("flashDur", 120000);
  ledBrightness = preferences.getUChar("bright", 204);
  flashSequenceMode = preferences.getInt("flashMode", 0);
  operationMode = preferences.getInt("opMode", 1);

  preferences.end();

  applyConfigGuardrails();

  Serial.println("Loaded configuration:");
  printConfig();
}

void saveConfig(double newThreshold,
                unsigned long newHoldMs,
                unsigned long newFlashDurationMs,
                uint8_t newBrightness,
                int newFlashMode,
                int newOperationMode) {
  magThresholdMt = newThreshold;
  successConfirmMs = newHoldMs;
  successFlashTotalMs = newFlashDurationMs;
  ledBrightness = newBrightness;
  flashSequenceMode = newFlashMode;
  operationMode = newOperationMode;

  applyConfigGuardrails();

  preferences.begin("maglight", false);  // read/write

  preferences.putDouble("threshold", magThresholdMt);
  preferences.putULong("holdMs", successConfirmMs);
  preferences.putULong("flashDur", successFlashTotalMs);
  preferences.putUChar("bright", ledBrightness);
  preferences.putInt("flashMode", flashSequenceMode);
  preferences.putInt("opMode", operationMode);

  preferences.end();

  ring.setBrightness(ledBrightness);

  // Re-apply the visible color at the new brightness.
  // During SUCCESS_FLASH, brightness applies on the next flash update.
  if (currentState == BATTERY_TEMP_FAULT) {
    setRingColor(0, 0, 0);
  } else if (currentState == BATTERY_THERMISTOR_ERROR || currentState == MAG_READ_ERROR) {
    if (faultBlinkLedOn) {
      setRingColor(255, 0, 0);
    } else {
      setRingColor(0, 0, 0);
    }
  } else if (currentState == IDLE_RED) {
    setRingColor(255, 0, 0);
  } else if (currentState == HELD_GREEN) {
    setRingColor(0, 255, 0);
  }

  Serial.println("Saved configuration:");
  printConfig();
}

void applyConfigGuardrails() {
  if (magThresholdMt < 0.05) {
    magThresholdMt = 0.05;
  }

  if (magThresholdMt > 200.0) {
    magThresholdMt = 200.0;
  }

  if (successConfirmMs < GREEN_CONFIRM_MS) {
    successConfirmMs = GREEN_CONFIRM_MS;
  }

  if (successConfirmMs > 60000) {
    successConfirmMs = 60000;
  }

  if (successFlashTotalMs < 1000) {
    successFlashTotalMs = 1000;
  }

  if (successFlashTotalMs > 3600000) {
    successFlashTotalMs = 3600000;
  }

  if (ledBrightness < 1) {
    ledBrightness = 1;
  }

  if (flashSequenceMode < 0 || flashSequenceMode > 2) {
    flashSequenceMode = 0;
  }

  if (operationMode < 0 || operationMode > 1) {
    operationMode = 1;
  }
}

void printConfig() {
  Serial.print("  Threshold: ");
  Serial.print(magThresholdMt, 3);
  Serial.println(" mT");

  Serial.print("  Hold time: ");
  Serial.print(successConfirmMs);
  Serial.println(" ms");

  Serial.print("  Flash duration: ");
  Serial.print(successFlashTotalMs);
  Serial.println(" ms");

  Serial.print("  Brightness: ");
  Serial.print(ledBrightness);
  Serial.println(" / 255");

  Serial.print("  Flash mode: ");
  Serial.println(flashSequenceMode);

  Serial.print("  Operation mode: ");
  Serial.print(operationMode);
  Serial.print(" - ");
  Serial.println(getOperationModeText(operationMode));
}

// -------------------- Wi-Fi Configuration Portal --------------------

void setupConfigPortal() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  IPAddress ip = WiFi.softAPIP();

  Serial.println("Configuration access point started.");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Open browser to: http://");
  Serial.println(ip);

  server.on("/", handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reset", HTTP_POST, handleResetDefaults);

  server.begin();

  Serial.println("Web configuration server started.");
}

void handleRoot() {
  String html = "";

  html += "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Mag Sensor Light Config</title>";

  html += "<style>";
  html += "body{font-family:Arial,sans-serif;margin:24px;max-width:620px;background:#f8f8f8;color:#222;}";
  html += "h1{font-size:1.5rem;margin-bottom:0.25rem;}";
  html += "h2{font-size:1.2rem;margin-top:0;}";
  html += "p{line-height:1.4;}";
  html += "label{display:block;margin-top:16px;font-weight:bold;}";
  html += "input,select,button{font-size:1rem;padding:10px;margin-top:6px;width:100%;box-sizing:border-box;}";
  html += "button{margin-top:20px;cursor:pointer;border:0;border-radius:8px;background:#1f6feb;color:white;font-weight:bold;}";
  html += "button.secondary{background:#777;}";
  html += ".card{background:white;border:1px solid #ddd;border-radius:12px;padding:18px;margin-top:16px;box-shadow:0 2px 8px rgba(0,0,0,0.06);}";
  html += ".note{font-size:0.9rem;color:#555;}";
  html += ".value{font-family:monospace;background:#eee;padding:2px 5px;border-radius:4px;}";
  html += ".ok{color:#137333;font-weight:bold;}";
  html += ".warn{color:#b06000;font-weight:bold;}";
  html += ".bad{color:#b3261e;font-weight:bold;}";
  html += ".mode{display:inline-block;padding:4px 8px;border-radius:6px;background:#eef;}";
  html += "</style>";

  html += "</head><body>";

  html += "<h1>Mag Sensor Light Configuration</h1>";
  html += "<p class='note'>Settings are saved on the ESP32 and survive power cycling.</p>";

  html += "<p>Operating Mode: <span class='mode'>";
  html += getOperationModeText(operationMode);
  html += "</span></p>";

  // -------------------- Magnetometer Status Block --------------------

  if (operationMode == 1) {
    html += "<div class='card'>";
    html += "<h2>Magnetometer Status</h2>";
    html += "<p class='note'>Setup / Lab Mode: this block updates every ";
    html += String(STATUS_POLL_INTERVAL_MS);
    html += " ms without refreshing the full page.</p>";

    html += "<p>Status: <span id='statusReadOk' class='value'>--</span></p>";

    html += "<p>X: <span id='magX' class='value'>--</span> mT</p>";
    html += "<p>Y: <span id='magY' class='value'>--</span> mT</p>";
    html += "<p>Z: <span id='magZ' class='value'>--</span> mT</p>";

    html += "<p>Magnitude: <span id='magMagnitude' class='value'>--</span> mT</p>";

    html += "<p>Threshold State: <span id='thresholdState' class='value'>--</span></p>";

    html += "<p>Temperature: <span id='magTemp' class='value'>--</span> C</p>";

    html += "<p>Last Read: <span id='lastReadAge' class='value'>--</span> seconds ago</p>";

    html += "<p>Current Light State: <span id='lightState' class='value'>--</span></p>";

    html += "<hr>";
    html += "<p>Battery-Area Temp: <span id='batteryTempC' class='value'>--</span> C / <span id='batteryTempF' class='value'>--</span> F</p>";
    html += "<p>Battery Temp Status: <span id='batteryTempStatus' class='value'>--</span></p>";
    html += "<p>Thermistor: <span id='batteryThermistorOhms' class='value'>--</span> ohms, <span id='batteryThermistorMv' class='value'>--</span> mV</p>";
    html += "<p class='note'>At 45 C / 113 F, the LED is turned off and the trigger is disabled.</p>";

    html += "</div>";
  } else {
    html += "<div class='card'>";
    html += "<h2>Field Mode</h2>";
    html += "<p class='note'>Live magnetometer status is hidden and not polled in Field Mode. ";
    html += "Use Setup / Lab Mode for magnetic strength testing and threshold tuning.</p>";
    html += "</div>";
  }

  // -------------------- Current Settings Block --------------------

  html += "<div class='card'>";
  html += "<h2>Current Settings</h2>";

  html += "<p>Threshold: <span class='value'>";
  html += String(magThresholdMt, 3);
  html += " mT</span></p>";

  html += "<p>Hold time: <span class='value'>";
  html += String(successConfirmMs);
  html += " ms</span></p>";

  html += "<p>Flash duration: <span class='value'>";
  html += String(successFlashTotalMs);
  html += " ms</span></p>";

  html += "<p>Brightness: <span class='value'>";
  html += String(ledBrightness);
  html += " / 255</span></p>";

  html += "<p>Flash mode: <span class='value'>";
  html += String(flashSequenceMode);
  html += "</span></p>";

  html += "<p>Operation mode: <span class='value'>";
  html += String(operationMode);
  html += " - ";
  html += getOperationModeText(operationMode);
  html += "</span></p>";

  html += "</div>";

  // -------------------- Edit Settings Block --------------------

  html += "<div class='card'>";
  html += "<h2>Edit Settings</h2>";
  html += "<form method='POST' action='/save'>";

  html += "<label for='threshold'>Magnetic Magnitude Threshold (mT)</label>";
  html += "<input id='threshold' name='threshold' type='number' step='0.01' min='0.05' max='200' value='";
  html += String(magThresholdMt, 3);
  html += "'>";

  html += "<label for='holdMs'>Magnet Hold Time to Trigger Flash (ms)</label>";
  html += "<input id='holdMs' name='holdMs' type='number' step='50' min='250' max='60000' value='";
  html += String(successConfirmMs);
  html += "'>";

  html += "<label for='flashDurationMs'>Total Flash Duration (ms)</label>";
  html += "<input id='flashDurationMs' name='flashDurationMs' type='number' step='1000' min='1000' max='3600000' value='";
  html += String(successFlashTotalMs);
  html += "'>";

  html += "<label for='brightness'>Brightness (1-255)</label>";
  html += "<input id='brightness' name='brightness' type='number' step='1' min='1' max='255' value='";
  html += String(ledBrightness);
  html += "'>";

  html += "<label for='flashMode'>Flash Sequence</label>";
  html += "<select id='flashMode' name='flashMode'>";

  html += "<option value='0'";
  if (flashSequenceMode == 0) html += " selected";
  html += ">0 - Red, Green, Blue - 1s on / 1s off</option>";

  html += "<option value='1'";
  if (flashSequenceMode == 1) html += " selected";
  html += ">1 - Reserved Sequence 1</option>";

  html += "<option value='2'";
  if (flashSequenceMode == 2) html += " selected";
  html += ">2 - Reserved Sequence 2</option>";

  html += "</select>";

  html += "<label for='opMode'>Operating Mode</label>";
  html += "<select id='opMode' name='opMode'>";

  html += "<option value='0'";
  if (operationMode == 0) html += " selected";
  html += ">0 - Field Mode - no live status polling</option>";

  html += "<option value='1'";
  if (operationMode == 1) html += " selected";
  html += ">1 - Setup / Lab Mode - live status polling</option>";

  html += "</select>";

  html += "<button type='submit'>Save Settings</button>";
  html += "</form>";
  html += "</div>";

  // -------------------- Reset Block --------------------

  html += "<div class='card'>";
  html += "<h2>Defaults</h2>";
  html += "<p class='note'>Default values: threshold 1.5 mT, hold time 2000 ms, flash duration 120000 ms, brightness 204, flash mode 0, Setup / Lab Mode.</p>";
  html += "<form method='POST' action='/reset'>";
  html += "<button class='secondary' type='submit'>Reset to Defaults</button>";
  html += "</form>";
  html += "</div>";

  // -------------------- Setup / Lab Mode JavaScript Polling --------------------

  if (operationMode == 1) {
    html += "<script>";
    html += "async function updateStatus(){";
    html += "try{";
    html += "const r=await fetch('/status',{cache:'no-store'});";
    html += "const s=await r.json();";

    html += "const statusEl=document.getElementById('statusReadOk');";
    html += "const thresholdEl=document.getElementById('thresholdState');";

    html += "statusEl.textContent=s.readOk?'OK':'READ ERROR';";
    html += "statusEl.className=s.readOk?'value ok':'value bad';";

    html += "document.getElementById('magX').textContent=Number(s.x).toFixed(3);";
    html += "document.getElementById('magY').textContent=Number(s.y).toFixed(3);";
    html += "document.getElementById('magZ').textContent=Number(s.z).toFixed(3);";
    html += "document.getElementById('magMagnitude').textContent=Number(s.magnitude).toFixed(3);";
    html += "document.getElementById('magTemp').textContent=Number(s.temperature).toFixed(2);";

    html += "thresholdEl.textContent=s.aboveThreshold?'ABOVE THRESHOLD':'below threshold';";
    html += "thresholdEl.className=s.aboveThreshold?'value warn':'value ok';";

    html += "document.getElementById('lightState').textContent=s.state;";
    html += "document.getElementById('lastReadAge').textContent=(Number(s.lastReadAgeMs)/1000).toFixed(1);";

    html += "const tempOk=s.batteryTempReadOk;";
    html += "document.getElementById('batteryTempC').textContent=tempOk?Number(s.batteryTempC).toFixed(2):'--';";
    html += "document.getElementById('batteryTempF').textContent=tempOk?Number(s.batteryTempF).toFixed(2):'--';";
    html += "document.getElementById('batteryThermistorOhms').textContent=tempOk?Number(s.batteryThermistorOhms).toFixed(1):'--';";
    html += "document.getElementById('batteryThermistorMv').textContent=Number(s.batteryThermistorMv).toFixed(1);";
    html += "const batteryTempStatusEl=document.getElementById('batteryTempStatus');";
    html += "batteryTempStatusEl.textContent=s.batteryTempStatus;";
    html += "batteryTempStatusEl.className=!tempOk?'value bad':(s.batteryTempFault?'value bad':(s.batteryTempC>=40?'value warn':'value ok'));";

    html += "}catch(e){";
    html += "const statusEl=document.getElementById('statusReadOk');";
    html += "if(statusEl){";
    html += "statusEl.textContent='CONNECTION ERROR';";
    html += "statusEl.className='value bad';";
    html += "}";
    html += "}";
    html += "}";

    html += "setInterval(updateStatus,";
    html += String(STATUS_POLL_INTERVAL_MS);
    html += ");";
    html += "updateStatus();";
    html += "</script>";
  }

  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleStatus() {
  String json = "{";

  json += "\"readOk\":";
  json += latestMagReadOk ? "true" : "false";

  json += ",\"x\":";
  json += String(latestMagX, 3);

  json += ",\"y\":";
  json += String(latestMagY, 3);

  json += ",\"z\":";
  json += String(latestMagZ, 3);

  json += ",\"magnitude\":";
  json += String(latestMagMagnitude, 3);

  json += ",\"temperature\":";
  json += String(latestTemperature, 2);

  json += ",\"batteryTempReadOk\":";
  json += latestBatteryTempReadOk ? "true" : "false";

  json += ",\"batteryTempC\":";
  json += latestBatteryTempReadOk ? String(latestBatteryTempC, 2) : "null";

  json += ",\"batteryTempF\":";
  json += latestBatteryTempReadOk ? String(latestBatteryTempF, 2) : "null";

  json += ",\"batteryThermistorRawAdc\":";
  json += String(latestBatteryThermistorRawAdc, 1);

  json += ",\"batteryThermistorMv\":";
  json += String(latestBatteryThermistorMv, 1);

  json += ",\"batteryThermistorOhms\":";
  json += latestBatteryTempReadOk ? String(latestBatteryThermistorOhms, 1) : "null";

  json += ",\"batteryTempFault\":";
  json += batteryTempFaultActive ? "true" : "false";

  json += ",\"batteryTempStatus\":\"";
  json += getBatteryTempStatusText();
  json += "\"";

  json += ",\"batteryTempWarningC\":";
  json += String(BATTERY_TEMP_WARNING_C, 1);

  json += ",\"batteryTempFaultC\":";
  json += String(BATTERY_TEMP_FAULT_C, 1);

  json += ",\"aboveThreshold\":";
  json += latestAboveThreshold ? "true" : "false";

  json += ",\"threshold\":";
  json += String(magThresholdMt, 3);

  json += ",\"state\":\"";
  json += getStateNameText(currentState);
  json += "\"";

  json += ",\"lastReadAgeMs\":";
  json += String(millis() - latestMagReadTimeMs);

  json += ",\"operationMode\":";
  json += String(operationMode);

  json += "}";

  server.send(200, "application/json", json);
}

void handleSave() {
  double newThreshold = magThresholdMt;
  unsigned long newHoldMs = successConfirmMs;
  unsigned long newFlashDurationMs = successFlashTotalMs;
  int newBrightnessInt = ledBrightness;
  int newFlashMode = flashSequenceMode;
  int newOperationMode = operationMode;

  if (server.hasArg("threshold")) {
    newThreshold = server.arg("threshold").toDouble();
  }

  if (server.hasArg("holdMs")) {
    newHoldMs = server.arg("holdMs").toInt();
  }

  if (server.hasArg("flashDurationMs")) {
    newFlashDurationMs = server.arg("flashDurationMs").toInt();
  }

  if (server.hasArg("brightness")) {
    newBrightnessInt = server.arg("brightness").toInt();
  }

  if (server.hasArg("flashMode")) {
    newFlashMode = server.arg("flashMode").toInt();
  }

  if (server.hasArg("opMode")) {
    newOperationMode = server.arg("opMode").toInt();
  }

  if (newBrightnessInt < 1) {
    newBrightnessInt = 1;
  }

  if (newBrightnessInt > 255) {
    newBrightnessInt = 255;
  }

  saveConfig(
    newThreshold,
    newHoldMs,
    newFlashDurationMs,
    (uint8_t)newBrightnessInt,
    newFlashMode,
    newOperationMode
  );

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleResetDefaults() {
  saveConfig(
    1.5,      // threshold mT
    2000,     // hold time ms
    120000,   // flash duration ms
    204,      // brightness
    0,        // flash sequence mode
    1         // operation mode: Setup / Lab Mode
  );

  server.sendHeader("Location", "/");
  server.send(303);
}

// -------------------- Debug Helpers --------------------

void printStateName(LedState state) {
  Serial.print(getStateNameText(state));
}

const char* getStateNameText(LedState state) {
  switch (state) {
    case IDLE_RED:
      return "IDLE_RED";

    case HELD_GREEN:
      return "HELD_GREEN";

    case SUCCESS_FLASH:
      return "SUCCESS_FLASH";

    case BATTERY_TEMP_FAULT:
      return "BATTERY_TEMP_FAULT";

    case BATTERY_THERMISTOR_ERROR:
      return "BATTERY_THERMISTOR_ERROR";

    case MAG_READ_ERROR:
      return "MAG_READ_ERROR";

    default:
      return "UNKNOWN";
  }
}

const char* getOperationModeText(int mode) {
  switch (mode) {
    case 0:
      return "Field Mode";

    case 1:
      return "Setup / Lab Mode";

    default:
      return "Unknown Mode";
  }
}