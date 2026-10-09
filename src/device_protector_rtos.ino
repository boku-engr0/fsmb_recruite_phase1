/* Device Protector - ESP32 + FreeRTOS proof of concept

 * Board: ESP32 DevKit (Arduino-ESP32 core)
 * Voltage sensor output: GPIO34 (ADC1)
 * Current sensor output: GPIO35 (ADC1)
 * Relay driver: GPIO26, active LOW by default
 * Serial console: 115200 baud
 *
 * FreeRTOS tasks:
 *   1. samplerTask    - nominal 1 kHz sampling and 200 ms RMS windows
 *   2. protectionTask - state machine and relay output
 *   3. serialTask     - command parsing, telemetry and NVS save/load
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// --------------------------- Hardware --------------------------------------
static constexpr uint8_t VOLTAGE_PIN             = 34;
static constexpr uint8_t CURRENT_PIN             = 35;
static constexpr uint8_t RELAY_PIN               = 26;
static constexpr uint8_t RELAY_ACTIVE_LEVEL      = LOW;
static constexpr uint8_t RELAY_INACTIVE_LEVEL    = HIGH;

// --------------------------- Timing ----------------------------------------
static constexpr uint32_t SAMPLE_PERIOD_MS       = 1;   // nominal 1 kHz
static constexpr uint32_t RMS_WINDOW_MS          = 200; // 10 cycles at 50 Hz
static constexpr uint32_t SENSOR_SETTLE_MS       = 1000;
static constexpr uint32_t PROTECTION_PERIOD_MS   = 20;
static constexpr uint32_t TELEMETRY_PERIOD_MS    = 1000;
static constexpr uint32_t MEASUREMENT_TIMEOUT_MS = 500;
static constexpr size_t   SERIAL_LINE_MAX        = 80;
static constexpr uint32_t CONFIG_MAGIC           = 0x51A7;
static constexpr uint16_t CONFIG_VERSION         = 1;

// --------------------------- Configuration ---------------------------------
struct Config {
  uint32_t magic;
  uint16_t version;
  float    vMin;
  float    vMax;
  float    iMax;
  float    vScale;
  float    iScale;
  uint32_t recoveryMs;
};

static Config config = {
  CONFIG_MAGIC,
  CONFIG_VERSION,
  180.0f, 250.0f, 10.0f,
  1.0f, 1.0f,
  3000UL
};

// --------------------------- State / faults --------------------------------
enum State : uint8_t {
  STATE_STARTUP,
  STATE_WAITING_FOR_SAFE,
  STATE_PROTECTED,
  STATE_TRIPPED
};

enum Fault : uint8_t {
  FAULT_NONE,
  FAULT_UNDERVOLTAGE,
  FAULT_OVERVOLTAGE,
  FAULT_OVERCURRENT,
  FAULT_SENSOR_INVALID
};

struct Measurement {
  float    voltageRms;
  float    currentRms;
  uint32_t sampleCount;
  uint32_t completedAtMs;
  bool     valid;
};

static Measurement measurement      = {0.0f,
     0.0f, 0, 0, false};
static uint32_t    stateSinceMs     = 0;
static uint32_t    bootMs           = 0;
static bool        relayCommandedOn = false;
static uint32_t    lastTelemetryMs  = 0;
static State       state            = STATE_STARTUP;
static Fault       fault            = FAULT_NONE;

// Shared state/configuration is protected across tasks
static SemaphoreHandle_t stateMutex = nullptr;
static SemaphoreHandle_t serialMutex = nullptr;
static Preferences preferences;

// Sampler-owned accumulators (not accessed by other tasks)
static uint32_t sampleCount   = 0;
static uint32_t windowStartMs = 0;
static uint64_t vSum          = 0;
static uint64_t iSum          = 0;
static uint64_t vSqSum        = 0;
static uint64_t iSqSum        = 0;

// --------------------------- Utility ---------------------------------------
static const char *stateName(State s) {
  switch (s) {
    case STATE_STARTUP:          return "STARTUP";
    case STATE_WAITING_FOR_SAFE: return "WAITING_FOR_SAFE";
    case STATE_PROTECTED:        return "PROTECTED";
    case STATE_TRIPPED:          return "TRIPPED";
    default:                     return "UNKNOWN";
  }
}

static const char *faultName(Fault f) {
  switch (f) {
    case FAULT_NONE:            return "NONE";
    case FAULT_UNDERVOLTAGE:    return "UNDERVOLTAGE";
    case FAULT_OVERVOLTAGE:     return "OVERVOLTAGE";
    case FAULT_OVERCURRENT:     return "OVERCURRENT";
    case FAULT_SENSOR_INVALID:  return "SENSOR_INVALID";
    default:                    return "UNKNOWN";
  }
}

static void printLine(const char *s) {
  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println(s);
    xSemaphoreGive(serialMutex);
  }
}

static bool finitePositive(float value) {
  return isfinite(value) && value > 0.0f;
}

static bool configIsValid(const Config &c) {
  return c.magic == CONFIG_MAGIC && c.version == CONFIG_VERSION &&
        isfinite(c.vMin) && isfinite(c.vMax) && isfinite(c.iMax) &&
        finitePositive(c.vScale) && finitePositive(c.iScale) &&
        c.vMin >= 0.0f && c.vMax > c.vMin && c.iMax > 0.0f &&
        c.vMax <= 100000.0f && c.iMax <= 100000.0f &&
        c.vMin <= 100000.0f && c.recoveryMs >= 100UL &&
        c.recoveryMs <= 3600000UL;
}

static void relaySet(bool on) {
  // Called by protectionTask only
  digitalWrite(RELAY_PIN, on ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
  relayCommandedOn = on;
}

static bool readConfigSnapshot(Config &out) {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
  out = config;
  xSemaphoreGive(stateMutex);
  return true;
}

static bool readMeasurementSnapshot(Measurement &out) {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
  out = measurement;
  xSemaphoreGive(stateMutex);
  return true;
}

static void saveConfig() {
  Config copy;
  if (!readConfigSnapshot(copy) || !configIsValid(copy)) {
    printLine("ERR config invalid; not saved");
    return;
  }
  // Preferences stores the struct in ESP32 NVS. Namespace/key are stable
  if (!preferences.begin("protector", false)) {
    printLine("ERR could not open NVS");
    return;
  }
  size_t written = preferences.putBytes("config", &copy, sizeof(copy));
  preferences.end();
  printLine(written == sizeof(copy) ? "OK config saved" : "ERR config save failed");
}

static void loadConfig() {
  Config loaded = config;
  if (!preferences.begin("protector", true)) return;
  size_t len = preferences.getBytesLength("config");
  if (len == sizeof(loaded)) {
    preferences.getBytes("config", &loaded, sizeof(loaded));
    if (configIsValid(loaded)) config = loaded;
  }
  preferences.end();
}

// --------------------------- RMS sampler -----------------------------------
static uint16_t readNormalizedAdc(uint8_t pin) {
  // ESP32 ADC is 12-bit by default. Normalize to a 0..1023 scale to make
  // scale factors conceptually similar to the Nano version. This does NOT
  // make ESP32 and Nano analog front ends electrically equivalent.
  uint16_t raw = analogRead(pin); // nominally 0..4095
  return (uint16_t)((raw + 2U) / 4U);
}

static void publishRmsWindow(uint32_t now) {
  // Require a useful number of samples. At 1 kHz and 200 ms, expect ~200.
  if (sampleCount < 50U) {
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      measurement.valid = false;
      measurement.sampleCount = sampleCount;
      measurement.completedAtMs = now;
      xSemaphoreGive(stateMutex);
    }
  } else {
    const double n = (double)sampleCount;
    const double vMean = (double)vSum / n;
    const double iMean = (double)iSum / n;
    double vVariance = ((double)vSqSum / n) - (vMean * vMean);
    double iVariance = ((double)iSqSum / n) - (iMean * iMean);
    if (vVariance < 0.0) vVariance = 0.0;
    if (iVariance < 0.0) iVariance = 0.0;

    Config cfg;
    if (readConfigSnapshot(cfg)) {
      Measurement next;
      next.voltageRms = (float)(sqrt(vVariance) * cfg.vScale);
      next.currentRms = (float)(sqrt(iVariance) * cfg.iScale);
      next.sampleCount = sampleCount;
      next.completedAtMs = now;
      next.valid = isfinite(next.voltageRms) && isfinite(next.currentRms);
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        measurement = next;
        xSemaphoreGive(stateMutex);
      }
    }
  }

  vSum = iSum = vSqSum = iSqSum = 0;
  sampleCount = 0;
  windowStartMs = now;
}

static void samplerTask(void *parameter) {
  (void)parameter;
  TickType_t lastWake = xTaskGetTickCount();
  windowStartMs = millis();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    uint32_t now = millis();
    uint16_t v = readNormalizedAdc(VOLTAGE_PIN);
    uint16_t i = readNormalizedAdc(CURRENT_PIN);

    vSum += v;
    iSum += i;
    vSqSum += (uint64_t)v * (uint64_t)v;
    iSqSum += (uint64_t)i * (uint64_t)i;
    sampleCount++;

    if ((uint32_t)(now - windowStartMs) >= RMS_WINDOW_MS) {
      publishRmsWindow(now);
    }
  }
}

// --------------------------- Protection ------------------------------------
static Fault findFault(const Measurement &m, const Config &c, uint32_t now) {
  if (!m.valid || (uint32_t)(now - m.completedAtMs) > MEASUREMENT_TIMEOUT_MS) {
    return FAULT_SENSOR_INVALID;
  }
  if (!isfinite(m.voltageRms) || !isfinite(m.currentRms)) return FAULT_SENSOR_INVALID;
  if (m.voltageRms < c.vMin) return FAULT_UNDERVOLTAGE;
  if (m.voltageRms > c.vMax) return FAULT_OVERVOLTAGE;
  if (m.currentRms > c.iMax) return FAULT_OVERCURRENT;
  return FAULT_NONE;
}

static bool readingsAreSafe(const Measurement &m, const Config &c, uint32_t now) {
  return findFault(m, c, now) == FAULT_NONE;
}

static void tripFor(Fault why, uint32_t now) {
  relaySet(false);
  if (state != STATE_TRIPPED || fault != why) {
    state = STATE_TRIPPED;
    fault = why;
    stateSinceMs = now;
  }
}

static void protectionTask(void *parameter) {
  (void)parameter;
  
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PROTECTION_PERIOD_MS));
    uint32_t now = millis();
    Measurement m;
    Config c;
    if (!readMeasurementSnapshot(m) || !readConfigSnapshot(c)) {
      relaySet(false);
      continue;
    }

    if ((uint32_t)(now - bootMs) < SENSOR_SETTLE_MS) {
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        relaySet(false);
        state = STATE_STARTUP;
        xSemaphoreGive(stateMutex);
      }
      continue;
    }

    Fault detected = findFault(m, c, now);
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      relaySet(false);
      continue;
    }

    if (detected != FAULT_NONE) {
      tripFor(detected, now);
      xSemaphoreGive(stateMutex);
      continue;
    }

    if (state == STATE_TRIPPED) {
      // A fault is latched. The operator must issue RESET after readings recover
      relaySet(false);
      xSemaphoreGive(stateMutex);
      continue;
    }

    if (state == STATE_STARTUP) {
      state = STATE_WAITING_FOR_SAFE;
      stateSinceMs = now;
      relaySet(false);
      xSemaphoreGive(stateMutex);
      continue;
    }

    if (state == STATE_WAITING_FOR_SAFE) {
      relaySet(false);
      if ((uint32_t)(now - stateSinceMs) >= c.recoveryMs) {
        state = STATE_PROTECTED;
        fault = FAULT_NONE;
        stateSinceMs = now;
        relaySet(true);
      }
      xSemaphoreGive(stateMutex);
      continue;
    }

    if (state == STATE_PROTECTED) {
      fault = FAULT_NONE;
      relaySet(true);
    }
    xSemaphoreGive(stateMutex);
  }
}

// --------------------------- UART commands ---------------------------------
static void printStatus() {
  Measurement m;
  Config c;
  if (!readMeasurementSnapshot(m) || !readConfigSnapshot(c)) {
    printLine("ERR busy; try again");
    return;
  }
  State stateCopy;
  Fault faultCopy;
  bool relayCopy;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) != pdTRUE) { 
    printLine("ERR busy; try again");
    return;
  }
  stateCopy = state; faultCopy = fault; relayCopy = relayCommandedOn;
  xSemaphoreGive(stateMutex);
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }
  Serial.printf("STATE=%s FAULT=%s RELAY_CMD=%s\n", stateName(stateCopy), f
    aultName(faultCopy), relayCopy ? "ON" : "OFF");
  Serial.printf("V_RMS=%.3f I_RMS=%.3f SAMPLES=%lu MEAS_VALID=%s AGE_MS=%lu\n",
    m.voltageRms, m.voltageRms, m.currentRms, (unsigned long)m.sampleCount,
    m.currentRms, (unsigned long)m.sampleCount,
    m.valid ? "YES" : "NO", (unsigned long)(millis() - m.completedAtMs));
  Serial.printf("VMIN=%.3f VMAX=%.3f IMAX=%.3f VSCALE=%.6f ISCALE=%.6f RECOVERY=%lu\n",
    c.vMin, c.vMax, c.iMax, c.vScale, c.iScale, (unsigned long)c.recoveryMs);
  xSemaphoreGive(serialMutex);
}

static void printConfig() {
  Config c;
  if (!readConfigSnapshot(c)) { 
    printLine("ERR busy; try again"); 
    return; 
  }
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }
  Serial.printf("VMIN=%.3f\nVMAX=%.3f\nIMAX=%.3f\nVSCALE=%.6f\nISCALE=%.6f\nRECOVERY=%lu\n",
    c.vMin, c.vMax, c.iMax, c.vScale, c.iScale, (unsigned long)c.recoveryMs);
  xSemaphoreGive(serialMutex);
}

static bool parseFloatStrict(const char *text, float &out) {
  if (!text || !*text) {
    return false;
  }
  char *end = nullptr;
  double value = strtod(text, &end);
  if (end == text || *end != '\0' || !isfinite(value) || value > 1.0e9 || value < -1.0e9) {
    return false;
  }
  out = (float)value;
  return isfinite(out);
}

static bool parseUnsignedStrict(const char *text, uint32_t &out) {
  if (!text || !*text || *text == '-') {
    return false;
  }
  char *end = nullptr;
  unsigned long value = strtoul(text, &end, 10);
  if (end == text || *end != '\0' || value > 3600000UL) {
    return false;
  }
  out = (uint32_t)value;
  return true;
}

static void showHelp() {
  printLine("Commands:");
  printLine("  HELP | STATUS | GET CONFIG");
  printLine("  SET VMIN <value> | SET VMAX <value> | SET IMAX <value>");
  printLine("  SET VSCALE <value> | SET ISCALE <value> | SET RECOVERY <ms>");
  printLine("  SAVE | RESET");
}

static void handleSet(const char *key, const char *valueText) {
  Config candidate;
  if (!readConfigSnapshot(candidate)) { 
    printLine("ERR busy; try again"); 
    return; 
  }

  if (strcmp(key, "RECOVERY") == 0) {
    uint32_t value;
    if (!parseUnsignedStrict(valueText, value) || value < 100UL || value > 3600000UL) {
      printLine("ERR RECOVERY must be 100..3600000 ms"); return;
    }
    candidate.recoveryMs = value;
  } else {
    float value;
    if (!parseFloatStrict(valueText, value) || value <= 0.0f) {
      printLine("ERR value must be a positive number"); return;
    }
    if (strcmp(key, "VMIN") == 0) candidate.vMin = value;
    else if (strcmp(key, "VMAX") == 0) candidate.vMax = value;
    else if (strcmp(key, "IMAX") == 0) candidate.iMax = value;
    else if (strcmp(key, "VSCALE") == 0) candidate.vScale = value;
    else if (strcmp(key, "ISCALE") == 0) candidate.iScale = value;
    else { printLine("ERR unknown setting"); return; }
  }

  candidate.magic = CONFIG_MAGIC;
  candidate.version = CONFIG_VERSION;
  if (!configIsValid(candidate)) {
    printLine("ERR invalid config (check VMIN < VMAX and value ranges)");
    return;
  }
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) != pdTRUE) { printLine("ERR busy; try again"); return; }
  config = candidate;
  xSemaphoreGive(stateMutex);
  printLine("OK setting updated in RAM; use SAVE to persist");
}

static void handleCommand(char *line) {
  // Uppercase the command line for case-insensitive commands and keys
  for (char *p = line; *p; ++p) *p = (char)toupper((unsigned char)*p);

  char *tokens[4] = {nullptr, nullptr, nullptr, nullptr};
  size_t count = 0;
  char *savePtr = nullptr;
  for (char *tok = strtok_r(line, " \t", &savePtr); tok; tok = strtok_r(nullptr, " \t", &savePtr)) {
    if (count >= 4) { printLine("ERR too many arguments"); return; }
    tokens[count++] = tok;
  }
  if (count == 0) return;

  if (strcmp(tokens[0], "HELP") == 0 && count == 1) { showHelp(); return; }
  if (strcmp(tokens[0], "STATUS") == 0 && count == 1) { printStatus(); return; }
  if (strcmp(tokens[0], "GET") == 0 && count == 2 && strcmp(tokens[1], "CONFIG") == 0) { printConfig(); return; }
  if (strcmp(tokens[0], "SAVE") == 0 && count == 1) { saveConfig(); return; }

  if (strcmp(tokens[0], "SET") == 0 && count == 3) {
    handleSet(tokens[1], tokens[2]);
    return;
  }

  if (strcmp(tokens[0], "RESET") == 0 && count == 1) {
    Measurement m;
    Config c;
    uint32_t now = millis();
    if (!readMeasurementSnapshot(m) || !readConfigSnapshot(c)) { printLine("ERR busy; try again"); return; }
    if (!readingsAreSafe(m, c, now)) {
      printLine("ERR cannot reset: readings unsafe or sensor data stale");
      return;
    }
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      state = STATE_WAITING_FOR_SAFE;
      fault = FAULT_NONE;
      stateSinceMs = now;
      xSemaphoreGive(stateMutex);
    }
    relaySet(false);
    printLine("OK fault reset; waiting for recovery interval");
    return;
  }

  printLine("ERR unknown command or invalid arguments; type HELP");
}

static void serialTask(void *parameter) {
  (void)parameter;
  char line[SERIAL_LINE_MAX];
  size_t used = 0;
  uint32_t lastTelemetry = millis();
  printLine("Device Protector ESP32 ready. Type HELP.");

  for (;;) {
    while (Serial.available() > 0) {
      int ch = Serial.read();
      if (ch == '\r') continue;
      if (ch == '\n') {
        line[used] = '\0';
        handleCommand(line);
        used = 0;
      } else if (used < sizeof(line) - 1) {
        line[used++] = (char)ch;
      } else {
        used = 0;
        printLine("ERR command line too long");
      }
    }

    uint32_t now = millis();
    if ((uint32_t)(now - lastTelemetry) >= TELEMETRY_PERIOD_MS) {
      lastTelemetry = now;
      Measurement m;
      if (readMeasurementSnapshot(m)) {
        State stateCopy;
        Fault faultCopy;
        bool relayCopy;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          stateCopy = state; faultCopy = fault; relayCopy = relayCommandedOn;
          xSemaphoreGive(stateMutex);
        } else {
          stateCopy = STATE_TRIPPED; faultCopy = FAULT_SENSOR_INVALID; relayCopy = false;
        }
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          Serial.printf("TELEMETRY,V=%.3f,I=%.3f,VALID=%u,STATE=%s,FAULT=%s,RELAY_CMD=%u\n",
            m.voltageRms, m.currentRms, m.valid ? 1U : 0U,
            stateName(stateCopy), faultName(faultCopy), relayCopy ? 1U : 0U);
          xSemaphoreGive(serialMutex);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// --------------------------- Arduino entry ---------------------------------
void setup() {

  digitalWrite(RELAY_PIN, RELAY_INACTIVE_LEVEL);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_INACTIVE_LEVEL);

  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);
  analogSetPinAttenuation(CURRENT_PIN, ADC_11db);

  stateMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  if (!stateMutex || !serialMutex) {
    // If synch cannot be created, leave the relay off
    relaySet(false);
    while (true) delay(1000);
  }

  loadConfig();
  uint32_t now  = millis();
  bootMs        = now;
  stateSinceMs  = now;
  windowStartMs = now;

  // Separate tasks keep UART activity from directly blocking sensor sampling
  BaseType_t ok1 = xTaskCreate(samplerTask, "sampler", 3072, nullptr, 3, nullptr);
  BaseType_t ok2 = xTaskCreate(protectionTask, "protection", 3072, nullptr, 4, nullptr);
  BaseType_t ok3 = xTaskCreate(serialTask, "serial", 4096, nullptr, 1, nullptr);

  if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS) {
    relaySet(false);
    printLine("FATAL: could not create all tasks; relay disabled");
    while (true) delay(1000);
  }
}

void loop() {
  // Work is performed by FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
