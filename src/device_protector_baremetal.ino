/* Device Protector - Arduino Nano proof of concept */

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// ----------------------------- Pin configuration ----------------------------

static const uint8_t PIN_VOLTAGE = A0;
static const uint8_t PIN_CURRENT = A1;
static const uint8_t PIN_RELAY   = 8;

// Change this to HIGH for an active-HIGH driver
static const uint8_t RELAY_ACTIVE_LEVEL = LOW;
static const uint8_t RELAY_INACTIVE_LEVEL =
    (RELAY_ACTIVE_LEVEL == LOW) ? HIGH : LOW;

// ----------------------------- Sampling setup -------------------------------

// Sample each sensor about once per millisecond. A 100 ms window covers 5 cycles
// of a 50 Hz signal. The actual interval can vary slightly due to serial handling.

static const uint16_t SAMPLE_INTERVAL_MS = 1;
static const uint16_t RMS_WINDOW_MS = 200;
static const uint16_t TELEMETRY_INTERVAL_MS = 1000;
static const uint16_t SENSOR_SETTLE_MS = 1000;
static const uint16_t MAX_LINE_LENGTH = 80;


// ----------------------------- Configuration --------------------------------

struct Config {
  // EEPROM validity marker
  uint16_t magic;
  uint8_t version;

  float vMin;
  float vMax;
  float iMax;

  // Set by calibration
  float vScale;
  float iScale;

  uint16_t recoveryMs;
};

static const uint16_t CONFIG_MAGIC = 0x51A7;
static const uint8_t CONFIG_VERSION = 1;
static const int EEPROM_ADDRESS = 0;

Config cfg;

// Calibrate scales and choose limits appropriate for actual test source/load
static const Config DEFAULT_CONFIG = {
  CONFIG_MAGIC,
  CONFIG_VERSION,

  9.0f,
  15.0f,
  0.50f,

  0.0100f,
  0.0100f,
  
  3000
};

// ----------------------------- State and faults -----------------------------

enum DeviceState : uint8_t {
  STATE_STARTUP,
  STATE_WAITING_FOR_SAFE,
  STATE_PROTECTED,
  STATE_TRIPPED
};

enum FaultCode : uint8_t {
  FAULT_NONE,
  FAULT_UNDERVOLTAGE,
  FAULT_OVERVOLTAGE,
  FAULT_OVERCURRENT,
  FAULT_SENSOR_INVALID
};

DeviceState state = STATE_STARTUP;
FaultCode fault = FAULT_NONE;

float voltageRms = 0.0f;
float currentRms = 0.0f;
float voltageCountsRms = 0.0f;
float currentCountsRms = 0.0f;

bool haveMeasurement = false;
bool relayConnected = false;

uint32_t bootMs = 0;
uint32_t lastSampleMs = 0;
uint32_t windowStartMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t safeSinceMs = 0;
bool safeTimerRunning = false;

// Accumulate mean and variance for each channel
uint32_t vSum = 0;
uint32_t iSum = 0;
uint32_t vSqSum = 0;
uint32_t iSqSum = 0;
uint16_t sampleCount = 0;

char commandLine[MAX_LINE_LENGTH + 1];
uint8_t commandLength = 0;

// ----------------------------- Utility functions ----------------------------

const __FlashStringHelper *stateName(DeviceState s) {
  switch (s) {
    case STATE_STARTUP:         return F("STARTUP");
    case STATE_WAITING_FOR_SAFE:return F("WAIT_SAFE");
    case STATE_PROTECTED:       return F("PROTECTED");
    case STATE_TRIPPED:         return F("TRIPPED");
    default:                    return F("UNKNOWN");
  }
}

const __FlashStringHelper *faultName(FaultCode f) {
  switch (f) {
    case FAULT_NONE:            return F("NONE");
    case FAULT_UNDERVOLTAGE:    return F("UNDERVOLTAGE");
    case FAULT_OVERVOLTAGE:     return F("OVERVOLTAGE");
    case FAULT_OVERCURRENT:     return F("OVERCURRENT");
    case FAULT_SENSOR_INVALID:  return F("SENSOR_INVALID");
    default:                    return F("UNKNOWN");
  }
}

void setRelayConnected(bool connected) {
  relayConnected = connected;
  digitalWrite(PIN_RELAY, connected ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
}

bool configIsValid(const Config &c) {
  if (c.magic != CONFIG_MAGIC || c.version != CONFIG_VERSION) return false;
  if (!isfinite(c.vMin) || !isfinite(c.vMax) || !isfinite(c.iMax) ||
      !isfinite(c.vScale) || !isfinite(c.iScale)) return false;

  if (c.vMin < 0.0f || c.vMax <= c.vMin || c.vMax > 1000.0f) return false;
  if (c.iMax <= 0.0f || c.iMax > 100.0f) return false;
  if (c.vScale <= 0.0f || c.vScale > 100.0f) return false;
  if (c.iScale <= 0.0f || c.iScale > 100.0f) return false;
  if (c.recoveryMs < 500 || c.recoveryMs > 60000) return false;
  return true;
}

void loadConfig() {
  Config stored;
  EEPROM.get(EEPROM_ADDRESS, stored);
  if (configIsValid(stored)) {
    cfg = stored;
    Serial.println(F("CONFIG loaded from EEPROM"));
  } else {
    cfg = DEFAULT_CONFIG;
    cfg.magic = CONFIG_MAGIC;
    cfg.version = CONFIG_VERSION;
    Serial.println(F("CONFIG defaults loaded; calibrate sensor scales"));
  }
}

void saveConfig() {
  cfg.magic = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  EEPROM.put(EEPROM_ADDRESS, cfg);
  Serial.println(F("OK config saved to EEPROM"));
}

void printConfig() {
  Serial.println(F("CONFIG BEGIN"));
  Serial.print(F("VMIN=")); Serial.println(cfg.vMin, 4);
  Serial.print(F("VMAX=")); Serial.println(cfg.vMax, 4);
  Serial.print(F("IMAX=")); Serial.println(cfg.iMax, 4);
  Serial.print(F("VSCALE=")); Serial.println(cfg.vScale, 6);
  Serial.print(F("ISCALE=")); Serial.println(cfg.iScale, 6);
  Serial.print(F("RECOVERY_MS=")); Serial.println(cfg.recoveryMs);
  Serial.println(F("CONFIG END"));
}

void printStatus() {
  Serial.print(F("STATUS,state=")); Serial.print(stateName(state));
  Serial.print(F(",fault=")); Serial.print(faultName(fault));
  Serial.print(F(",relay=")); Serial.print(relayConnected ? F("ON") : F("OFF"));
  Serial.print(F(",vrms=")); Serial.print(voltageRms, 3);
  Serial.print(F(",irms=")); Serial.print(currentRms, 3);
  Serial.print(F(",vcounts=")); Serial.print(voltageCountsRms, 2);
  Serial.print(F(",icounts=")); Serial.print(currentCountsRms, 2);
  Serial.print(F(",vmin=")); Serial.print(cfg.vMin, 2);
  Serial.print(F(",vmax=")); Serial.print(cfg.vMax, 2);
  Serial.print(F(",imax=")); Serial.print(cfg.iMax, 3);
  Serial.println();
}

void printHelp() {
  Serial.println(F("Commands (115200 baud, newline ending):"));
  Serial.println(F("  HELP"));
  Serial.println(F("  STATUS"));
  Serial.println(F("  GET CONFIG"));
  Serial.println(F("  SET VMIN <volts>"));
  Serial.println(F("  SET VMAX <volts>"));
  Serial.println(F("  SET IMAX <amps>"));
  Serial.println(F("  SET VSCALE <volts_per_ADC_RMS_count>"));
  Serial.println(F("  SET ISCALE <amps_per_ADC_RMS_count>"));
  Serial.println(F("  SET RECOVERY <milliseconds 500..60000>"));
  Serial.println(F("  SAVE"));
  Serial.println(F("  RESET  (only accepted when readings are safe)"));
  Serial.println(F("Scale factors require calibration against a trusted meter."));
}

bool parseFloatStrict(const char *s, float &out) {
  if (s == NULL || *s == '\0') return false;
  char *endPtr = NULL;
  out = strtod(s, &endPtr);
  if (endPtr == s || *endPtr != '\0' || !isfinite(out)) return false;
  return true;
}

bool parseUnsignedStrict(const char *s, uint32_t &out) {
  if (s == NULL || *s == '\0' || *s == '-') return false;
  char *endPtr = NULL;
  unsigned long value = strtoul(s, &endPtr, 10);
  if (endPtr == s || *endPtr != '\0') return false;
  out = (uint32_t)value;
  return true;
}

void uppercaseInPlace(char *s) {
  while (*s) {
    *s = (char)toupper((unsigned char)*s);
    ++s;
  }
}

bool readingsAreSafe() {
  if (!haveMeasurement) return false;
  if (!isfinite(voltageRms) || !isfinite(currentRms)) return false;
  // Near-zero current is normal when no load is connected; do not reject it.
  if (voltageCountsRms < 2.0f) return false;
  if (voltageRms < cfg.vMin || voltageRms > cfg.vMax) return false;
  if (currentRms > cfg.iMax) return false;
  return true;
}

void trip(FaultCode reason) {
  setRelayConnected(false);
  fault = reason;
  state = STATE_TRIPPED;
  safeTimerRunning = false;
  Serial.print(F("FAULT,reason="));
  Serial.println(faultName(reason));
}

FaultCode evaluateFault() {
  if (!haveMeasurement) return FAULT_SENSOR_INVALID;

  // Reject a near-flat voltage channel. Near-zero current is valid with no load,
  // so this software cannot reliably detect a disconnected ACS712 from RMS alone.

  if (!isfinite(voltageCountsRms) || !isfinite(currentCountsRms) ||
      voltageCountsRms < 2.0f) {
    return FAULT_SENSOR_INVALID;
  }

  if (voltageRms < cfg.vMin) return FAULT_UNDERVOLTAGE;
  if (voltageRms > cfg.vMax) return FAULT_OVERVOLTAGE;
  if (currentRms > cfg.iMax) return FAULT_OVERCURRENT;
  return FAULT_NONE;
}

// ----------------------------- RMS acquisition ------------------------------

// Each ADC input is assumed to be a biased waveform centered somewhere between
// 0 and 5 V. RMS AC magnitude is computed after subtracting the measured mean.

void sampleSensors() {
  const uint16_t v = (uint16_t)analogRead(PIN_VOLTAGE);
  const uint16_t i = (uint16_t)analogRead(PIN_CURRENT);

  vSum += v;
  iSum += i;
  vSqSum += (uint32_t)v * (uint32_t)v;
  iSqSum += (uint32_t)i * (uint32_t)i;
  if (sampleCount < 65535) sampleCount++;
}

void resetSampleWindow(uint32_t now) {
  vSum = 0;
  iSum = 0;
  vSqSum = 0;
  iSqSum = 0;
  sampleCount = 0;
  windowStartMs = now;
}

void finishSampleWindow() {
  if (sampleCount < 10) {
    haveMeasurement = false;
    return;
  }

  const float n = (float)sampleCount;
  const float vMean = (float)vSum / n;
  const float iMean = (float)iSum / n;
  float vVariance = ((float)vSqSum / n) - (vMean * vMean);
  float iVariance = ((float)iSqSum / n) - (iMean * iMean);

  // Floating-point roundoff can make a near-zero variance slightly negative.
  if (vVariance < 0.0f) vVariance = 0.0f;
  if (iVariance < 0.0f) iVariance = 0.0f;

  voltageCountsRms = sqrt(vVariance);
  currentCountsRms = sqrt(iVariance);
  voltageRms = voltageCountsRms * cfg.vScale;
  currentRms = currentCountsRms * cfg.iScale;
  haveMeasurement = true;
}

// ----------------------------- Protection FSM -------------------------------

void updateProtection(uint32_t now) {
  if ((uint32_t)(now - bootMs) < SENSOR_SETTLE_MS || !haveMeasurement) {
    setRelayConnected(false);
    if (state != STATE_TRIPPED) state = STATE_STARTUP;
    return;
  }

  // If already tripped, remain tripped until an explicit RESET command.
  if (state == STATE_TRIPPED) {
    setRelayConnected(false);
    return;
  }

  const FaultCode currentFault = evaluateFault();
  if (currentFault != FAULT_NONE) {
    trip(currentFault);
    return;
  }

  if (state == STATE_STARTUP) {
    state = STATE_WAITING_FOR_SAFE;
    safeSinceMs = now;
    safeTimerRunning = true;
  }

  if (state == STATE_WAITING_FOR_SAFE) {
    if (!safeTimerRunning) {
      safeSinceMs = now;
      safeTimerRunning = true;
    }
    if ((uint32_t)(now - safeSinceMs) >= cfg.recoveryMs) {
      state = STATE_PROTECTED;
      fault = FAULT_NONE;
      setRelayConnected(true);
      Serial.println(F("EVENT,load_connected"));
    } else {
      setRelayConnected(false);
    }
  } else if (state == STATE_PROTECTED) {
    setRelayConnected(true);
  }
}

// ----------------------------- Command parser -------------------------------

void handleCommand(char *line) {
  while (*line && isspace((unsigned char)*line)) line++;
  if (*line == '\0') return;

  // Split into at most 3 tokens. Commands are case-insensitive
  char *token1 = strtok(line, " \t");
  char *token2 = strtok(NULL, " \t");
  char *token3 = strtok(NULL, " \t");
  char *extra  = strtok(NULL, " \t");

  if (!token1) return;
  uppercaseInPlace(token1);
  if (token2) uppercaseInPlace(token2);

  if (strcmp(token1, "HELP") == 0) {
    printHelp();
    return;
  }
  if (strcmp(token1, "STATUS") == 0) {
    printStatus();
    return;
  }
  if (strcmp(token1, "GET") == 0 && token2 && strcmp(token2, "CONFIG") == 0 && !token3) {
    printConfig();
    return;
  }
  if (strcmp(token1, "SAVE") == 0 && !token2) {
    saveConfig();
    return;
  }
  if (strcmp(token1, "RESET") == 0 && !token2) {
    if (state != STATE_TRIPPED) {
      Serial.println(F("ERR no_latched_fault"));
    } else if (readingsAreSafe()) {
      fault = FAULT_NONE;
      state = STATE_WAITING_FOR_SAFE;
      setRelayConnected(false);
      safeSinceMs = millis();
      safeTimerRunning = true;
      Serial.println(F("OK fault_reset; waiting for recovery interval"));
    } else {
      Serial.println(F("ERR readings_not_safe; relay remains OFF"));
    }
    return;
  }

  if (strcmp(token1, "SET") == 0 && token2 && token3 && !extra) {
    float value = 0.0f;
    uint32_t uvalue = 0;

    if (strcmp(token2, "RECOVERY") == 0) {
      if (!parseUnsignedStrict(token3, uvalue) || uvalue < 500 || uvalue > 60000) {
        Serial.println(F("ERR recovery_range_500_to_60000_ms"));
        return;
      }
      cfg.recoveryMs = (uint16_t)uvalue;
    } else {
      if (!parseFloatStrict(token3, value)) {
        Serial.println(F("ERR invalid_number"));
        return;
      }

      if (strcmp(token2, "VMIN") == 0) {
        if (value < 0.0f || value >= cfg.vMax || value > 1000.0f) {
          Serial.println(F("ERR VMIN_must_be_nonnegative_and_less_than_VMAX"));
          return;
        }
        cfg.vMin = value;
      } else if (strcmp(token2, "VMAX") == 0) {
        if (value <= cfg.vMin || value > 1000.0f) {
          Serial.println(F("ERR VMAX_must_be_greater_than_VMIN_and_at_most_1000"));
          return;
        }
        cfg.vMax = value;
      } else if (strcmp(token2, "IMAX") == 0) {
        if (value <= 0.0f || value > 100.0f) {
          Serial.println(F("ERR IMAX_range_0_to_100_A"));
          return;
        }
        cfg.iMax = value;
      } else if (strcmp(token2, "VSCALE") == 0) {
        if (value <= 0.0f || value > 100.0f) {
          Serial.println(F("ERR VSCALE_range_0_to_100"));
          return;
        }
        cfg.vScale = value;
      } else if (strcmp(token2, "ISCALE") == 0) {
        if (value <= 0.0f || value > 100.0f) {
          Serial.println(F("ERR ISCALE_range_0_to_100"));
          return;
        }
        cfg.iScale = value;
      } else {
        Serial.println(F("ERR unknown_setting"));
        return;
      }
    }

    Serial.println(F("OK setting_updated_not_saved; send SAVE to persist"));

    // Apply changed limits immediately. If protected and the new limits make the
    // present readings unsafe, trip without waiting for another full window.

    if (state == STATE_PROTECTED && haveMeasurement) {
      FaultCode f = evaluateFault();
      if (f != FAULT_NONE) trip(f);
    }
    printConfig();
    return;
  }

  Serial.println(F("ERR unknown_or_malformed_command; send HELP"));
}

void serviceSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (commandLength > 0) {
        commandLine[commandLength] = '\0';
        handleCommand(commandLine);
        commandLength = 0;
      }
    } else if (c == 8 || c == 127) {
      if (commandLength > 0) commandLength--;
    } else if ((unsigned char)c >= 32 && commandLength < MAX_LINE_LENGTH) {
      commandLine[commandLength++] = c;
    } else if (commandLength >= MAX_LINE_LENGTH) {
      commandLength = 0;
      Serial.println(F("ERR command_too_long"));
    }
  }
}

// --------------------------------- Arduino -----------------------------------

void setup() {
  // Set the output latch to OFF before enabling output mode to reduce startup glitches.
  digitalWrite(PIN_RELAY, RELAY_INACTIVE_LEVEL);
  pinMode(PIN_RELAY, OUTPUT);
  setRelayConnected(false);

  pinMode(PIN_VOLTAGE, INPUT);
  pinMode(PIN_CURRENT, INPUT);

  Serial.begin(115200);
  bootMs = millis();
  lastSampleMs = bootMs;
  windowStartMs = bootMs;
  lastTelemetryMs = bootMs;

  Serial.println();
  Serial.println(F("Device Protector - Arduino Nano PoC"));
  Serial.println(F("SAFETY: low-voltage isolated test setup only; never connect to mains."));
  loadConfig();
  printHelp();
  printConfig();

  state = STATE_STARTUP;
}

void loop() {
  const uint32_t now = millis();

  // Keep serial responsive while sampling. Each call reads both ADC channels.
  if ((uint32_t)(now - lastSampleMs) >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    sampleSensors();
  }

  if ((uint32_t)(now - windowStartMs) >= RMS_WINDOW_MS) {
    finishSampleWindow();
    resetSampleWindow(now);
    updateProtection(now);
  }

  serviceSerial();

  if ((uint32_t)(now - lastTelemetryMs) >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    printStatus();
  }
}
