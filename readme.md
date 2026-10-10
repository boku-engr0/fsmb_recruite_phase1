# Device Protector (FreeRTOS / ESP32)

A FreeRTOS-based protection relay proof-of-concept targeting the **ESP32 DevKit** (Arduino-ESP32 core).

Compared to the single-loop bare-metal version, this implementation decouples high-speed ADC sampling, fault evaluation, and serial console communication into dedicated preemptive FreeRTOS tasks. Serial activity or telemetry formatting never delays ADC sampling or slows down relay trip response.

> **Safety Warning:** This project is intended strictly for isolated bench-testing (e.g. signal generators, isolated step-down transformers, or low-voltage AC/DC supplies). **Never connect raw mains directly to an ESP32 or breadboard.**

---

## Hardware & Pinout

| Pin | Function | Notes |
|---|---|---|
| `GPIO34` | Voltage Sense | ADC1 channel (input-only pin). AC signal biased to ~1.65 V mid-rail. |
| `GPIO35` | Current Sense | ADC1 channel (input-only pin). AC signal from Hall/CT sensor biased to ~1.65 V. |
| `GPIO26` | Relay Output | Active-LOW driver by default (`RELAY_ACTIVE_LEVEL = LOW`). Pin is driven LOW to turn the relay ON. |
| `UART0` | Serial Console | 115200 baud, newline line ending. |

> **Why ADC1?** `GPIO34` and `GPIO35` belong to ADC1. ADC2 pins cannot be used when Wi-Fi is active on the ESP32. Even though Wi-Fi is unused here, sticking to ADC1 keeps things portable if network features are added later.

---

## FreeRTOS Task Architecture

The sketch creates three tasks and coordinates them using mutexes:

```text
+-------------------------------------------------------------+
| samplerTask (Priority 3, 1 kHz)                             |
| - Samples GPIO34 & GPIO35 every 1 ms                        |
| - Accumulates 64-bit sums over 200 ms RMS window            |
| - Writes latest measurement to shared struct under mutex    |
+-------------------------------------------------------------+
                               |
                        [stateMutex]
                               |
                               v
+-------------------------------------------------------------+
| protectionTask (Priority 4, 20 ms period)                   |
| - Highest application priority                              |
| - Checks RMS limits & stale-data timeout (< 500 ms)         |
| - Actuates relay GPIO immediately on fault                  |
| - Manages STARTUP, WAITING_FOR_SAFE, PROTECTED, TRIPPED     |
+-------------------------------------------------------------+

+-------------------------------------------------------------+
| serialTask (Priority 1, background)                         |
| - Non-blocking UART line buffering & command parsing        |
| - Prints telemetry every 1 second                           |
| - Reads/writes non-volatile config via Preferences (NVS)    |
| - Synchronized via serialMutex                              |
+-------------------------------------------------------------+
```

### Shared State & Synchronization
- **`stateMutex`**: Protects the active `Config`, the latest `Measurement` snapshot, and the FSM `State` / `Fault` variables across tasks.
- **`serialMutex`**: Protects UART output so periodic telemetry and command responses don't interleave.

---

## How It Works

### 1. Sampling and RMS Calculation
- The ESP32 ADC runs at 12-bit resolution (`0..4095`) with 11 dB attenuation. In software, values are scaled by `(raw + 2) / 4` down to a nominal `0..1023` count range.
- `samplerTask` uses `vTaskDelayUntil` to sample at a steady 1 kHz.
- Over each 200 ms window (10 cycles at 50 Hz), it computes sample variance:

$$\text{Var} = \frac{\sum x^2}{N} - \left(\frac{\sum x}{N}\right)^2$$

- $\text{RMS Counts} = \sqrt{\text{Var}}$, which inherently removes the DC midpoint offset.
- Real RMS values are calculated using calibrated multipliers:
  - $\text{Voltage}_{\text{RMS}} = \text{vCounts}_{\text{RMS}} \times \text{vScale}$
  - $\text{Current}_{\text{RMS}} = \text{iCounts}_{\text{RMS}} \times \text{iScale}$

### 2. Protection FSM
- **`STARTUP`**: Waits 1000 ms (`SENSOR_SETTLE_MS`) after boot for analog front-ends to stabilize. Relay remains OFF.
- **`WAITING_FOR_SAFE`**: Requires readings to remain safely within limits for `recoveryMs` (default 3000 ms) before energizing the relay.
- **`PROTECTED`**: Normal operation. Relay is closed and power is supplied to the load.
- **`TRIPPED`**: Latched shutdown. Any out-of-bounds reading immediately turns the relay OFF:
  - **`UNDERVOLTAGE`**: `voltageRms < vMin`
  - **`OVERVOLTAGE`**: `voltageRms > vMax`
  - **`OVERCURRENT`**: `currentRms > iMax`
  - **`SENSOR_INVALID`**: Non-finite readings or measurement older than 500 ms (`MEASUREMENT_TIMEOUT_MS`).

Once tripped, the relay **stays OFF**. Recovery requires an operator `RESET` command over serial once conditions are verified safe.

---

## Serial Console & Commands

Connect at **115200 baud** (newline line ending).

### Periodic Telemetry (Every 1s)
```text
TELEMETRY,V=230.120,I=1.450,VALID=1,STATE=PROTECTED,FAULT=NONE,RELAY_CMD=1
```

### Interactive Commands
- `HELP` – Lists available commands.
- `STATUS` – Detailed status dump including state, active fault, RMS readings, sample count, measurement age, and active thresholds.
- `GET CONFIG` – Prints current configuration parameters.
- `SET VMIN <value>` – Sets minimum voltage threshold (volts).
- `SET VMAX <value>` – Sets maximum voltage threshold (volts).
- `SET IMAX <value>` – Sets maximum current trip limit (amps).
- `SET VSCALE <value>` – Multiplier for voltage calibration.
- `SET ISCALE <value>` – Multiplier for current calibration.
- `SET RECOVERY <ms>` – Recovery wait delay (100 to 3600000 ms).
- `SAVE` – Persists active RAM configuration into ESP32 NVS using the `Preferences` library (namespace `"protector"`).
- `RESET` – Clears a latched trip state. Reconnects the load only if readings are safe and fresh.

---

## Configuration & Calibration

Default thresholds in this sketch:
- $V_{min} = 180.0\,\text{V}$
- $V_{max} = 250.0\,\text{V}$
- $I_{max} = 10.0\,\text{A}$
- Scales: `VSCALE = 1.0`, `ISCALE = 1.0`
- Recovery: `3000 ms`

Because sensor divider ratios, op-amp conditioning circuits, and ESP32 ADC characteristics vary, calibrate scale factors with an external multimeter:
1. Apply a known test voltage and current.
2. Check unscaled RMS or compute:
   $$\text{scale} = \frac{\text{Meter Reading (RMS)}}{\text{Unscaled RMS}}$$
3. Update settings via `SET VSCALE <val>` / `SET ISCALE <val>`, then save with `SAVE`.

---

## Building and Flashing

1. Install the ESP32 board package in Arduino IDE (**Boards Manager** -> search for `esp32` by Espressif).
2. Select Board: **ESP32 Dev Module** (or your specific dev board).
3. Open `src/device_protector_rtos.ino`.
4. Upload speed: **921600** (or default 115200).
5. Open Serial Monitor at **115200 baud**.
