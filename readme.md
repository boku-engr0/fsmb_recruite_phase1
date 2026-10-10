#### Video demo 👇
---
[DRIVE_LINK](https://drive.google.com/file/d/1QdF8cLGoUGWj9jDq1L8lWFXvCwWL2edd/view?usp=sharing)

# Device Protector (Bare-Metal / Arduino Nano)

A proof-of-concept protection relay system written for the **Arduino Nano (ATmega328P)**. It continuously monitors AC voltage and current using the Nano's analog inputs, computes true RMS values across a rolling 200 ms sampling window, and cuts off a load relay whenever measurements go outside safe thresholds.

> **Safety Warning:** This sketch is meant strictly for low-voltage, isolated bench testing (e.g. testing with an isolated step-down transformer, bench supply, or signal generator). **Do not hook raw mains directly to an Arduino.**

---

## Hardware & Pinout

| Pin | Function | Notes |
|---|---|---|
| `A0` | Voltage Sense | AC signal biased around ~2.5 V (or scaled DC). |
| `A1` | Current Sense | AC signal from a current sensor (e.g. ACS712 or CT with bias circuit) centered at ~2.5 V. |
| `D8` | Relay Output | Active-LOW driver by default (`RELAY_ACTIVE_LEVEL = LOW`). Driven LOW to energize the relay. |
| `UART` | Serial Console | 115200 baud, newline line ending. |

---

## How It Works

### 1. AC RMS Calculation
The sketch samples both `A0` and `A1` approximately once every 1 ms (`SAMPLE_INTERVAL_MS = 1`). Over a 200 ms window (which covers 10 full cycles at 50 Hz or 12 cycles at 60 Hz), it accumulates sample count, sum, and sum of squares.

At the end of each window, it computes the sample variance:

$$\text{Var} = \frac{\sum x^2}{N} - \left(\frac{\sum x}{N}\right)^2$$

The AC RMS in ADC counts is $\sqrt{\text{Var}}$. This approach strips off the DC midpoint bias without needing computationally heavy software filtering inside the 1 kHz sampling loop. The resulting RMS counts are converted to engineering units using user-configurable scale factors:

- `voltageRms = voltageCountsRms * cfg.vScale`
- `currentRms = currentCountsRms * cfg.iScale`

### 2. State Machine & Protection Logic
The protector runs through four states:

1. **`STARTUP`**: Held for `SENSOR_SETTLE_MS` (1 second) after boot to let analog sensors and power rails stabilize. The relay is kept OFF.
2. **`WAIT_SAFE`**: Readings are valid and within bounds, but the firmware waits for a continuous safe period (`cfg.recoveryMs`, default 3000 ms) before engaging the relay.
3. **`PROTECTED`**: The relay is closed (load connected). Continuous monitoring evaluates every completed 200 ms RMS window.
4. **`TRIPPED`**: Relay is immediately de-energized if a fault occurs:
   - **`UNDERVOLTAGE`**: `voltageRms < cfg.vMin`
   - **`OVERVOLTAGE`**: `voltageRms > cfg.vMax`
   - **`OVERCURRENT`**: `currentRms > cfg.iMax`
   - **`SENSOR_INVALID`**: Flatlined voltage signal (`voltageCountsRms < 2.0`) or invalid readings.

The trip is **latched**. Even if the voltage or current returns to normal, the relay stays disconnected until an operator sends a `RESET` command over serial.

---

## Serial Console & Commands

Connect using any serial terminal at **115200 baud** with newline (`\n`) line ending.

The sketch outputs telemetry every 1 second:
```text
STATUS,state=PROTECTED,fault=NONE,relay=ON,vrms=12.015,irms=0.230,vcounts=120.15,icounts=23.00,vmin=9.00,vmax=15.00,imax=0.500
```

### Supported Commands
- `HELP` – Shows command summary.
- `STATUS` – Prints immediate state, fault code, relay status, and sensor readings.
- `GET CONFIG` – Prints current configuration (limits, scale factors, recovery delay).
- `SET VMIN <volts>` – Sets minimum voltage threshold (must be $\ge 0$ and $< V_{max}$).
- `SET VMAX <volts>` – Sets maximum voltage threshold (must be $> V_{min}$).
- `SET IMAX <amps>` – Sets maximum current trip limit.
- `SET VSCALE <scale>` – Volts per ADC RMS count multiplier.
- `SET ISCALE <scale>` – Amps per ADC RMS count multiplier.
- `SET RECOVERY <ms>` – Recovery delay in milliseconds (500 to 60000 ms).
- `SAVE` – Writes current configuration to internal EEPROM (validated via magic number `0x51A7`).
- `RESET` – Clears a latched trip state. The command will be rejected if current readings are still out of spec.

> **Note:** Changes made via `SET` take effect immediately in RAM, but won't survive a reboot unless you send `SAVE`.

---

## Calibration

The firmware ships with default thresholds suitable for low-voltage testing:
- $V_{min} = 9.0\,\text{V}$, $V_{max} = 15.0\,\text{V}$, $I_{max} = 0.50\,\text{A}$
- $V_{scale} = 0.0100$, $I_{scale} = 0.0100$

Because sensor dividers and modules vary, calibrate scales against a known reference meter:
1. Apply a known test voltage or load.
2. Check `vcounts` or `icounts` in the `STATUS` output.
3. Calculate: $\text{scale} = \frac{\text{True RMS Measurement}}{\text{counts}}$.
4. Apply using `SET VSCALE <value>` or `SET ISCALE <value>`, verify, then `SAVE`.

---

## Getting Started

1. Open `src/device_protector_baremetal.ino` in Arduino IDE.
2. Under **Tools > Board**, select **Arduino Nano**.
3. Under **Tools > Processor**, select **ATmega328P** (or **ATmega328P (Old Bootloader)** if using standard CH340 clones).
4. Select the serial port and flash the board.
5. Open Serial Monitor at **115200 baud** to see startup messages and status output.
