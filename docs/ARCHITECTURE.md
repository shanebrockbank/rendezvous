# Software Architecture

## Overview

The firmware is a standard ESP-IDF application using FreeRTOS tasks. `app_main()` performs sequential hardware init, creates the telemetry mutex, starts ESP-NOW, then calls `tasks_start()` which creates all FreeRTOS tasks according to `ACTIVE_MILESTONE`.

```
app_main()
 ├── bsp_init()                   Hardware bringup (NVS, I2C×2, UART, GPIO)
 ├── icm20948_init()              [M3.5+] IMU init
 ├── lcd_init()                   [M2+]   OLED init
 ├── xSemaphoreCreateMutex()      Shared telemetry mutex
 ├── espnow_comms_init()          WiFi + ESP-NOW
 ├── gps_nmea_init()              GPS UART config + baud scan
 └── tasks_start()                Create FreeRTOS tasks
```

---

## Components

### `bsp` — Board Support Package

Initialises all hardware peripherals. Called once before anything else.

- Initialises NVS flash (required by ESP-NOW)
- Configures I2C bus 0 (OLED, GPIO21/22, 100 kHz)
- Configures I2C bus 1 (IMU, GPIO18/19, 100 kHz)
- Configures UART2 (GPS, GPIO16/17)
- Configures GPIO for LEDs (GPIO2, GPIO4) and buttons (GPIO0, GPIO13)

Key file: `bsp.c`, `board_config.h`

---

### `espnow_comms` — Wireless Communication

Handles all ESP-NOW TX and RX.

**Telemetry packet** (`telemetry_packet_t`, 250-byte limit):

| Field          | Type      | Description                               |
|----------------|-----------|-------------------------------------------|
| `node_id`      | `uint8_t` | 0 = Chaser, 1 = Target                    |
| `pitch`        | `float`   | Pitch angle in degrees (from ICM-20948)   |
| `roll`         | `float`   | Roll angle in degrees                     |
| `heading`      | `float`   | Tilt-compensated heading 0–360°           |
| `lat`          | `double`  | GPS latitude (0.0 until fix)              |
| `lon`          | `double`  | GPS longitude                             |
| `timestamp_ms` | `uint32_t`| Sender's uptime in ms                     |
| `seq`          | `uint32_t`| Packet sequence number                    |
| `button_state` | `uint8_t` | 1 if BOOT button currently pressed        |

The struct is `__attribute__((packed))` to ensure consistent wire layout across builds.

**TX task** (`espnow_tx_task`, Core 0, Priority 5): broadcasts `g_local` every 200 ms, plus an immediate extra send on BOOT button edge.

**RX callback** (`recv_cb`): called in WiFi task context; copies data into `g_remote`, captures RSSI into `g_remote_rssi`, latches timestamps.

**Shared globals** (defined in `main.c`, extern'd by `espnow_comms.h`):

| Variable          | Type                    | Description                      |
|-------------------|-------------------------|----------------------------------|
| `g_local`         | `telemetry_packet_t`    | This node's outgoing telemetry   |
| `g_remote`        | `telemetry_packet_t`    | Last received remote telemetry   |
| `g_telem_mutex`   | `SemaphoreHandle_t`     | Mutex protecting both structs    |
| `g_remote_rx_ms`  | `volatile uint32_t`     | Local uptime at last RX          |
| `g_remote_btn_ms` | `volatile uint32_t`     | Latched remote button press time |
| `g_remote_rssi`   | `volatile int8_t`       | RSSI of last received packet     |

---

### `icm20948` — IMU Driver

Drives the ICM-20948 9-DOF IMU plus its internal AK09916 magnetometer.

**Init sequence:**
1. Auto-probe I2C address 0x68, then 0x69.
2. Soft reset (100 ms delay — critical for reliable init over long wire runs).
3. Configure accelerometer (±2g, 1.125 kHz ODR) and gyroscope.
4. Enable AK09916 via bypass mode (more reliable than indirect SLV4 writes):
   - Disable ICM I2C master, enable `BYPASS_EN`.
   - Write AK09916 `CNTL2 = 0x08` (continuous 100 Hz).
   - Re-enable ICM I2C master, configure SLV0 for 9-byte reads (ST1 + HX..HZ + RSVA + ST2).
5. Configure ICM I2C master for automatic mag sampling.

**Read (`icm20948_read`):**

1. Read accelerometer XYZ and magnetometer XYZ from ICM registers.
2. If `ICM_MOUNT_INVERTED = 1`, negate `az` and `bz`.
3. Compute pitch and roll via `atan2`:
   ```
   pitch = atan2(-ax, sqrt(ay² + az²))
   roll  = atan2(ay, az)
   ```
4. Apply EMA smoothing (α = 0.3, ~140 ms time constant at 20 Hz).
5. Subtract static offsets (`ICM_PITCH_OFFSET_DEG`, `ICM_ROLL_OFFSET_DEG`).
6. Compute tilt-compensated heading:
   ```
   Xh = Bx·cos(pitch) + Bz·sin(pitch)
   Yh = Bx·sin(roll)·sin(pitch) + By·cos(roll) - Bz·sin(roll)·cos(pitch)
   heading = atan2(Yh, Xh)
   ```
7. Apply circular EMA on heading (sin/cos domain, α = 0.3, wrap-safe).

**IMU task** (`imu_task`, Core 0, Priority 5): reads ICM every 50 ms, updates `g_local.pitch/roll/heading` under mutex.

---

### `gps_nmea` — GPS Parser

Parses NMEA 0183 `$GPRMC` and `$GNRMC` sentences from a u-blox M8N over UART2.

- **Baud auto-scan** at init: tries 9600, 4800, 19200, 38400, 57600, 115200 — handles cheap clones at non-default rates.
- Mutex-protected `gps_get_fix()` API returns lat, lon, and validity flag.
- Logs a warning every 5 s if no valid NMEA sentence has been received (useful for detecting wiring or cold-start issues).
- Logs once on first GPS fix.

---

### `distance_estimator` — GPS + RSSI Fusion

Produces a single distance estimate by blending GPS haversine and RSSI log-distance.

**Update inputs** (called every 500 ms from `gps_copy_task`):
- Local lat/lon
- Remote lat/lon (from `g_remote`)
- GPS validity flag (both nodes must have a fix)
- Latest RSSI from `g_remote_rssi`

**RSSI distance model:**
```
d = 10 ^ ((RSSI_REF_1M - rssi) / (10 × n))
```
with `n = 2.0` open air.

**Fusion weights** (based on fused distance, not raw GPS/RSSI separately):

| Zone          | GPS weight | RSSI weight |
|---------------|------------|-------------|
| > 20 m        | 1.0        | 0.0         |
| 2–20 m        | linear     | linear      |
| < 2 m         | 0.0        | 1.0         |

**Signal smoothing within the estimator:**

| Signal              | Filter              | Parameter              |
|---------------------|---------------------|------------------------|
| RSSI                | 3-sample ring-buffer median | ~600 ms window |
| GPS haversine       | EMA                 | α = 0.8                |
| Fused output        | EMA                 | α = 0.7                |

`distance_get_components()` exposes the per-component GPS distance, RSSI distance, and median RSSI — used by OLED page 3 and for RSSI calibration on page 0.

---

### `rendezvous_logic` — State Machine

Evaluates the docking state given a snapshot of local and remote telemetry.

Uses GPS haversine directly (not the fused estimate) for the state machine decision — the fused estimate is display-only. This avoids RSSI noise influencing the docking decision at short range.

**State transitions:**

```
Any node no GPS fix → STANDBY

dist ≥ 20 m          → STANDBY
2 m ≤ dist < 20 m    → ALIGNING
dist < 2 m:
  |ΔPitch| ≤ 15° and |ΔRoll| ≤ 15°  → DOCKED
  otherwise                           → MISS (FAILED)
```

Heading is excluded — too noisy without magnetometer calibration.

Both nodes run the same logic on the same data and will converge to the same state (modulo small timing differences in GPS update propagation).

---

### `lcd_i2c` — SSD1306 OLED Driver

Custom driver targeting the 128×64 SSD1306 over I2C. Uses a 5×8 bitmap font with a page-addressed HD44780-style API (`lcd_set_cursor`, `lcd_write_string`).

The display has a hardware 2-tone layout:
- Top ~16 px (rows 0–1): yellow zone — used for page title.
- Remaining 48 px (rows 2–5): blue zone — used for data content.

`lcd_clear()` blanks all pages. Page title is written once on page change; data rows are written every 50 ms.

---

### `tasks` — Task Manager

`tasks_start()` creates all FreeRTOS tasks, gated by `ACTIVE_MILESTONE`:

| Task             | Core | Priority | Stack  | Milestone | Period  |
|------------------|------|----------|--------|-----------|---------|
| `espnow_tx_task` | 0    | 5        | 4 KB   | M1+       | 200 ms  |
| `display_task`   | 1    | 3        | 6 KB   | M2+       | 50 ms   |
| `imu_task`       | 0    | 5        | 4 KB   | M3+       | 50 ms   |
| `gps_nmea_task`  | 1    | 4        | 4 KB   | M4+       | continuous |
| `gps_copy_task`  | 1    | 3        | 2 KB   | M4+       | 500 ms  |
| `rendezvous_task`| 0    | 3        | 3 KB   | M5        | 500 ms  |

`rendezvous_task` logs every state transition to serial with distance and pitch/roll deltas — useful for field diagnostics without needing to watch the OLED.

---

## Signal Smoothing Summary

| Signal              | Filter         | Parameter              | Location              |
|---------------------|----------------|------------------------|-----------------------|
| IMU pitch/roll      | EMA            | α = 0.3 (~140 ms τ)    | `icm20948.c`          |
| IMU heading         | Circular EMA   | α = 0.3, sin/cos domain| `icm20948.c`          |
| RSSI                | 3-sample median| ~600 ms window         | `distance_estimator.c`|
| GPS haversine       | EMA            | α = 0.8                | `distance_estimator.c`|
| Fused distance      | EMA            | α = 0.7                | `distance_estimator.c`|

Debug-level logging of raw vs. smoothed values is available — enable with `CONFIG_LOG_DEFAULT_LEVEL=DEBUG`.

---

## Future Work

### Magnetometer Hard/Soft-Iron Calibration

The current heading is computed but excluded from docking decisions. To make heading reliable enough for docking:

1. Implement a figure-8 calibration sweep to collect raw magnetometer data across all orientations.
2. Fit hard-iron offsets (per-axis bias) and a soft-iron correction matrix (3×3).
3. Store them as constants in `board_config.h`.
4. Apply correction in `icm20948_read()` before tilt-compensation.
5. Re-introduce heading delta into `rendezvous_evaluate()` as a docking criterion.

### Proximity at 2 m and GPS Accuracy

GPS CEP of 3–5 m means the state machine will straddle the 2 m `PROX_THRESHOLD_M` — brief DOCKED ↔ ALIGNING flips are expected and normal at close range. Possible mitigations:
- Lower the threshold if GPS conditions allow.
- Add hysteresis to `rendezvous_evaluate()` (separate entry/exit thresholds).
- Increase RSSI dominance weighting at close range.
