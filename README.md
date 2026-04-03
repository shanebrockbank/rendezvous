# Spaceium — ESP32 Rendezvous System

A two-device ESP32 firmware that demonstrates a docking/rendezvous scenario using real GPS, IMU orientation, and ESP-NOW wireless communication. Each node acquires its own GPS position and IMU pitch/roll, shares that data wirelessly with its peer, then evaluates a GPS + RSSI-fused distance and orientation alignment to declare a **DOCKED**, **ALIGNING**, or **MISS** outcome — displayed live on a local SSD1306 OLED.

---

## Hardware

| Component         | Part                         | Interface           |
|-------------------|------------------------------|---------------------|
| MCU               | ESP32 (classic, dual-core)   | —                   |
| GPS               | u-blox M8N (NEO-M8N clone)   | UART2               |
| IMU               | ICM-20948 (accel/gyro + AK09916 mag) | I2C bus 1   |
| Display           | SSD1306 OLED 128×64          | I2C bus 0 (0x3C)    |
| Wireless          | ESP-NOW (built-in WiFi radio) | —                  |

Two identical hardware units are required — one flashed as **Node 0 (Chaser)**, one as **Node 1 (Target)**.

---

## Pin Assignment

| Function              | GPIO |
|-----------------------|------|
| I2C0 SDA — OLED       | 21   |
| I2C0 SCL — OLED       | 22   |
| I2C1 SDA — IMU        | 18   |
| I2C1 SCL — IMU        | 19   |
| GPS UART2 RX          | 16   |
| GPS UART2 TX          | 17   |
| TX LED (onboard)      | 2    |
| RX LED (external)     | 4    |
| BOOT button (telemetry) | 0  |
| Page-cycle button     | 13   |

Both buses run at **100 kHz** — reduced from 400 kHz to handle long protoboard wire runs on the ICM-20948.

---

## Quick Start

### Prerequisites

- [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
- `IDF_PATH` environment variable set

### Build & Flash

```bash
cd rendezvous
idf.py set-target esp32    # once per checkout
idf.py build
idf.py flash monitor
```

### Configure Node Identity

Open `components/bsp/board_config.h` and set `NODE_ID` before flashing each device:

```c
#define NODE_ID  0   // 0 = Chaser, 1 = Target
```

Flash each device with the matching value. All other hardware and pin config is symmetric — the only difference is `NODE_ID`.

---

## OLED Display Pages

Cycled by pressing the **GPIO13** button. The display uses a two-tone SSD1306 layout: the top ~16 px (yellow zone) shows the page title; the remainder (blue zone) shows live data.

| Page | Title        | Content                                                     |
|------|--------------|-------------------------------------------------------------|
| 0    | LINK         | Link quality (GOOD/FAIR/POOR), packet age, sequence, RSSI   |
| 1    | GPS          | Local Tx lat/lon + remote Rx lat/lon                        |
| 2    | IMU          | Tx pitch/roll, Rx pitch/roll, delta pitch/roll              |
| 3    | DISTANCE     | GPS distance + blend %, RSSI distance + blend %, fused      |
| 4    | RENDEZVOUS   | State, fused distance, pitch/roll deltas, status line       |

On power-up a 2-second boot splash displays the project name and node identity.

---

## Milestone System

Features are enabled incrementally via `ACTIVE_MILESTONE` in `board_config.h`. Incrementing this value activates more tasks at startup — nothing else in the codebase changes.

| Milestone | Feature                                                  | Status            |
|-----------|----------------------------------------------------------|-------------------|
| M1        | ESP-NOW broadcast, TX/RX LEDs, button telemetry          | Hardware-verified |
| M2        | SSD1306 OLED, 5 display pages, page-cycle button         | Hardware-verified |
| M3        | MPU6050 + HMC5883L IMU *(superseded by M3.5)*            | Deprecated        |
| M3.5      | ICM-20948 (AK09916 mag), tilt-compensated heading        | Hardware-verified |
| M4        | GPS NMEA parser + GPS/RSSI fused distance estimator      | Hardware-verified |
| M5        | Full rendezvous: all tasks active, pitch/roll docking    | Hardware-verified |

The deprecated M3 drivers (`mpu6050`, `hmc5883l`) are retained in the tree for reference.

---

## Distance Estimation

GPS haversine and RSSI-derived distance are fused by range:

- **> 20 m** — GPS dominant (3–5 m CEP reliable at this scale)
- **< 2 m** — RSSI dominant (GPS noise exceeds resolution)
- **2–20 m** — linear blend between the two

RSSI model: `d = 10 ^ ((RSSI_REF_1M − rssi) / (10 × n))`, with `n = 2.0` for open air.

See [docs/CALIBRATION.md](docs/CALIBRATION.md) for the RSSI reference calibration procedure.

---

## Docking Logic

The rendezvous state machine evaluates independently on each node:

| State      | Condition                                                    |
|------------|--------------------------------------------------------------|
| `STANDBY`  | GPS haversine distance ≥ 20 m, or no GPS fix                 |
| `ALIGNING` | 2 m ≤ distance < 20 m                                        |
| `DOCKED`   | distance < 2 m **and** \|ΔPitch\| ≤ 15° **and** \|ΔRoll\| ≤ 15° |
| `MISS`     | distance < 2 m but orientation out of threshold             |

Heading is intentionally excluded — tilt-compensated heading from the AK09916 proved too noisy in field conditions without hard/soft-iron calibration. See the Future Work section in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

State transitions are logged to serial with distance and delta values, useful for field diagnostics without watching the OLED.

---

## Calibration

Two per-device calibration values must be set in `board_config.h` before use:

- **`RSSI_REF_1M`** — RSSI at 1 m reference distance (field measured)
- **`ICM_PITCH_OFFSET_DEG` / `ICM_ROLL_OFFSET_DEG`** — static IMU mounting offset

Full procedures in [docs/CALIBRATION.md](docs/CALIBRATION.md).

---

## Dev Container

A `.devcontainer/` is provided with a pre-configured ESP-IDF environment. Open in VS Code with the Dev Containers extension installed — no local IDF install required.

---

## Documentation

- [docs/HARDWARE.md](docs/HARDWARE.md) — Bill of materials, wiring notes, I2C/UART details
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — Component breakdown, task model, algorithms
- [docs/CALIBRATION.md](docs/CALIBRATION.md) — RSSI and IMU calibration procedures

---

## Project Structure

```
rendezvous/
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   └── main.c                   # app_main: bsp_init → sensor inits → tasks
└── components/
    ├── bsp/                     # Hardware bringup: NVS, I2C×2, UART, GPIO
    ├── espnow_comms/            # ESP-NOW TX/RX, telemetry_packet_t
    ├── gps_nmea/                # UART NMEA parser → lat/lon, mutex-protected
    ├── icm20948/                # ICM-20948 + AK09916: tilt-compensated heading
    ├── distance_estimator/      # GPS haversine + RSSI log-distance fusion
    ├── lcd_i2c/                 # SSD1306 OLED: 5×8 bitmap font
    ├── rendezvous_logic/        # Haversine, state machine (STANDBY/ALIGNING/DOCKED/MISS)
    ├── tasks/                   # FreeRTOS task creation, milestone-gated
    ├── mpu6050/                 # [deprecated M3] MPU6050 accel driver
    └── hmc5883l/                # [deprecated M3] HMC5883L mag driver
```
