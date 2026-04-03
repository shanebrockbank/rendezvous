# Hardware Reference

## Bill of Materials

| Qty | Component               | Notes                                      |
|-----|-------------------------|--------------------------------------------|
| 2   | ESP32 development board | Classic ESP32 (dual-core Xtensa LX6)       |
| 2   | u-blox M8N GPS module   | NEO-M8N or compatible clone, UART output   |
| 2   | ICM-20948 breakout      | 9-DOF: accel + gyro + AK09916 magnetometer |
| 2   | SSD1306 OLED 128×64     | I2C, 0x3C address, 2-tone yellow/blue      |
| 2   | LED (external)          | Any colour, connected to GPIO4             |
| 2   | Tactile button          | Normally open, connected to GPIO13         |
| —   | Protoboard + jumper wire | Signal wiring                             |
| —   | 3.3 V power supply      | Or USB from development board              |

All components are purchased separately; no custom PCB is required for the prototype.

---

## Pin Assignment

Both nodes use identical pin layouts — only `NODE_ID` in `board_config.h` differs.

| Signal               | ESP32 GPIO | Direction | Notes                              |
|----------------------|------------|-----------|------------------------------------|
| I2C0 SDA (OLED)      | 21         | Bidirect  | 100 kHz; breakout pull-ups used    |
| I2C0 SCL (OLED)      | 22         | Out       | 100 kHz                            |
| I2C1 SDA (IMU)       | 18         | Bidirect  | 100 kHz; breakout pull-ups used    |
| I2C1 SCL (IMU)       | 19         | Out       | 100 kHz                            |
| GPS UART2 RX         | 16         | In        | Receives NMEA from GPS TX          |
| GPS UART2 TX         | 17         | Out       | Transmits to GPS RX (not required) |
| TX LED (onboard)     | 2          | Out       | Blinks on each ESP-NOW send        |
| RX LED (external)    | 4          | Out       | High while remote button is held   |
| BOOT button          | 0          | In        | Active-low; sends button_state in telemetry |
| Page-cycle button    | 13         | In        | Active-low; cycles OLED pages      |

---

## I2C Buses

Two separate I2C buses are used to avoid address conflicts and reduce per-bus load:

| Bus   | ESP-IDF Port | SDA | SCL | Devices              | Address |
|-------|--------------|-----|-----|----------------------|---------|
| I2C0  | `I2C_NUM_0`  | 21  | 22  | SSD1306 OLED         | 0x3C    |
| I2C1  | `I2C_NUM_1`  | 18  | 19  | ICM-20948            | 0x68 or 0x69 |

**Frequency:** Both buses run at **100 kHz** (`HAL_I2C_FREQ_HZ = 100000`). This was reduced from 400 kHz because the long protoboard wire runs to the ICM-20948 caused I2C timeout errors at higher speed. If routing moves to a PCB, 400 kHz is worth retrying.

**Pull-ups:** The SSD1306 and ICM-20948 breakout boards carry their own pull-up resistors. ESP32 internal pull-ups are also enabled in `bsp.c` as a secondary measure.

**ICM-20948 I2C address auto-probe:** `icm20948_init()` probes 0x68 first, then 0x69, so the ADO pin state doesn't matter at firmware level — it will find the chip regardless.

---

## GPS UART

| Parameter  | Value                                          |
|------------|------------------------------------------------|
| UART port  | `UART_NUM_2`                                   |
| RX pin     | GPIO16                                         |
| TX pin     | GPIO17                                         |
| Protocol   | NMEA 0183: `$GPRMC` / `$GNRMC` sentences       |
| Baud rate  | Auto-scanned at init: 9600, 4800, 19200, 38400, 57600, 115200 |

Cheap NEO-M8N clones sometimes ship at non-default baud rates. The firmware auto-scans all common rates at startup and locks onto whichever one produces valid NMEA data. If no valid sentence arrives within 5 seconds a warning is logged to serial every 5 s.

Only the RX connection (GPS TX → ESP32 GPIO16) is required for position data. TX is wired to allow future configuration commands.

---

## ESP-NOW Wireless

- **Mode:** Broadcast (no pairing required, no router)
- **Channel:** 1
- **Heartbeat rate:** 200 ms
- **Extra send:** immediate on BOOT button press/release edge
- **RSSI:** captured from `recv_info->rx_ctrl->rssi` in the receive callback

No external antenna or RF hardware is needed beyond the built-in ESP32 PCB antenna.

---

## ICM-20948 Mounting Notes

The ICM-20948 orientation matters for pitch/roll sign conventions.

**`ICM_MOUNT_INVERTED` in `board_config.h`:**
- Set to `1` if the chip is mounted **component-side down** (upside down on protoboard).
- This negates `az` (accelerometer Z) and `bz` (magnetometer Z) so pitch/roll read 0° when flat.
- The `atan2(Yh, Xh)` sign used in tilt-compensated heading (positive, not negative) follows from this mounting.

To verify: flash with offsets at 0.0, lay device flat, check OLED page 2. Pitch and roll should be close to 0°. If they read ±180°, toggle `ICM_MOUNT_INVERTED`.

---

## LED Behaviour

| LED          | GPIO | Behaviour                                             |
|--------------|------|-------------------------------------------------------|
| TX (onboard) | 2    | Blinks briefly on every ESP-NOW packet sent (~200 ms cadence) |
| RX (external)| 4    | Held high while the remote node's BOOT button is pressed |

---

## Two-Device Setup

Both devices are physically and electrically identical. The only firmware difference is `NODE_ID`:

| Node  | `NODE_ID` | Role   |
|-------|-----------|--------|
| Node 0 | `0`      | Chaser |
| Node 1 | `1`      | Target |

After flashing, place both nodes in the same area with clear sky view for GPS and power them on. They will begin exchanging telemetry automatically over ESP-NOW without any pairing step.
