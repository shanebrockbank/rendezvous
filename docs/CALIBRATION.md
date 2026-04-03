# Calibration Guide

Two calibration values must be set per-device in `components/bsp/board_config.h` before the system is accurate. Neither requires any equipment beyond the hardware itself.

---

## 1 — RSSI Reference Level (`RSSI_REF_1M`)

**What it is:** The RSSI value (in dBm) measured when the two devices are exactly 1 m apart in open air. Used as the reference point for the RSSI log-distance model.

**Why it matters:** The RSSI distance model is:

```
d = 10 ^ ((RSSI_REF_1M - rssi) / (10 × n))
```

with `n = 2.0` for open air. If `RSSI_REF_1M` is wrong, the RSSI-derived distance (dominant at < 2 m) will be off.

**Procedure:**

1. Flash both devices with the current firmware.
2. Take both devices outdoors to an open area (avoid walls and metal).
3. Place them exactly **1 m apart**, face to face, at the same height.
4. Power both on and wait for the link to establish (page 0 shows GOOD link status).
5. Navigate to **OLED page 0 (LINK)** on either device.
6. Watch the `RSSI: xxx dBm` line and wait ~5 seconds for the reading to stabilise.
7. Note the displayed dBm value (e.g. `-55`).
8. Set `RSSI_REF_1M` to that value in `board_config.h` on **both** devices:

   ```c
   #define RSSI_REF_1M  -55
   ```

9. Reflash both devices.
10. Verify: hold devices 1 m apart and check OLED page 3 (DISTANCE). The `RSSI` distance line should read approximately `1.0 m`.

**Current calibrated value:** `-55 dBm` (field measured, devices 1 m apart outdoors).

> Previous firmware used `-29 dBm` back-calculated from -38 dBm at ~3 m, which was inaccurate. Always use a direct 1 m measurement.

---

## 2 — IMU Static Offset (`ICM_PITCH_OFFSET_DEG` / `ICM_ROLL_OFFSET_DEG`)

**What it is:** A static offset in degrees subtracted from pitch and roll after EMA smoothing. Corrects for physical mounting misalignment of the ICM-20948 on the protoboard.

**Why it matters:** If the chip is not perfectly flat, the raw readings will have a constant bias. These offsets remove it so that "flat on a level surface" reads 0.0° / 0.0°.

**Procedure (per device):**

1. Set both offsets to zero:

   ```c
   #define ICM_PITCH_OFFSET_DEG  0.0f
   #define ICM_ROLL_OFFSET_DEG   0.0f
   ```

2. Flash the device.
3. Place the device on a **level surface** (use a bubble level if available).
4. Navigate to **OLED page 2 (IMU)**.
5. Wait ~5 seconds for the EMA filter to settle.
6. Note the displayed `Tx-P:` (pitch) and `Tx-R:` (roll) values — e.g. `-2.4` and `+3.6`.
7. Set the offsets to those exact values:

   ```c
   #define ICM_PITCH_OFFSET_DEG  -2.4f
   #define ICM_ROLL_OFFSET_DEG    3.6f
   ```

8. Reflash. OLED page 2 should now show `Tx-P:+0.0 Tx-R:+0.0` when flat.

**Note:** Each device gets its own offset values. Repeat this procedure independently for each node.

---

## 3 — Node Identity (`NODE_ID`)

Not a calibration in the sensor sense, but must be set per-device before flashing:

```c
#define NODE_ID  0   // 0 = Chaser, 1 = Target
```

Flash Node 0 (Chaser) with `NODE_ID 0` and Node 1 (Target) with `NODE_ID 1`. The `NODE_ID` value is included in every telemetry packet so each device knows which data is its own.

---

## 4 — Future: Magnetometer Hard/Soft-Iron Calibration

> **Not yet implemented.** This section documents the planned approach.

The AK09916 heading is currently used for display and telemetry only — it is intentionally excluded from the docking check because uncalibrated heading error is too large to be reliable.

To re-introduce heading into docking decisions, two correction steps are needed:

**Hard-iron correction** removes constant bias caused by nearby ferrous materials (screws, battery, etc.). It adds a fixed offset to each axis:

```
Bx_cal = Bx - offset_x
By_cal = By - offset_y
Bz_cal = Bz - offset_z
```

**Soft-iron correction** removes axis scaling and skew from induced magnetism:

```
B_cal = M × B_raw
```

where `M` is a 3×3 correction matrix.

Both are computed from a figure-8 calibration sweep (rotate the device slowly through all orientations while logging raw magnetometer data), then fit to the recorded min/max/ellipse. The resulting offsets and matrix are stored as constants in `board_config.h`.

Until this is implemented, heading accuracy is limited to approximately ±15–30° depending on local magnetic interference.

---

## Quick Reference

| Parameter              | Location          | Per-device? | Default      |
|------------------------|-------------------|-------------|--------------|
| `RSSI_REF_1M`          | `board_config.h`  | Yes (both)  | -55          |
| `ICM_PITCH_OFFSET_DEG` | `board_config.h`  | Yes (each)  | 0.0f         |
| `ICM_ROLL_OFFSET_DEG`  | `board_config.h`  | Yes (each)  | 0.0f         |
| `NODE_ID`              | `board_config.h`  | Yes (each)  | 0            |
| `ICM_MOUNT_INVERTED`   | `board_config.h`  | Yes (each)  | 1            |
| `ACTIVE_MILESTONE`     | `board_config.h`  | No (same)   | 5            |
