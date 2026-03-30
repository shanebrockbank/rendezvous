#include "icm20948.h"
#include "board_config.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "icm20948";

// AK09916 I2C address — fixed, on ICM's internal secondary I2C bus
#define AK09916_ADDR  0x0C
// AK09916 WIA2 register (device ID = 0x09)
#define AK_WIA2   0x01
#define AK_ST1    0x10   // data ready (bit0: DRDY)
#define AK_CNTL2  0x31   // measurement mode

#define RAD_TO_DEG  (180.0f / (float)M_PI)
#define ACCEL_SCALE 16384.0f   // LSB/g for ±2g default full-scale

// REG_BANK_SEL is accessible from all register banks
#define REG_BANK_SEL       0x7F

// Bank 0 registers
#define B0_WHO_AM_I        0x00   // reads 0xEA for ICM-20948
#define B0_USER_CTRL       0x03   // bit5: I2C_MST_EN
#define B0_PWR_MGMT_1      0x06
#define B0_PWR_MGMT_2      0x07
#define B0_INT_PIN_CFG     0x0F   // bit1: BYPASS_EN
#define B0_ACCEL_XOUT_H    0x2D   // 6 bytes: XH,XL,YH,YL,ZH,ZL
#define B0_EXT_SLV_DATA_00 0x3B   // 9-byte AK09916 snapshot (ST1+XYZ+RSVA+ST2)

// Bank 3 registers (I2C master)
#define B3_I2C_MST_CTRL    0x01
#define B3_I2C_SLV0_ADDR   0x03   // bit7: R/W, bits6:0: slave addr
#define B3_I2C_SLV0_REG    0x04
#define B3_I2C_SLV0_CTRL   0x05   // bit7: EN, bits3:0: byte count

static i2c_port_t s_port;
static uint8_t    s_addr = ICM20948_I2C_ADDR;   // resolved at init — may be 0x68 or 0x69

// ---------------------------------------------------------------------------
// EMA smoothing state — pitch/roll use linear EMA; heading uses circular EMA
// (sin/cos domain) to handle the 0°/360° wrap correctly.
// ---------------------------------------------------------------------------
#define EMA_ALPHA_IMU   0.3f

static bool  s_ema_init    = false;
static float s_pitch_ema   = 0.0f;
static float s_roll_ema    = 0.0f;
static float s_sin_hdg_ema = 0.0f;
static float s_cos_hdg_ema = 1.0f;   // default: 0° (north)

// ---------------------------------------------------------------------------
// Low-level I2C helpers — talk to ICM-20948 itself
// ---------------------------------------------------------------------------
static esp_err_t icm_write(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t icm_read(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t set_bank(uint8_t bank)
{
    return icm_write(REG_BANK_SEL, (uint8_t)(bank << 4));
}

// Direct I2C write to AK09916 — only valid while BYPASS_EN is set
static esp_err_t ak_write(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK09916_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Direct I2C read from AK09916 — only valid while BYPASS_EN is set
static esp_err_t ak_read(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK09916_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK09916_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
esp_err_t icm20948_init(i2c_port_t port)
{
    s_port = port;
    esp_err_t ret;

    // --- Auto-detect I2C address: 0x68 (ADO low) or 0x69 (ADO high) ---
    // Try the address from board_config.h first, then the other one.
    // This handles boards where ADO is pulled high (e.g. different PCB revision).
    {
        uint8_t alt = (ICM20948_I2C_ADDR == 0x68) ? 0x69 : 0x68;
        s_addr = ICM20948_I2C_ADDR;
        if (set_bank(0) != ESP_OK) {
            ESP_LOGW(TAG, "no ACK at 0x%02X — trying 0x%02X (ADO pin high?)", s_addr, alt);
            s_addr = alt;
            if (set_bank(0) != ESP_OK) {
                ESP_LOGE(TAG, "ICM-20948 not found at 0x68 or 0x69");
                ESP_LOGE(TAG, "  Check I2C bus 1 wiring: SDA=GPIO%d SCL=GPIO%d",
                         HAL_I2C2_SDA_PIN, HAL_I2C2_SCL_PIN);
                return ESP_ERR_NOT_FOUND;
            }
        }
        ESP_LOGI(TAG, "ICM-20948 found at 0x%02X", s_addr);
    }

    if ((ret = set_bank(0)) != ESP_OK) goto fail;

    // Soft reset
    if ((ret = icm_write(B0_PWR_MGMT_1, 0x80)) != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(20));

    // Wake + auto clock
    if ((ret = icm_write(B0_PWR_MGMT_1, 0x01)) != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(5));

    // Verify ICM WHO_AM_I
    uint8_t who;
    if ((ret = icm_read(B0_WHO_AM_I, &who, 1)) != ESP_OK) goto fail;
    if (who != 0xEA) {
        ESP_LOGE(TAG, "ICM WHO_AM_I mismatch: got 0x%02X, expected 0xEA", who);
        return ESP_ERR_NOT_FOUND;
    }

    // Enable all accel + gyro axes
    if ((ret = icm_write(B0_PWR_MGMT_2, 0x00)) != ESP_OK) goto fail;

    // --- Configure AK09916 via bypass mode ---
    // I2C master must be OFF for bypass to bridge AK09916 onto the host I2C bus.
    if ((ret = icm_write(B0_USER_CTRL,   0x00)) != ESP_OK) goto fail; // I2C_MST_EN = 0
    if ((ret = icm_write(B0_INT_PIN_CFG, 0x02)) != ESP_OK) goto fail; // BYPASS_EN = 1
    vTaskDelay(pdMS_TO_TICKS(5));

    // Verify AK09916 is reachable
    uint8_t ak_who;
    if ((ret = ak_read(AK_WIA2, &ak_who, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "AK09916 not responding (bypass mode) — check wiring");
        goto fail;
    }
    if (ak_who != 0x09) {
        ESP_LOGE(TAG, "AK09916 WIA2 mismatch: got 0x%02X, expected 0x09", ak_who);
        return ESP_ERR_NOT_FOUND;
    }

    // Put AK09916 into continuous 100 Hz measurement mode
    if ((ret = ak_write(AK_CNTL2, 0x08)) != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(5));

    // --- Disable bypass, enable I2C master ---
    if ((ret = icm_write(B0_INT_PIN_CFG, 0x00)) != ESP_OK) goto fail; // BYPASS_EN = 0
    if ((ret = icm_write(B0_USER_CTRL,   0x20)) != ESP_OK) goto fail; // I2C_MST_EN = 1

    // Bank 3: configure I2C master clock and SLV0 for continuous AK09916 reads
    if ((ret = set_bank(3)) != ESP_OK) goto fail;

    // I2C master clock ~304 kHz (safe for AK09916 ≤400 kHz)
    if ((ret = icm_write(B3_I2C_MST_CTRL, 0x07)) != ESP_OK) goto fail;

    // SLV0: continuously read 9 bytes from AK09916 starting at ST1 (0x10)
    // EXT_SLV_DATA layout: [ST1, HXL, HXH, HYL, HYH, HZL, HZH, RSVA, ST2]
    if ((ret = icm_write(B3_I2C_SLV0_ADDR, AK09916_ADDR | 0x80)) != ESP_OK) goto fail; // read
    if ((ret = icm_write(B3_I2C_SLV0_REG,  AK_ST1))              != ESP_OK) goto fail;
    if ((ret = icm_write(B3_I2C_SLV0_CTRL, 0x89))                != ESP_OK) goto fail; // en+9 bytes

    if ((ret = set_bank(0)) != ESP_OK) goto fail;

    ESP_LOGI(TAG, "ICM-20948 + AK09916 initialised at 0x%02X%s",
             s_addr,
             ICM_MOUNT_INVERTED ? " [Z-axis inverted for upside-down mount]" : "");
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t icm20948_read(float *pitch_deg, float *roll_deg, float *heading_deg)
{
    esp_err_t ret;

    if ((ret = set_bank(0)) != ESP_OK) return ret;

    // --- Accelerometer (6 bytes) ---
    uint8_t raw[6];
    if ((ret = icm_read(B0_ACCEL_XOUT_H, raw, 6)) != ESP_OK) {
        ESP_LOGW(TAG, "Accel read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);

    float ax = ax_raw / ACCEL_SCALE;
    float ay = ay_raw / ACCEL_SCALE;
    float az = az_raw / ACCEL_SCALE;

#if ICM_MOUNT_INVERTED
    // Board is mounted component-side down — Z axis points opposite to normal.
    // Negate az so pitch/roll read correctly (0° when flat, not ±180°).
    az = -az;
#endif

    float pitch_rad = atan2f(ay, sqrtf(ax * ax + az * az));
    float roll_rad  = atan2f(-ax, az);

    *pitch_deg = pitch_rad * RAD_TO_DEG;
    *roll_deg  = roll_rad  * RAD_TO_DEG;

    // --- AK09916 magnetometer (from EXT_SLV_SENS_DATA, 9 bytes) ---
    // Layout: [ST1, HXL, HXH, HYL, HYH, HZL, HZH, RSVA, ST2]
    uint8_t mag[9];
    if ((ret = icm_read(B0_EXT_SLV_DATA_00, mag, 9)) != ESP_OK) {
        ESP_LOGW(TAG, "Mag read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // AK09916 is little-endian: low byte first
    int16_t mx = (int16_t)((mag[2] << 8) | mag[1]);
    int16_t my = (int16_t)((mag[4] << 8) | mag[3]);
    int16_t mz = (int16_t)((mag[6] << 8) | mag[5]);

    float bx = (float)mx;
    float by = (float)my;
    float bz = (float)mz;

#if ICM_MOUNT_INVERTED
    // Mag Z axis mirrors the accel Z axis — negate it to match the corrected frame.
    bz = -bz;
#endif

    // --- Tilt-compensated heading ---
    // Xh = Bx*cos(pitch) + Bz*sin(pitch)
    // Yh = Bx*sin(roll)*sin(pitch) + By*cos(roll) - Bz*sin(roll)*cos(pitch)
    // heading = atan2(-Yh, Xh)
    float cp = cosf(pitch_rad), sp = sinf(pitch_rad);
    float cr = cosf(roll_rad),  sr = sinf(roll_rad);

    float xh = bx * cp + bz * sp;
    float yh = bx * sr * sp + by * cr - bz * sr * cp;

    float heading_raw = atan2f(yh, xh) * RAD_TO_DEG;
    if (heading_raw < 0.0f) heading_raw += 360.0f;

    // Log raw values at ~1 s intervals so raw vs smooth can be compared
    static int s_log_cnt = 0;
    bool do_log = (++s_log_cnt >= 20);
    if (do_log) s_log_cnt = 0;

    if (do_log) {
        ESP_LOGD(TAG, "RAW    P=%+6.1f  R=%+6.1f  HDG=%5.1f",
                 *pitch_deg, *roll_deg, heading_raw);
    }

    // --- EMA smoothing ---
    if (!s_ema_init) {
        s_pitch_ema   = *pitch_deg;
        s_roll_ema    = *roll_deg;
        float h_rad   = heading_raw * ((float)M_PI / 180.0f);
        s_sin_hdg_ema = sinf(h_rad);
        s_cos_hdg_ema = cosf(h_rad);
        s_ema_init    = true;
    } else {
        s_pitch_ema = EMA_ALPHA_IMU * (*pitch_deg) + (1.0f - EMA_ALPHA_IMU) * s_pitch_ema;
        s_roll_ema  = EMA_ALPHA_IMU * (*roll_deg)  + (1.0f - EMA_ALPHA_IMU) * s_roll_ema;
        float h_rad   = heading_raw * ((float)M_PI / 180.0f);
        s_sin_hdg_ema = EMA_ALPHA_IMU * sinf(h_rad) + (1.0f - EMA_ALPHA_IMU) * s_sin_hdg_ema;
        s_cos_hdg_ema = EMA_ALPHA_IMU * cosf(h_rad) + (1.0f - EMA_ALPHA_IMU) * s_cos_hdg_ema;
    }

    *pitch_deg = s_pitch_ema;
    *roll_deg  = s_roll_ema;
    float smooth_hdg = atan2f(s_sin_hdg_ema, s_cos_hdg_ema) * RAD_TO_DEG;
    if (smooth_hdg < 0.0f) smooth_hdg += 360.0f;
    *heading_deg = smooth_hdg;

    if (do_log) {
        ESP_LOGD(TAG, "SMOOTH P=%+6.1f  R=%+6.1f  HDG=%5.1f  (alpha=%.1f)",
                 *pitch_deg, *roll_deg, *heading_deg, EMA_ALPHA_IMU);
    }

    return ESP_OK;
}
