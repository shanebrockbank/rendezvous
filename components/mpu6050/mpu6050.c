#include "mpu6050.h"
#include "board_config.h"
#include "register_maps.h"

#include "esp_log.h"
#include "math.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "mpu6050";

#define RAD_TO_DEG   (180.0f / M_PI)
#define ACCEL_SCALE  16384.0f   // LSB/g for ±2g default range

static i2c_port_t s_port;

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_init(i2c_port_t port)
{
    s_port = port;

    // Wake from sleep: write 0 to PWR_MGMT_1
    esp_err_t ret = i2c_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialised at 0x%02X", MPU6050_I2C_ADDR);
    return ESP_OK;
}

esp_err_t mpu6050_read(float *pitch_deg, float *roll_deg)
{
    uint8_t raw[6];
    esp_err_t ret = i2c_read_regs(MPU6050_REG_ACCEL_XOUT_H, raw, 6);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);

    float ax = ax_raw / ACCEL_SCALE;
    float ay = ay_raw / ACCEL_SCALE;
    float az = az_raw / ACCEL_SCALE;

    *pitch_deg = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
    *roll_deg  = atan2f(-ax, az) * RAD_TO_DEG;

    return ESP_OK;
}
