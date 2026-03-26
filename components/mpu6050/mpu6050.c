#include "mpu6050.h"
#include "board_config.h"
#include "register_maps.h"

#include "esp_log.h"
#include "math.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "mpu6050";

#define RAD_TO_DEG  (180.0f / M_PI)
#define ACCEL_SCALE  16384.0f   // LSB/g for ±2g default range

static i2c_master_dev_handle_t s_dev = NULL;

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, pdMS_TO_TICKS(50));
}

static esp_err_t i2c_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_transmit(s_dev, &reg, 1, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) return ret;
    return i2c_master_receive(s_dev, data, len, pdMS_TO_TICKS(50));
}

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_I2C_ADDR,
        .scl_speed_hz    = HAL_I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wake from sleep: write 0 to PWR_MGMT_1
    ret = i2c_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
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
