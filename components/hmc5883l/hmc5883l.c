#include "hmc5883l.h"
#include "board_config.h"
#include "register_maps.h"

#include "esp_log.h"
#include "math.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "hmc5883l";

#define RAD_TO_DEG  (180.0f / M_PI)

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

esp_err_t hmc5883l_init(i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = HMC5883L_I2C_ADDR,
        .scl_speed_hz    = HAL_I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Config A: 8 samples average, 15 Hz output rate
    ret = i2c_write_reg(HMC5883L_REG_CONFIG_A, 0x70);
    if (ret != ESP_OK) goto fail;

    // Config B: gain = 1090 LSB/Gauss (default)
    ret = i2c_write_reg(HMC5883L_REG_CONFIG_B, 0x20);
    if (ret != ESP_OK) goto fail;

    // Mode: continuous measurement
    ret = i2c_write_reg(HMC5883L_REG_MODE, 0x00);
    if (ret != ESP_OK) goto fail;

    ESP_LOGI(TAG, "HMC5883L initialised at 0x%02X", HMC5883L_I2C_ADDR);
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t hmc5883l_read(float *heading_deg)
{
    // HMC5883L data register order: X_H, X_L, Z_H, Z_L, Y_H, Y_L  (Z before Y!)
    uint8_t raw[6];
    esp_err_t ret = i2c_read_regs(HMC5883L_REG_DATA_XH, raw, 6);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t z = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t y = (int16_t)((raw[4] << 8) | raw[5]);
    (void)z; // Z not used for flat-plane heading

    float heading = atan2f((float)y, (float)x) * RAD_TO_DEG;
    if (heading < 0.0f) heading += 360.0f;
    *heading_deg = heading;

    return ESP_OK;
}
