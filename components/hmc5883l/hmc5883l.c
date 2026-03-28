#include "hmc5883l.h"
#include "board_config.h"
#include "register_maps.h"

#include "esp_log.h"
#include "math.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "hmc5883l";

#define RAD_TO_DEG  (180.0f / M_PI)

static i2c_port_t s_port;

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (HMC5883L_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
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
    i2c_master_write_byte(cmd, (HMC5883L_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (HMC5883L_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t hmc5883l_init(i2c_port_t port)
{
    s_port = port;

    // Config A: 8 samples average, 15 Hz output rate
    esp_err_t ret = i2c_write_reg(HMC5883L_REG_CONFIG_A, 0x70);
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
