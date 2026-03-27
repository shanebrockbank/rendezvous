#include "bsp.h"
#include "board_config.h"

#include "nvs_flash.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "bsp";

static i2c_master_bus_handle_t s_i2c_bus = NULL;

void bsp_init(void)
{
    // NVS — required by WiFi / ESP-NOW
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // I2C master bus (shared by MPU6050, HMC5883L, LCD)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = HAL_I2C_SDA_PIN,
        .scl_io_num        = HAL_I2C_SCL_PIN,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
    ESP_LOGI(TAG, "I2C bus initialised (SDA=%d SCL=%d)", HAL_I2C_SDA_PIN, HAL_I2C_SCL_PIN);

    // i2c.common fires "GPIO not usable" on every transaction (ESP-IDF v5 ownership check) — suppress
    // i2c.master kept at WARN so real errors (timeout, NACK) remain visible
    esp_log_level_set("i2c.common", ESP_LOG_NONE);
    esp_log_level_set("i2c.master", ESP_LOG_WARN);

    // GPS UART
    uart_config_t uart_cfg = {
        .baud_rate  = GPS_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM,
                                 GPS_UART_TX_PIN,
                                 GPS_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, 1024, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "GPS UART%d initialised (%d baud)", GPS_UART_NUM, GPS_UART_BAUD_RATE);

    // GPIO — TX LED (onboard), RX LED (external), BOOT button
    gpio_config_t io_out = {
        .pin_bit_mask = (1ULL << HAL_LED_TX_PIN) | (1ULL << HAL_LED_RX_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_out));

    gpio_config_t io_btn = {
        .pin_bit_mask = (1ULL << HAL_BUTTON_PIN) | (1ULL << HAL_BUTTON_PAGE_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_btn));

    gpio_set_level(HAL_LED_TX_PIN, 0);
    gpio_set_level(HAL_LED_RX_PIN, 0);

    ESP_LOGI(TAG, "BSP init complete (node_id=%d)", NODE_ID);
}

i2c_master_bus_handle_t bsp_get_i2c_bus(void)
{
    return s_i2c_bus;
}

uart_port_t bsp_get_gps_uart(void)
{
    return GPS_UART_NUM;
}
