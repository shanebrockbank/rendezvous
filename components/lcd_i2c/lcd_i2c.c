#include "lcd_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lcd_i2c";

// PCF8574 bit positions for HD44780 4-bit mode backpack
// Typical wiring: P0=RS P1=RW P2=E P3=Backlight P4=D4 P5=D5 P6=D6 P7=D7
#define LCD_RS   (1 << 0)
#define LCD_RW   (1 << 1)
#define LCD_EN   (1 << 2)
#define LCD_BL   (1 << 3)   // backlight always on
#define LCD_D4   (1 << 4)
#define LCD_D5   (1 << 5)
#define LCD_D6   (1 << 6)
#define LCD_D7   (1 << 7)

static i2c_master_dev_handle_t s_dev = NULL;
static uint8_t s_backlight = LCD_BL;

static esp_err_t lcd_write_i2c(uint8_t data)
{
    return i2c_master_transmit(s_dev, &data, 1, pdMS_TO_TICKS(50));
}

static void lcd_pulse_enable(uint8_t data)
{
    lcd_write_i2c(data | LCD_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_i2c(data & ~LCD_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void lcd_write_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data = s_backlight | rs;
    data |= (nibble & 0x01) ? LCD_D4 : 0;
    data |= (nibble & 0x02) ? LCD_D5 : 0;
    data |= (nibble & 0x04) ? LCD_D6 : 0;
    data |= (nibble & 0x08) ? LCD_D7 : 0;
    lcd_pulse_enable(data);
}

static void lcd_send_byte(uint8_t byte, uint8_t rs)
{
    lcd_write_nibble(byte >> 4, rs);
    lcd_write_nibble(byte & 0x0F, rs);
}

static void lcd_cmd(uint8_t cmd)
{
    lcd_send_byte(cmd, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = i2c_addr,
        .scl_speed_hz    = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_dev));

    // HD44780 initialisation sequence — 4-bit mode
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write_i2c(s_backlight);

    // Send 0x03 three times (special reset sequence)
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(1));

    // Switch to 4-bit mode
    lcd_write_nibble(0x02, 0); vTaskDelay(pdMS_TO_TICKS(1));

    // Function set: 4-bit, 2 lines, 5x8 font
    lcd_cmd(0x28);
    // Display on, cursor off, blink off
    lcd_cmd(0x0C);
    // Clear display
    lcd_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(3));
    // Entry mode: increment, no shift
    lcd_cmd(0x06);

    ESP_LOGI(TAG, "LCD initialised at addr 0x%02X", i2c_addr);
}

void lcd_clear(void)
{
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(3));
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    static const uint8_t row_offsets[] = {0x00, 0x40};
    lcd_cmd(0x80 | (row_offsets[row & 1] + col));
}

void lcd_write_char(char c)
{
    lcd_send_byte((uint8_t)c, LCD_RS);
}

void lcd_write_string(const char *str)
{
    while (*str) {
        lcd_write_char(*str++);
    }
}
