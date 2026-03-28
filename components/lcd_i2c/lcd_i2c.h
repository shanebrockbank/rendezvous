#pragma once

#include "driver/i2c.h"

/**
 * lcd_init - initialise SSD1306 OLED via legacy I2C driver.
 *
 * @param port      I2C port number (e.g. I2C_NUM_0)
 * @param i2c_addr  SSD1306 address (typically 0x3C)
 */
void lcd_init(i2c_port_t port, uint8_t i2c_addr);

/**
 * lcd_clear - clear display and return cursor to home.
 */
void lcd_clear(void);

/**
 * lcd_set_cursor - position cursor.
 *
 * @param row  0 or 1
 * @param col  0-15 (for 16-char wide display)
 */
void lcd_set_cursor(uint8_t row, uint8_t col);

/**
 * lcd_write_string - write a null-terminated string at the current cursor.
 */
void lcd_write_string(const char *str);
