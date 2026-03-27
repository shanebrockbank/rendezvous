#pragma once

#include "driver/i2c_master.h"

/**
 * lcd_init - initialise HD44780 LCD via PCF8574 I2C backpack.
 *
 * @param bus_handle  Shared I2C master bus handle from hal_get_i2c_bus()
 * @param i2c_addr    PCF8574 address (0x27 or 0x3F)
 */
void lcd_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr);

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

