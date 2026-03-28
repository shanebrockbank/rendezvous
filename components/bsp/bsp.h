#pragma once

#include "driver/i2c.h"
#include "driver/uart.h"
#include "board_config.h"

/**
 * bsp_init - one-time hardware bringup.
 * Call from app_main before creating any tasks.
 */
void bsp_init(void);

/**
 * bsp_get_i2c_port - returns I2C_NUM_0 (OLED display bus).
 */
i2c_port_t bsp_get_i2c_port(void);

/**
 * bsp_get_i2c_port2 - returns I2C_NUM_1 (IMU sensor bus).
 */
i2c_port_t bsp_get_i2c_port2(void);

/**
 * bsp_get_gps_uart - returns the UART port number used for GPS.
 */
uart_port_t bsp_get_gps_uart(void);
