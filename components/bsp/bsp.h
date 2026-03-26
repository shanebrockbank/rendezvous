#pragma once

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "board_config.h"

/**
 * bsp_init - one-time hardware bringup.
 * Call from app_main before creating any tasks.
 */
void bsp_init(void);

/**
 * bsp_get_i2c_bus - returns the shared I2C master bus handle.
 * All I2C component drivers call this to obtain the bus handle.
 */
i2c_master_bus_handle_t bsp_get_i2c_bus(void);

/**
 * bsp_get_gps_uart - returns the UART port number used for GPS.
 */
uart_port_t bsp_get_gps_uart(void);
