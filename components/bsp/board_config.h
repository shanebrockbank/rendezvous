#pragma once

// I2C Bus 0 — OLED display (SSD1306)
#define HAL_I2C_SDA_PIN       21
#define HAL_I2C_SCL_PIN       22
#define HAL_I2C_FREQ_HZ       400000

// I2C Bus 1 — IMU sensors (MPU6050 + HMC5883L)
#define HAL_I2C2_SDA_PIN      18
#define HAL_I2C2_SCL_PIN      19

// I2C Device Addresses
#define MPU6050_I2C_ADDR      0x68
#define HMC5883L_I2C_ADDR     0x1E
#define LCD_I2C_ADDR          0x3C

// GPS UART
#define GPS_UART_NUM          UART_NUM_2
#define GPS_UART_RX_PIN       16
#define GPS_UART_TX_PIN       17
#define GPS_UART_BAUD_RATE    9600

// GPIO
#define HAL_LED_TX_PIN        2      // onboard LED — blinks on ESP-NOW send
#define HAL_LED_RX_PIN        4      // external LED — blinks on ESP-NOW receive
#define HAL_BUTTON_PIN        0      // BOOT button — ESP-NOW button_state telemetry (active low)
#define HAL_BUTTON_PAGE_PIN   13     // External button — cycles LCD display pages (active low)

// Node identity — change per flash (0 = chaser, 1 = target)
#define NODE_ID               1

// Active milestone — controls which tasks run at startup
// 1 = ESP-NOW comms + LED/button
// 2 = + LCD display
// 3 = + IMU (MPU6050 + HMC5883L)
// 4 = + GPS NMEA
// 5 = + rendezvous docking logic (all tasks)
#define ACTIVE_MILESTONE      3
