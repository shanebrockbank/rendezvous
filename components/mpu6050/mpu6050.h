#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

/**
 * mpu6050_init - wake the MPU6050 from sleep.
 *
 * @param port  I2C port number (e.g. I2C_NUM_1)
 * @return ESP_OK on success
 */
esp_err_t mpu6050_init(i2c_port_t port);

/**
 * mpu6050_read - read accelerometer and compute pitch and roll.
 *
 * @param pitch_deg  Output: pitch angle in degrees
 * @param roll_deg   Output: roll angle in degrees
 * @return ESP_OK on success
 */
esp_err_t mpu6050_read(float *pitch_deg, float *roll_deg);
