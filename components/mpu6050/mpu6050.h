#pragma once

#include "driver/i2c_master.h"

/**
 * mpu6050_init - wake the MPU6050 from sleep and verify communication.
 *
 * @param bus_handle  Shared I2C master bus handle from hal_get_i2c_bus()
 * @return ESP_OK on success
 */
esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle);

/**
 * mpu6050_read - read accelerometer and compute pitch and roll.
 *
 * @param pitch_deg  Output: pitch angle in degrees
 * @param roll_deg   Output: roll angle in degrees
 * @return ESP_OK on success
 */
esp_err_t mpu6050_read(float *pitch_deg, float *roll_deg);
