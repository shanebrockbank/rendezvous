#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

/**
 * icm20948_init - initialise ICM-20948 and its AK09916 magnetometer.
 *
 * Configures the ICM as I2C master to sample AK09916 automatically each cycle.
 * Call once from app_main before creating any tasks.
 *
 * @param port  I2C port number (e.g. I2C_NUM_1)
 * @return ESP_OK on success
 */
esp_err_t icm20948_init(i2c_port_t port);

/**
 * icm20948_read - read accel-derived pitch/roll and tilt-compensated heading.
 *
 * @param pitch_deg    Output: pitch angle in degrees (−180 to +180)
 * @param roll_deg     Output: roll angle in degrees  (−180 to +180)
 * @param heading_deg  Output: tilt-compensated magnetic heading, 0–360°
 * @return ESP_OK on success
 */
esp_err_t icm20948_read(float *pitch_deg, float *roll_deg, float *heading_deg);
