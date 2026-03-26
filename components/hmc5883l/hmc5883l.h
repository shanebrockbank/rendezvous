#pragma once

#include "driver/i2c_master.h"

/**
 * hmc5883l_init - configure HMC5883L for continuous measurement.
 *
 * @param bus_handle  Shared I2C master bus handle from hal_get_i2c_bus()
 * @return ESP_OK on success
 */
esp_err_t hmc5883l_init(i2c_master_bus_handle_t bus_handle);

/**
 * hmc5883l_read - read magnetometer and compute compass heading.
 *
 * @param heading_deg  Output: heading in degrees [0, 360)
 * @return ESP_OK on success
 */
esp_err_t hmc5883l_read(float *heading_deg);
