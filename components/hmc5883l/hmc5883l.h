#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

/**
 * hmc5883l_init - configure HMC5883L for continuous measurement.
 *
 * @param port  I2C port number (e.g. I2C_NUM_1)
 * @return ESP_OK on success
 */
esp_err_t hmc5883l_init(i2c_port_t port);

/**
 * hmc5883l_read - read magnetometer and compute compass heading.
 *
 * @param heading_deg  Output: heading in degrees [0, 360)
 * @return ESP_OK on success
 */
esp_err_t hmc5883l_read(float *heading_deg);
