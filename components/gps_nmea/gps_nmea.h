#pragma once

#include <stdbool.h>

/**
 * gps_nmea_init - install the UART driver and start the GPS reader task.
 * Must be called after hal_init().
 */
void gps_nmea_init(void);

/**
 * gps_task - FreeRTOS task: reads UART, parses $GPRMC, updates internal state.
 * Pass NULL as pvParameters.
 */
void gps_task(void *pvParameters);

/**
 * gps_get_fix - copy latest GPS position to caller.
 *
 * @param lat    Output: latitude in decimal degrees (positive = North)
 * @param lon    Output: longitude in decimal degrees (positive = East)
 * @param valid  Output: true if last $GPRMC was status 'A' (valid fix)
 */
void gps_get_fix(double *lat, double *lon, bool *valid);
