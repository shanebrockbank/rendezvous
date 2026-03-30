#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * distance_estimator_update - feed new observations and refresh the fused estimate.
 *
 * Call from gps_copy_task each time GPS data is refreshed (~500 ms).
 * RSSI is the raw dBm value from the most recent ESP-NOW packet.
 *
 * @param gps_lat_l   Local GPS latitude  (decimal degrees)
 * @param gps_lon_l   Local GPS longitude
 * @param gps_lat_r   Remote GPS latitude
 * @param gps_lon_r   Remote GPS longitude
 * @param gps_valid   true if both local and remote GPS fixes are present
 * @param rssi        RSSI of last received packet (dBm, typically −40 … −100)
 */
void distance_estimator_update(double gps_lat_l, double gps_lon_l,
                               double gps_lat_r, double gps_lon_r,
                               bool gps_valid, int8_t rssi);

/**
 * distance_get_estimate_m - return the latest smoothed fused estimate in metres.
 *
 * Returns −1.0 if no estimate is available yet (no link and no GPS).
 */
float distance_get_estimate_m(void);
