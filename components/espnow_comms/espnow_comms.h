#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * Shared telemetry packet exchanged between the two ESP32 nodes.
 * Packed to ensure consistent wire layout regardless of compiler padding.
 */
typedef struct {
    uint8_t  node_id;        // 0 = chaser, 1 = target
    float    pitch;          // degrees, from MPU6050
    float    roll;           // degrees, from MPU6050
    float    heading;        // degrees 0-360, from HMC5883L
    double   lat;            // GPS latitude  (0.0 until GPS fix)
    double   lon;            // GPS longitude
    uint32_t timestamp_ms;   // sender's uptime in ms — receiver uses for staleness
    uint32_t seq;            // packet sequence number
    uint8_t  button_state;   // 1 if BOOT button currently pressed
} __attribute__((packed)) telemetry_packet_t;

/**
 * espnow_comms_init - initialise WiFi + ESP-NOW, register peer + callbacks.
 * Must be called after hal_init().
 */
void espnow_comms_init(void);

/**
 * espnow_tx_task - FreeRTOS task: broadcasts g_local every 200 ms.
 * Pass NULL as pvParameters.
 */
void espnow_tx_task(void *pvParameters);

// Shared state — defined in main.c, extern here for use by comms module
extern telemetry_packet_t g_local;
extern telemetry_packet_t g_remote;
extern SemaphoreHandle_t  g_telem_mutex;
