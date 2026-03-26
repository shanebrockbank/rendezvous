#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "bsp.h"
#include "board_config.h"
#include "espnow_comms.h"
#include "gps_nmea.h"
#include "tasks.h"

static const char *TAG = "main";

// Shared telemetry state — extern'd by espnow_comms.h and tasks
telemetry_packet_t g_local  = {0};
telemetry_packet_t g_remote = {0};
SemaphoreHandle_t  g_telem_mutex = NULL;

void app_main(void)
{
    ESP_LOGI(TAG, "Spaceium Rendezvous booting (node_id=%d)", NODE_ID);

    bsp_init();

    g_telem_mutex = xSemaphoreCreateMutex();
    configASSERT(g_telem_mutex);

    espnow_comms_init();
    gps_nmea_init();
    tasks_start();
}
