#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "bsp.h"
#include "board_config.h"
#include "espnow_comms.h"
#include "gps_nmea.h"
#include "lcd_i2c.h"
#include "icm20948.h"
#include "tasks.h"

static const char *TAG = "main";

// Shared telemetry state — extern'd by espnow_comms.h and tasks
telemetry_packet_t g_local  = {0};
telemetry_packet_t g_remote = {0};
SemaphoreHandle_t  g_telem_mutex = NULL;

void app_main(void)
{
    ESP_LOGI(TAG, "Spaceium Rendezvous booting (node_id=%d)", NODE_ID);

    // Hardware bringup — must be first
    bsp_init();

#if ACTIVE_MILESTONE >= 3
    icm20948_init(bsp_get_i2c_port2());
#endif
#if ACTIVE_MILESTONE >= 2
    lcd_init(bsp_get_i2c_port(), LCD_I2C_ADDR);
#endif

    g_telem_mutex = xSemaphoreCreateMutex();
    configASSERT(g_telem_mutex);

    espnow_comms_init();
    gps_nmea_init();
    tasks_start();
}
