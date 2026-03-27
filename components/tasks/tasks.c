#include "tasks.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "bsp.h"
#include "board_config.h"
#include "espnow_comms.h"
#include "lcd_i2c.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "gps_nmea.h"
#include "rendezvous_logic.h"

static const char *TAG = "tasks";

#define NUM_DISPLAY_PAGES  5
static volatile int s_display_page = 0;

// ---------------------------------------------------------------------------
// IMU task (Core 0) — reads MPU6050 + HMC5883L every 50 ms
// ---------------------------------------------------------------------------
static void imu_task(void *pvParameters)
{
    // Sensors are initialised in app_main before tasks start
    while (1) {
        float pitch = 0.0f, roll = 0.0f, heading = 0.0f;
        mpu6050_read(&pitch, &roll);
        hmc5883l_read(&heading);

        if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_local.pitch   = pitch;
            g_local.roll    = roll;
            g_local.heading = heading;
            xSemaphoreGive(g_telem_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------------------------------------------------------------------------
// GPS tasks (Core 1)
// ---------------------------------------------------------------------------
static void gps_nmea_task(void *pvParameters)
{
    gps_task(NULL); // runs the NMEA parser loop, never returns
}

static void gps_copy_task(void *pvParameters)
{
    while (1) {
        double lat, lon;
        bool   valid;
        gps_get_fix(&lat, &lon, &valid);

        if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (valid) {
                g_local.lat = lat;
                g_local.lon = lon;
            }
            xSemaphoreGive(g_telem_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------------------------------------------------------------------------
// Display task (Core 1) — updates LCD every 250 ms, button cycles pages
// ---------------------------------------------------------------------------
static void display_task(void *pvParameters)
{
    // LCD initialised in app_main — just show splash
    // Boot splash
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_write_string("  AT SPACEIUM  ");
    lcd_set_cursor(1, 0);
    lcd_write_string(" RENDEZVOUS SYS");
    vTaskDelay(pdMS_TO_TICKS(2000));

    char     row0[17], row1[17];
    int      prev_button   = 1;
    uint32_t local_btn_ms  = 0;   // latch timestamp for local button press

    while (1) {
        // Page scroll button edge detection (HAL_BUTTON_PAGE_PIN, active low)
        int btn = gpio_get_level(HAL_BUTTON_PAGE_PIN);
        if (btn == 0 && prev_button == 1) {
            s_display_page = (s_display_page + 1) % NUM_DISPLAY_PAGES;
        }
        prev_button = btn;

        // Snapshot telemetry
        telemetry_packet_t local, remote;
        if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            local  = g_local;
            remote = g_remote;
            xSemaphoreGive(g_telem_mutex);
        }

        uint32_t now_ms  = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint32_t rx_age  = now_ms - g_remote_rx_ms;
        bool     link_ok = (remote.seq > 0) && (rx_age < 1000);

        // Latch local button press — update timestamp while held, show for 600 ms after release
        if (!gpio_get_level(HAL_BUTTON_PIN)) {
            local_btn_ms = now_ms;
        }
        bool local_btn  = (local_btn_ms > 0 && (now_ms - local_btn_ms) < 600);

        // Latch remote button indicator for 600 ms so a quick tap is always visible
        bool remote_btn = remote.button_state ||
                          (g_remote_btn_ms > 0 && (now_ms - g_remote_btn_ms) < 600);

        double lat_l, lon_l, lat_r, lon_r;
        bool   gps_l_valid, gps_r_valid;
        gps_get_fix(&lat_l, &lon_l, &gps_l_valid);
        lat_r       = remote.lat;
        lon_r       = remote.lon;
        gps_r_valid = link_ok && (remote.lat != 0.0 || remote.lon != 0.0);

        switch (s_display_page) {

        case 0: { // Button alerts + signal health
            const char *l_btn = local_btn ? "***PRESS***" : "           ";
            snprintf(row0, sizeof(row0), "L %s", l_btn);

            // Row 1: remote button press latched 600ms; otherwise show link quality
            if (remote_btn) {
                snprintf(row1, sizeof(row1), "R ***PRESS***   ");
            } else if (!link_ok) {
                snprintf(row1, sizeof(row1), "R  NO LINK      ");
            } else {
                const char *quality = rx_age < 250 ? "GOOD" :
                                      rx_age < 600 ? "FAIR" : "POOR";
                snprintf(row1, sizeof(row1), "R %s %4lums  ", quality, (unsigned long)rx_age);
            }
            break;
        }

        case 1: // Local GPS
            if (gps_l_valid) {
                snprintf(row0, sizeof(row0), "L %9.4f%c", fabs(lat_l), lat_l >= 0 ? 'N' : 'S');
                snprintf(row1, sizeof(row1), "  %9.4f%c", fabs(lon_l), lon_l >= 0 ? 'E' : 'W');
            } else {
                snprintf(row0, sizeof(row0), "L GPS NO FIX    ");
                snprintf(row1, sizeof(row1), "  Waiting...    ");
            }
            break;

        case 2: // Remote GPS
            if (gps_r_valid) {
                snprintf(row0, sizeof(row0), "R %9.4f%c", fabs(lat_r), lat_r >= 0 ? 'N' : 'S');
                snprintf(row1, sizeof(row1), "  %9.4f%c", fabs(lon_r), lon_r >= 0 ? 'E' : 'W');
            } else if (!link_ok) {
                snprintf(row0, sizeof(row0), "R ---NO LINK--- ");
                snprintf(row1, sizeof(row1), "                ");
            } else {
                snprintf(row0, sizeof(row0), "R GPS NO FIX    ");
                snprintf(row1, sizeof(row1), "                ");
            }
            break;

        case 3: // Link status
            snprintf(row0, sizeof(row0), "SEQ: %05lu      ", (unsigned long)remote.seq);
            if (link_ok) {
                uint32_t age = rx_age;
                const char *quality = age < 250  ? "GOOD" :
                                      age < 600  ? "FAIR" : "POOR";
                snprintf(row1, sizeof(row1), "%s  %4lums   ", quality, (unsigned long)age);
            } else {
                snprintf(row1, sizeof(row1), "NO LINK         ");
            }
            break;

        case 4: { // Docking status
            rdv_state_t state = rendezvous_evaluate(&local, &remote);
            if (gps_l_valid && gps_r_valid) {
                float dist = haversine_distance_m(lat_l, lon_l, lat_r, lon_r);
                snprintf(row0, sizeof(row0), "DIST: %5.1fm    ", dist);
            } else {
                snprintf(row0, sizeof(row0), "DIST: NO GPS    ");
            }
            snprintf(row1, sizeof(row1), "%-16s", rdv_state_name(state));
            break;
        }

        default:
            snprintf(row0, sizeof(row0), "                ");
            snprintf(row1, sizeof(row1), "                ");
            break;
        }

        lcd_set_cursor(0, 0);
        lcd_write_string(row0);
        lcd_set_cursor(1, 0);
        lcd_write_string(row1);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------------------------------------------------------------------------
// tasks_start — create and pin tasks up to ACTIVE_MILESTONE
// Increment ACTIVE_MILESTONE in board_config.h to enable more hardware.
// ---------------------------------------------------------------------------
void tasks_start(void)
{
    // M1: ESP-NOW TX always runs
    xTaskCreatePinnedToCore(espnow_tx_task, "espnow_tx", 4096, NULL, 5, NULL, 0);

#if ACTIVE_MILESTONE >= 2
    xTaskCreatePinnedToCore(display_task,   "display",   6144, NULL, 3, NULL, 1);
#endif

#if ACTIVE_MILESTONE >= 3
    xTaskCreatePinnedToCore(imu_task,       "imu",       4096, NULL, 5, NULL, 0);
#endif

#if ACTIVE_MILESTONE >= 4
    xTaskCreatePinnedToCore(gps_nmea_task,  "gps_nmea",  4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(gps_copy_task,  "gps_copy",  2048, NULL, 3, NULL, 1);
#endif

    ESP_LOGI(TAG, "Tasks started (milestone=%d).", ACTIVE_MILESTONE);
}
