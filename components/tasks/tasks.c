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
#include "icm20948.h"
#include "gps_nmea.h"
#include "rendezvous_logic.h"
#include "distance_estimator.h"

static const char *TAG = "tasks";

#if ACTIVE_MILESTONE >= 4
#define NUM_DISPLAY_PAGES  8   // page 7: RSSI calibration
#elif ACTIVE_MILESTONE >= 3
#define NUM_DISPLAY_PAGES  7   // pages 5+6: local/remote IMU
#else
#define NUM_DISPLAY_PAGES  5
#endif
static volatile int s_display_page = 0;

// ---------------------------------------------------------------------------
// IMU task (Core 0) — reads ICM-20948 every 50 ms
// ---------------------------------------------------------------------------
static void imu_task(void *pvParameters)
{
    // Sensor is initialised in app_main before tasks start
    int log_count = 0;
    while (1) {
        float pitch = 0.0f, roll = 0.0f, heading = 0.0f;
        icm20948_read(&pitch, &roll, &heading);

        if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_local.pitch   = pitch;
            g_local.roll    = roll;
            g_local.heading = heading;
            xSemaphoreGive(g_telem_mutex);
        }

        if (++log_count % 20 == 0) {
            ESP_LOGI(TAG, "IMU pitch=%+6.1f roll=%+6.1f heading=%5.1f",
                     pitch, roll, heading);
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

        double r_lat = 0.0, r_lon = 0.0;
        if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (valid) {
                g_local.lat = lat;
                g_local.lon = lon;
            }
            r_lat = g_remote.lat;
            r_lon = g_remote.lon;
            xSemaphoreGive(g_telem_mutex);
        }

        bool r_valid = (r_lat != 0.0 || r_lon != 0.0);
        distance_estimator_update(lat, lon, r_lat, r_lon,
                                  valid && r_valid, g_remote_rssi);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------------------------------------------------------------------------
// Rendezvous task (Core 0) — evaluates and logs state changes every 500 ms
// ---------------------------------------------------------------------------
#if ACTIVE_MILESTONE >= 5
static void rendezvous_task(void *pvParameters)
{
    rdv_state_t prev_state = RDV_STANDBY;
    while (1) {
        telemetry_packet_t local, remote;
        if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            local  = g_local;
            remote = g_remote;
            xSemaphoreGive(g_telem_mutex);
        }
        float dist = distance_get_estimate_m();
        rdv_state_t state = rendezvous_evaluate(&local, &remote);
        if (state != prev_state) {
            ESP_LOGI(TAG, "RDV STATE: %s  dist=%.1fm  P_delta=%.1f R_delta=%.1f",
                     rdv_state_name(state), dist,
                     fabsf(local.pitch - remote.pitch),
                     fabsf(local.roll  - remote.roll));
            prev_state = state;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

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

        case 4: { // Docking status — fused GPS+RSSI distance
            rdv_state_t state = rendezvous_evaluate(&local, &remote);
            float dist = distance_get_estimate_m();
            if (dist >= 0.0f) {
                snprintf(row0, sizeof(row0), "DIST: %5.1fm    ", dist);
            } else {
                snprintf(row0, sizeof(row0), "DIST: NO DATA   ");
            }
            snprintf(row1, sizeof(row1), "%-16s", rdv_state_name(state));
            break;
        }

        case 5: // Local IMU — pitch, roll, tilt-compensated heading
            // Row 0: "P+123.4 R-056.7 " (P=Pitch, R=Roll, %+6.1f always 6 chars)
            snprintf(row0, sizeof(row0), "P%+6.1f R%+6.1f ", local.pitch, local.roll);
            snprintf(row1, sizeof(row1), "HDG %5.1f       ", local.heading);
            break;

        case 6: // Remote IMU
            if (!link_ok) {
                snprintf(row0, sizeof(row0), "R ---NO LINK--- ");
                snprintf(row1, sizeof(row1), "                ");
            } else {
                snprintf(row0, sizeof(row0), "P%+6.1f R%+6.1f ", remote.pitch, remote.roll);
                snprintf(row1, sizeof(row1), "HDG %5.1f       ", remote.heading);
            }
            break;

        case 7: { // RSSI calibration — median RSSI dBm + component distances
            float gps_d, rssi_d, rssi_dbm;
            distance_get_components(&gps_d, &rssi_d, &rssi_dbm);
            // Row 0 (16 chars): "RS: -38   1.2m "
            snprintf(row0, sizeof(row0), "RS:%4d %6.1fm ", (int)rssi_dbm, rssi_d);
            // Row 1 (16 chars): "GPS:    1.1m    " or "GPS: NO FIX     "
            if (gps_l_valid) {
                snprintf(row1, sizeof(row1), "GPS: %6.1fm    ", gps_d);
            } else {
                snprintf(row1, sizeof(row1), "GPS: NO FIX     ");
            }
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

#if ACTIVE_MILESTONE >= 5
    xTaskCreatePinnedToCore(rendezvous_task, "rdv",       3072, NULL, 3, NULL, 0);
#endif

    ESP_LOGI(TAG, "Tasks started (milestone=%d).", ACTIVE_MILESTONE);
}
