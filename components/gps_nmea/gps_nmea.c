#include "gps_nmea.h"
#include "board_config.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

static const char *TAG = "gps_nmea";

#define GPS_RX_BUF_SIZE  512
#define GPS_LINE_BUF     128

static SemaphoreHandle_t s_fix_mutex = NULL;
static double  s_lat   = 0.0;
static double  s_lon   = 0.0;
static bool    s_valid = false;

/**
 * Convert NMEA ddmm.mmmm format to decimal degrees.
 */
static double nmea_to_decimal(double nmea_val)
{
    int degrees = (int)(nmea_val / 100);
    double minutes = nmea_val - (degrees * 100.0);
    return degrees + (minutes / 60.0);
}

/**
 * Parse $GPRMC sentence and extract lat/lon.
 * Format: $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,...*hh
 * Returns true if sentence has a valid fix (status field == 'A').
 */
static bool parse_gprmc(char *sentence, double *lat_out, double *lon_out)
{
    // Tokenise by commas
    char *fields[13];
    int n = 0;
    char *p = sentence;

    while (n < 13) {
        fields[n++] = p;
        p = strchr(p, ',');
        if (!p) break;
        *p++ = '\0';
    }

    if (n < 7) return false;

    // Field 2: status ('A' = valid, 'V' = void)
    if (fields[2][0] != 'A') return false;

    // Field 3: latitude ddmm.mmmm, field 4: N/S
    double lat_nmea = atof(fields[3]);
    double lat_dd   = nmea_to_decimal(lat_nmea);
    if (fields[4][0] == 'S') lat_dd = -lat_dd;

    // Field 5: longitude dddmm.mmmm, field 6: E/W
    double lon_nmea = atof(fields[5]);
    double lon_dd   = nmea_to_decimal(lon_nmea);
    if (fields[6][0] == 'W') lon_dd = -lon_dd;

    *lat_out = lat_dd;
    *lon_out = lon_dd;
    return true;
}

// ---------------------------------------------------------------------------
// Baud-rate scanner — tries common rates and looks for '$' (NMEA start).
// Cheap NEO clones are often shipped at non-default rates (38400, 115200…).
// Worst-case scan time: 6 rates × ~700 ms = ~4 s.
// ---------------------------------------------------------------------------
static const int k_baud_candidates[] = {9600, 4800, 19200, 38400, 57600, 115200};
#define NUM_BAUD_CANDIDATES  6

static int gps_baud_scan(void)
{
    uint8_t buf[128];

    for (int i = 0; i < NUM_BAUD_CANDIDATES; i++) {
        int baud = k_baud_candidates[i];
        ESP_LOGI(TAG, "[scan] trying %d baud...", baud);
        uart_set_baudrate(GPS_UART_NUM, baud);
        uart_flush(GPS_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(200)); // let output settle at new rate

        // Sample ~500 ms for an NMEA '$' start character
        int  total_bytes = 0;
        bool found_nmea  = false;
        for (int t = 0; t < 5 && !found_nmea; t++) {
            int len = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf) - 1,
                                      pdMS_TO_TICKS(100));
            if (len > 0) {
                total_bytes += len;
                for (int j = 0; j < len; j++) {
                    if (buf[j] == '$') { found_nmea = true; break; }
                }
            }
        }

        if (found_nmea) {
            ESP_LOGI(TAG, "[scan] >>> NMEA found at %d baud (%d bytes sampled)",
                     baud, total_bytes);
            return baud;
        }

        if (total_bytes > 0) {
            // Bytes received but no '$' — almost certainly wrong baud rate (garbled)
            ESP_LOGW(TAG, "[scan] %d bytes at %d baud but no NMEA '$' (garbled — wrong rate?)",
                     total_bytes, baud);
        } else {
            ESP_LOGW(TAG, "[scan] no bytes at %d baud", baud);
        }
    }

    ESP_LOGE(TAG, "[scan] no NMEA at any baud rate");
    ESP_LOGE(TAG, "[scan] likely causes: TX/RX swapped (GPS TX -> ESP32 GPIO%d), "
                  "module not powered, or bad module",
             GPS_UART_RX_PIN);
    uart_set_baudrate(GPS_UART_NUM, GPS_UART_BAUD_RATE); // restore default
    return -1;
}

void gps_nmea_init(void)
{
    s_fix_mutex = xSemaphoreCreateMutex();
    configASSERT(s_fix_mutex);

    int baud = gps_baud_scan();
    if (baud < 0) {
        baud = GPS_UART_BAUD_RATE;
        ESP_LOGW(TAG, "GPS module not detected — defaulting to %d baud", baud);
    }
    ESP_LOGI(TAG, "GPS NMEA parser ready (UART%d @ %d baud)", GPS_UART_NUM, baud);
}

void gps_task(void *pvParameters)
{
    uint8_t buf[GPS_RX_BUF_SIZE];
    char    line[GPS_LINE_BUF];
    int     line_len      = 0;
    int     no_data_ticks = 0;   // counts 100 ms uart_read_bytes intervals with no data
    bool    first_data    = false;

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            // Warn every ~5 s if the UART is completely silent — most likely a wiring issue
            if (++no_data_ticks % 50 == 0) {
                ESP_LOGW(TAG, "No bytes from GPS UART%d in 5s — check GPS TX→ESP32 RX (GPIO%d)",
                         GPS_UART_NUM, GPS_UART_RX_PIN);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        no_data_ticks = 0;
        if (!first_data) {
            ESP_LOGI(TAG, "GPS UART receiving data (%d bytes on first read) — dumping all NMEA", len);
            first_data = true;
        }

        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];

            if (c == '$') {
                line_len = 0;
                line[line_len++] = c;
            } else if (c == '\n' || c == '\r') {
                if (line_len > 6) {
                    line[line_len] = '\0';

                    // Parse position from $GPRMC / $GNRMC only
                    if (strncmp(line, "$GPRMC", 6) == 0 || strncmp(line, "$GNRMC", 6) == 0) {
                        double lat, lon;
                        bool valid = parse_gprmc(line, &lat, &lon);

                        if (xSemaphoreTake(s_fix_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            s_valid = valid;
                            if (valid) {
                                s_lat = lat;
                                s_lon = lon;
                                static bool s_fix_logged = false;
                                if (!s_fix_logged) {
                                    ESP_LOGI(TAG, "Fix: lat=%.6f  lon=%.6f", lat, lon);
                                    s_fix_logged = true;
                                }
                            }
                            xSemaphoreGive(s_fix_mutex);
                        }
                    }
                }
                line_len = 0;
            } else {
                // Only store characters after '$' has been seen (line_len > 0)
                // Prevents garbled partial sentences from the first buffer read
                if (line_len > 0 && line_len < GPS_LINE_BUF - 1) {
                    line[line_len++] = c;
                }
            }
        }
    }
}

void gps_get_fix(double *lat, double *lon, bool *valid)
{
    if (xSemaphoreTake(s_fix_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *lat   = s_lat;
        *lon   = s_lon;
        *valid = s_valid;
        xSemaphoreGive(s_fix_mutex);
    } else {
        *lat   = 0.0;
        *lon   = 0.0;
        *valid = false;
    }
}
