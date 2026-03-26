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

void gps_nmea_init(void)
{
    s_fix_mutex = xSemaphoreCreateMutex();
    configASSERT(s_fix_mutex);
    // UART was installed in bsp_init(); nothing else needed here.
    ESP_LOGI(TAG, "GPS NMEA parser ready (UART%d)", GPS_UART_NUM);
}

void gps_task(void *pvParameters)
{
    uint8_t buf[GPS_RX_BUF_SIZE];
    char    line[GPS_LINE_BUF];
    int     line_len = 0;

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];

            if (c == '$') {
                // Start of new sentence — reset buffer
                line_len = 0;
                line[line_len++] = c;
            } else if (c == '\n' || c == '\r') {
                if (line_len > 6) {
                    line[line_len] = '\0';

                    // Only process $GPRMC (also accept $GNRMC for multi-constellation modules)
                    if (strncmp(line, "$GPRMC", 6) == 0 || strncmp(line, "$GNRMC", 6) == 0) {
                        ESP_LOGD(TAG, "%s", line);
                        double lat, lon;
                        bool valid = parse_gprmc(line, &lat, &lon);

                        if (xSemaphoreTake(s_fix_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            s_valid = valid;
                            if (valid) {
                                s_lat = lat;
                                s_lon = lon;
                                ESP_LOGI(TAG, "Fix: %.6f, %.6f", lat, lon);
                            }
                            xSemaphoreGive(s_fix_mutex);
                        }
                    }
                }
                line_len = 0;
            } else {
                if (line_len < GPS_LINE_BUF - 1) {
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
