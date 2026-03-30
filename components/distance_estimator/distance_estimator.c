#include "distance_estimator.h"
#include "board_config.h"

#include "esp_log.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "dist_est";

// ---------------------------------------------------------------------------
// RSSI log-distance path loss model
//   d = 10 ^ ((RSSI_REF_1M - rssi) / (10 * n))
//   n ≈ 2.0 in open air  (increase to 2.5–3.0 indoors)
// ---------------------------------------------------------------------------
#define RSSI_PATH_LOSS_N    2.0f

// ---------------------------------------------------------------------------
// Fusion blend zone
//   > BLEND_FAR_M  → GPS only (weight = 1.0)
//   < BLEND_NEAR_M → RSSI only (weight = 0.0)
//   in between     → linear interpolation
// ---------------------------------------------------------------------------
#define BLEND_FAR_M     20.0f
#define BLEND_NEAR_M     2.0f

// ---------------------------------------------------------------------------
// EMA alphas
//   GPS distance  : α = 0.5  → ~720 ms time constant at 500 ms update rate
//   Fused output  : α = 0.4  → ~980 ms time constant at 500 ms update rate
// ---------------------------------------------------------------------------
#define EMA_ALPHA_GPS   0.5f
#define EMA_ALPHA_OUT   0.4f

// ---------------------------------------------------------------------------
// RSSI median — 5-sample ring buffer
//   At 200 ms heartbeat rate this covers 1 s of data.
//   Median rejects outlier spikes better than EMA.
// ---------------------------------------------------------------------------
#define RSSI_MED_N  5

static int8_t s_rssi_buf[RSSI_MED_N] = {-100, -100, -100, -100, -100};
static int    s_rssi_idx = 0;

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------
static bool  s_init          = false;   // true after first real-data update
static bool  s_rssi_seeded   = false;   // true after first non-(-100) RSSI
static float s_gps_ema       = 0.0f;
static volatile float s_output = -1.0f; // exposed via distance_get_estimate_m()

// ---------------------------------------------------------------------------
// Haversine great-circle distance (static — keep dependency-free)
// ---------------------------------------------------------------------------
#define EARTH_R_M   6371000.0
#define DEG2RAD(d)  ((d) * (M_PI / 180.0))

static float haversine_m(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = DEG2RAD(lat2 - lat1);
    double dlon = DEG2RAD(lon2 - lon1);
    double a = sin(dlat / 2) * sin(dlat / 2)
             + cos(DEG2RAD(lat1)) * cos(DEG2RAD(lat2))
             * sin(dlon / 2) * sin(dlon / 2);
    return (float)(EARTH_R_M * 2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
}

// ---------------------------------------------------------------------------
// 5-element insertion sort → return median
// ---------------------------------------------------------------------------
static float rssi_median_update(int8_t new_val)
{
    // Pre-fill buffer on first real contact so median is immediately valid
    // rather than needing 5 packets to wash out the -100 sentinel
    if (!s_rssi_seeded && new_val != -100) {
        for (int i = 0; i < RSSI_MED_N; i++) s_rssi_buf[i] = new_val;
        s_rssi_seeded = true;
    }

    s_rssi_buf[s_rssi_idx] = new_val;
    s_rssi_idx = (s_rssi_idx + 1) % RSSI_MED_N;

    int8_t tmp[RSSI_MED_N];
    memcpy(tmp, s_rssi_buf, RSSI_MED_N);

    for (int i = 1; i < RSSI_MED_N; i++) {
        int8_t key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    return (float)tmp[RSSI_MED_N / 2];
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void distance_estimator_update(double gps_lat_l, double gps_lon_l,
                               double gps_lat_r, double gps_lon_r,
                               bool gps_valid, int8_t rssi)
{
    // Don't initialise the EMA from a cold "no link, no GPS" state.
    // rssi = -100 is the sentinel value meaning no packet received yet.
    // Keep s_output = -1.0 (display shows NO DATA) until real data arrives.
    if (rssi == -100 && !gps_valid) {
        return;
    }

    // --- RSSI distance ---
    float rssi_med = rssi_median_update(rssi);
    // d = 10^((rssi_0 − rssi) / (10 × n))
    float rssi_dist = powf(10.0f,
                           ((float)RSSI_REF_1M - rssi_med) / (10.0f * RSSI_PATH_LOSS_N));

    // --- GPS distance (EMA-smoothed haversine) ---
    float gps_dist = 0.0f;
    if (gps_valid) {
        float raw = haversine_m(gps_lat_l, gps_lon_l, gps_lat_r, gps_lon_r);
        if (!s_init) {
            s_gps_ema = raw;
        } else {
            s_gps_ema = EMA_ALPHA_GPS * raw + (1.0f - EMA_ALPHA_GPS) * s_gps_ema;
        }
        gps_dist = s_gps_ema;
    }

    // --- Linear fusion blend ---
    // w_gps: 1.0 = GPS only, 0.0 = RSSI only
    float w_gps = 0.0f;
    float fused;
    if (!gps_valid) {
        fused = rssi_dist;
    } else {
        if (s_gps_ema >= BLEND_FAR_M) {
            w_gps = 1.0f;
        } else if (s_gps_ema <= BLEND_NEAR_M) {
            w_gps = 0.0f;
        } else {
            w_gps = (s_gps_ema - BLEND_NEAR_M) / (BLEND_FAR_M - BLEND_NEAR_M);
        }
        fused = w_gps * gps_dist + (1.0f - w_gps) * rssi_dist;
    }

    // --- Output EMA ---
    float out;
    if (!s_init) {
        out    = fused;
        s_init = true;
    } else {
        out = EMA_ALPHA_OUT * fused + (1.0f - EMA_ALPHA_OUT) * s_output;
    }

    s_output = out;

    ESP_LOGD(TAG, "rssi=%+4ddBm(med=%+4.0f)  rssi_d=%6.2fm  gps_d=%6.2fm(%s)  gps_wt=%3.0f%%  out=%6.2fm",
             (int)rssi, rssi_med, rssi_dist, gps_dist,
             gps_valid ? "valid " : "no fix", w_gps * 100.0f, out);
}

float distance_get_estimate_m(void)
{
    return s_output;
}
