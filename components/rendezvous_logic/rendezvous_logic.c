#include "rendezvous_logic.h"
#include "math.h"

// Thresholds
#define PROX_THRESHOLD_M    5.0f    // must be within 5 m
#define ALIGN_THRESHOLD_M   20.0f   // "aligning" zone (approaching)
#define HDG_THRESHOLD_DEG   20.0f   // max heading difference for docking
#define PITCH_THRESHOLD_DEG 15.0f   // max pitch difference
#define ROLL_THRESHOLD_DEG  15.0f   // max roll difference

#define DEG_TO_RAD  (M_PI / 180.0)
#define EARTH_R_M   6371000.0       // mean Earth radius in metres

float haversine_distance_m(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double a = sin(dlat / 2) * sin(dlat / 2)
             + cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD)
             * sin(dlon / 2) * sin(dlon / 2);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return (float)(EARTH_R_M * c);
}

static float angle_delta(float a, float b)
{
    float d = fabsf(a - b);
    if (d > 180.0f) d = 360.0f - d;
    return d;
}

rdv_state_t rendezvous_evaluate(const telemetry_packet_t *local,
                                const telemetry_packet_t *remote)
{
    // If either node has no GPS fix (lat == 0 and lon == 0), stay in standby
    if ((local->lat == 0.0 && local->lon == 0.0) ||
        (remote->lat == 0.0 && remote->lon == 0.0)) {
        return RDV_STANDBY;
    }

    float dist = haversine_distance_m(local->lat, local->lon,
                                      remote->lat, remote->lon);

    if (dist >= ALIGN_THRESHOLD_M) {
        return RDV_STANDBY;
    }

    if (dist >= PROX_THRESHOLD_M) {
        return RDV_ALIGNING;
    }

    // Within docking range — check orientation
    float hdg_d   = angle_delta(local->heading, remote->heading);
    float pitch_d = angle_delta(local->pitch,   remote->pitch);
    float roll_d  = angle_delta(local->roll,    remote->roll);

    if (hdg_d   <= HDG_THRESHOLD_DEG &&
        pitch_d <= PITCH_THRESHOLD_DEG &&
        roll_d  <= ROLL_THRESHOLD_DEG) {
        return RDV_DOCKED;
    }

    return RDV_FAILED;
}

const char *rdv_state_name(rdv_state_t state)
{
    switch (state) {
        case RDV_STANDBY:  return "STANDBY ";
        case RDV_ALIGNING: return "ALIGNING";
        case RDV_DOCKED:   return "DOCKED  ";
        case RDV_FAILED:   return "MISS    ";
        default:           return "UNKNOWN ";
    }
}
