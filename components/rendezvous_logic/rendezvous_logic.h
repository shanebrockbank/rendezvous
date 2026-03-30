#pragma once

#include "espnow_comms.h"

typedef enum {
    RDV_STANDBY,    // no proximity
    RDV_ALIGNING,   // within range, checking orientation
    RDV_DOCKED,     // success — all deltas within threshold
    RDV_FAILED,     // proximity achieved but misaligned
} rdv_state_t;

/**
 * haversine_distance_m - compute great-circle distance between two GPS coords.
 *
 * @return distance in metres
 */
float haversine_distance_m(double lat1, double lon1, double lat2, double lon2);

/**
 * rendezvous_evaluate - evaluate current docking state.
 *
 * Rules:
 *   dist >= ALIGN_THRESHOLD_M                               → RDV_STANDBY
 *   PROX_THRESHOLD_M <= dist < ALIGN_THRESHOLD_M            → RDV_ALIGNING
 *   dist < PROX_THRESHOLD_M, pitch+roll aligned             → RDV_DOCKED
 *   dist < PROX_THRESHOLD_M, pitch or roll out of threshold → RDV_FAILED
 *
 * Note: heading is intentionally excluded from the docking check.
 *
 * @param local   Pointer to local telemetry
 * @param remote  Pointer to remote telemetry
 * @return        Current rendezvous state
 */
rdv_state_t rendezvous_evaluate(const telemetry_packet_t *local,
                                const telemetry_packet_t *remote);

/**
 * rdv_state_name - return a short human-readable string for a state.
 */
const char *rdv_state_name(rdv_state_t state);
