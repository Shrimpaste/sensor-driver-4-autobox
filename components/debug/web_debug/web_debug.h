#pragma once

/*
 * web_debug.h - WiFi SoftAP + HTTP debug dashboard for the follow-me suitcase.
 *
 * Provides:
 *   - Phone hotspot with web UI for real-time telemetry
 *   - Remote E-stop / ARM / MOTION OFF commands
 *   - Flash CSV logging to SPIFFS (async, low-priority task)
 *   - Fast-path E-stop flag polled by control_task every cycle
 *
 * Usage:
 *   1. web_debug_init(&cfg)         — start WiFi AP + HTTP server + SPIFFS log
 *   2. web_debug_wait_startup()     — block until phone connects + heartbeat
 *   3. Each control cycle:
 *        web_debug_update(&rec)     — publish telemetry snapshot
 *        bool ok = web_debug_motion_allowed()  — check if motion permitted
 *        if (web_debug_estop_pending()) { ... } — fast-path emergency stop
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- UWB frame type (matches bu_uwb parser output) ---- */
typedef enum {
    WDBG_UWB_FRAME_NONE = 0,
    WDBG_UWB_FRAME_TWR,
    WDBG_UWB_FRAME_RANGE_ONLY,
    WDBG_UWB_FRAME_PARSE_ERROR,
} wdbg_uwb_frame_type_t;

/* ---- UWB diagnostic snapshot ---- */
typedef struct {
    bool valid;
    bool filter_initialized;
    bool last_frame_accepted;
    bool bearing_stale;
    wdbg_uwb_frame_type_t frame_type;

    int raw_x_cm;
    int raw_y_cm;
    int raw_distance_cm;
    float raw_fwd_m;
    float raw_left_m;
    float raw_range_m;
    float raw_bearing_rad;

    float filt_fwd_m;
    float filt_left_m;
    float filt_range_m;
    float filt_bearing_rad;
    float speed_mps;
    float bearing_rate_rps;

    uint64_t ts_us;
    uint64_t accepted_ts_us;
    uint32_t frame_count;
    uint32_t twr_count;
    uint32_t range_only_count;
    uint32_t parse_error_count;
    uint32_t outlier_count;
    uint32_t consecutive_outliers;
} wdbg_uwb_t;

/* ---- Follow state (for telemetry display) ---- */
typedef enum {
    WDBG_STATE_IDLE = 0,
    WDBG_STATE_SEARCH,
    WDBG_STATE_FOLLOW,
} wdbg_state_t;

/* ---- Telemetry record: one snapshot per control cycle ---- */
typedef struct {
    uint64_t t_us;

    /* Remote */
    bool remote_client_connected;
    bool remote_estop_latched;
    bool remote_motion_armed;
    bool remote_motion_allowed;
    uint32_t remote_hb_age_ms;
    uint32_t remote_hb_count;

    /* State machine */
    wdbg_state_t state;
    bool target_valid;
    uint32_t target_age_ms;
    float target_distance_m;
    float target_bearing_rad;

    /* UWB */
    wdbg_uwb_t uwb;

    /* Control */
    float algo_v_mps;
    float algo_w_rps;
    float ramp_v_mps;
    float ramp_w_rps;
    float applied_v_mps;
    float applied_w_rps;

    /* Chassis: targets vs actuals */
    float target_left_mps;
    float target_right_mps;
    float cmd_left_us;
    float cmd_right_us;
    float meas_left_mps;
    float meas_right_mps;
    float meas_v_mps;
    float meas_w_rps;
    int chassis_update_ret;
    int chassis_pulse_ret;

    uint32_t log_dropped;
} wdbg_record_t;

/* ---- Configuration ---- */
typedef struct {
    const char *ap_ssid;
    const char *ap_pass;
    int ap_channel;
    int ap_max_conn;
    const char *log_partition_label;   /* SPIFFS partition label, e.g. "logs" */
    const char *log_base_path;         /* VFS mount point, e.g. "/spiffs" */
    const char *log_file_path;         /* full path, e.g. "/spiffs/follow_log.csv" */
    int log_queue_len;
    int log_hz;
    uint64_t hb_timeout_us;           /* heartbeat timeout (microseconds) */
} wdbg_config_t;

/* Sensible defaults matching Kconfig values. */
wdbg_config_t web_debug_default_config(void);

/*
 * Start WiFi SoftAP, HTTP server, and SPIFFS flash log.
 * Must be called once before any other web_debug_* function.
 */
esp_err_t web_debug_init(const wdbg_config_t *cfg);

/*
 * Block until a phone/client connects to the AP AND sends at least one
 * heartbeat. Returns ESP_OK on success, ESP_ERR_TIMEOUT if timeout_ms elapses.
 */
esp_err_t web_debug_wait_startup(uint32_t timeout_ms);

/*
 * Publish a telemetry snapshot. Called by control_task each cycle.
 * Enriches remote fields and enqueues to flash log at configured rate.
 */
void web_debug_update(const wdbg_record_t *rec);

/*
 * Returns true if motion is currently permitted:
 *   - estop_latched == false AND motion_armed == true
 *   - client_connected == true
 *   - heartbeat not timed out
 * On heartbeat timeout, latches E-stop and triggers hard stop.
 * Checked by control_task each cycle.
 */
bool web_debug_motion_allowed(void);

/*
 * Fast-path E-stop: returns true if the HTTP STOP handler fired since the
 * last call. Control_task should check this FIRST, and if true, immediately
 * command zero velocity and skip the rest of the cycle.
 * Reading clears the flag.
 */
bool web_debug_estop_pending(void);

/*
 * Programmatic commands (same as web page buttons).
 */
void web_debug_arm(void);
void web_debug_estop(void);
void web_debug_motion_off(void);

/*
 * Request deletion of the CSV log file. Takes effect asynchronously in the
 * flash log task.
 */
void web_debug_clear_log(void);

#ifdef __cplusplus
}
#endif
