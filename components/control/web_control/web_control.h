#pragma once

/*
 * web_control.h - WiFi SoftAP + HTTP 2D remote control component.
 *
 * Designed for remote_box: web UI sends 2D motion intent (x, y, scale,
 * deadman) via POST /cmd, ESP converts to v/omega and drives chassis.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================ Remote command */

typedef struct {
    uint32_t session;       /* incremented on each CLEAR/ARM */
    uint32_t seq;           /* monotonically increasing */
    uint64_t rx_us;         /* esp_timer timestamp when received */

    float x_norm;           // [-1,+1] left=positive, right=negative
    float y_norm;           // [-1,+1] forward=positive, back=negative
    float scale;            // [0,1] speed multiplier

    bool deadman;           // true = operator actively controlling
    bool valid;             // false = no command received yet or stale
} wctl_cmd_t;

/* ================================================================ Telemetry */

typedef struct {
    uint64_t t_us;

    /* Connection / safety state */
    bool client_connected;
    bool estop_latched;
    bool motion_armed;
    bool motion_allowed;
    uint32_t hb_age_ms;

    /* Session / command */
    uint32_t cmd_session;
    float cmd_x;
    float cmd_y;
    uint32_t cmd_seq;
    uint32_t cmd_age_ms;
    bool cmd_deadman;

    /* Control mapping output */
    float target_v_mps;
    float target_w_rps;
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

    /* Safety */
    const char *safety_reason;

    bool using_encoders;    /* true = closed-loop, false = open-loop feed-forward */
    uint32_t log_dropped;
} wctl_record_t;

/* ================================================================ Configuration */

typedef struct {
    const char *ap_ssid;
    const char *ap_pass;
    int ap_channel;
    int ap_max_conn;
    uint64_t hb_timeout_us;
    const char *log_partition_label;
    const char *log_base_path;
    const char *log_file_path;
    int log_queue_len;
    int log_hz;
    uint32_t cmd_timeout_ms;
} wctl_config_t;

wctl_config_t web_control_default_config(void);

/* ================================================================ Core API */

esp_err_t web_control_init(const wctl_config_t *cfg);
esp_err_t web_control_wait_startup(uint32_t timeout_ms);
void web_control_update(const wctl_record_t *rec);
bool web_control_motion_allowed(void);
bool web_control_estop_pending(void);
bool web_control_get_remote_cmd(wctl_cmd_t *cmd);

/* Get current session ID (incremented on each CLEAR/ARM). */
uint32_t web_control_get_session(void);

void web_control_arm(void);
void web_control_estop(void);
void web_control_motion_off(void);
void web_control_clear_log(void);

#ifdef __cplusplus
}
#endif
