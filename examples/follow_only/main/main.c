/*
 * follow_only_uwb_debug_estop_flashlog
 *
 * UWB-only 调试固件，使用 web_debug 组件提供：
 *   - WiFi SoftAP + 手机网页实时遥测
 *   - 远程 E-stop / ARM / MOTION OFF
 *   - Flash CSV 日志
 *   - E-stop 快速路径
 *
 * 本文件保留：UWB 滤波、跟随状态机、IMU 航向闭环、底盘控制。
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "bu_uwb.h"
#include "chassis.h"
#include "web_debug.h"

#include "driver/i2c_master.h"
#include "imu_i2c.h"

static const char *TAG = "follow_only_dbg";

#define M_PI 3.14159265358979323846
#define DEG2RAD(d) ((float)(d) * (float)M_PI / 180.0f)
#define RAD2DEG(r) ((float)(r) * 180.0f / (float)M_PI)

/* ---------------- Compile-time debug switches ---------------- */
#define FR_ENABLE_UWB            1
#define FR_ENABLE_IMU            1
#define FR_ENABLE_CHASSIS        1
#define FR_ENABLE_MOTION         1

#define FR_USE_MOTION_DEBUG_LIMITS       0
#define FR_MOTION_DEBUG_MAX_LINEAR_MPS   0.15f
#define FR_MOTION_DEBUG_MAX_ANGULAR_RPS  0.35f

#define FR_USE_DIRECT_PULSE_CONTROL      0

#define FR_LEFT_INVERT false
#define FR_RIGHT_INVERT true
#define FR_LEFT_ENC_INVERT true
#define FR_RIGHT_ENC_INVERT true
#define FR_UWB_LEFT_SIGN 1.0f
#define FR_IMU_YAW_SIGN -1.0f

/* UWB smoothing / outlier rejection. */
#define FR_UWB_SMOOTH_TAU_S          0.35f
#define FR_UWB_MAX_TARGET_SPEED_MPS  3.0f
#define FR_UWB_JUMP_MARGIN_M         0.30f
#define FR_UWB_REINIT_OUTLIERS       5

#define FR_UWB_RANGE_ONLY_REFRESH_TARGET 0

/* ---------------- Utilities ---------------- */
static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float wrap_pi(float a)
{
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static float ramp(float current, float target, float rate, float dt)
{
    if (rate <= 0.0f || dt <= 0.0f) return target;
    float step = rate * dt;
    float d = target - current;
    if (d > step) d = step;
    else if (d < -step) d = -step;
    return current + d;
}

static int clamp_int(int x, int lo, int hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static int wheel_speed_to_pulse_us(float wheel_mps,
                                   float max_wheel_mps,
                                   int esc_min_us,
                                   int esc_mid_us,
                                   int esc_max_us,
                                   bool invert)
{
    if (max_wheel_mps <= 1e-6f) {
        return esc_mid_us;
    }

    if (invert) {
        wheel_mps = -wheel_mps;
    }

    wheel_mps = clampf(wheel_mps, -max_wheel_mps, max_wheel_mps);

    float pulse = (float)esc_mid_us;
    if (wheel_mps >= 0.0f) {
        pulse += (wheel_mps / max_wheel_mps) * (float)(esc_max_us - esc_mid_us);
    } else {
        pulse += (wheel_mps / max_wheel_mps) * (float)(esc_mid_us - esc_min_us);
    }

    return clamp_int((int)(pulse + 0.5f), esc_min_us, esc_max_us);
}

static void velocity_to_direct_pulses(float v_mps,
                                      float omega_rps,
                                      float track_width_m,
                                      float max_wheel_mps,
                                      int esc_min_us,
                                      int esc_mid_us,
                                      int esc_max_us,
                                      int *left_us,
                                      int *right_us,
                                      float *left_mps,
                                      float *right_mps)
{
    if (max_wheel_mps <= 1e-6f) {
        if (left_us) *left_us = esc_mid_us;
        if (right_us) *right_us = esc_mid_us;
        if (left_mps) *left_mps = 0.0f;
        if (right_mps) *right_mps = 0.0f;
        return;
    }

    const float half_track = 0.5f * track_width_m;
    float l = v_mps - omega_rps * half_track;
    float r = v_mps + omega_rps * half_track;

    float m = fabsf(l);
    if (fabsf(r) > m) {
        m = fabsf(r);
    }
    if (m > max_wheel_mps) {
        const float scale = max_wheel_mps / m;
        l *= scale;
        r *= scale;
    }

    l = clampf(l, -max_wheel_mps, max_wheel_mps);
    r = clampf(r, -max_wheel_mps, max_wheel_mps);

    if (left_mps) *left_mps = l;
    if (right_mps) *right_mps = r;
    if (left_us) {
        *left_us = wheel_speed_to_pulse_us(l, max_wheel_mps, esc_min_us,
                                           esc_mid_us, esc_max_us, FR_LEFT_INVERT);
    }
    if (right_us) {
        *right_us = wheel_speed_to_pulse_us(r, max_wheel_mps, esc_min_us,
                                            esc_mid_us, esc_max_us, FR_RIGHT_INVERT);
    }
}

/* ---------------- Follow state ---------------- */
typedef enum {
    FOLLOW_STATE_IDLE = 0,
    FOLLOW_STATE_SEARCH,
    FOLLOW_STATE_FOLLOW,
} follow_state_t;

static const char *state_name(follow_state_t s)
{
    switch (s) {
    case FOLLOW_STATE_IDLE:   return "IDLE";
    case FOLLOW_STATE_SEARCH: return "SEARCH";
    case FOLLOW_STATE_FOLLOW: return "FOLLOW";
    default:                  return "?";
    }
}

/* ---------------- Shared state (UWB + IMU) ---------------- */
typedef struct {
    SemaphoreHandle_t lock;

    float tgt_distance_m;
    float tgt_bearing_rad;
    uint64_t tgt_ts_us;

    wdbg_uwb_t uwb;
} shared_t;

static shared_t g_shared;
static imu_i2c_t s_imu;
static bool s_imu_ok = false;
static chassis_t s_chassis;

static void lock(void) { xSemaphoreTake(g_shared.lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(g_shared.lock); }

/* ---------------- UWB filtering ---------------- */
static void uwb_publish_twr(int x_cm, int y_cm, int distance_cm)
{
    const uint64_t t = now_us();
    const float fwd_m = (float)y_cm / 100.0f;
    const float left_m = FR_UWB_LEFT_SIGN * (float)x_cm / 100.0f;
    const float range_m = (distance_cm > 0) ? (float)distance_cm / 100.0f
                                           : sqrtf(fwd_m * fwd_m + left_m * left_m);
    float bearing = 0.0f;
    if (fabsf(fwd_m) > 1e-3f || fabsf(left_m) > 1e-3f) {
        bearing = atan2f(left_m, fwd_m);
    }

    lock();
    wdbg_uwb_t *u = &g_shared.uwb;

    const uint64_t prev_acc_ts = u->accepted_ts_us;
    const float prev_fwd = u->filt_fwd_m;
    const float prev_left = u->filt_left_m;
    const float prev_bearing = u->filt_bearing_rad;
    const bool had_filter = u->filter_initialized;
    float dt = 0.0f;
    if (prev_acc_ts != 0 && t > prev_acc_ts) {
        dt = (float)(t - prev_acc_ts) / 1e6f;
    }

    u->frame_count++;
    u->twr_count++;
    u->frame_type = WDBG_UWB_FRAME_TWR;
    u->ts_us = t;
    u->raw_x_cm = x_cm;
    u->raw_y_cm = y_cm;
    u->raw_distance_cm = distance_cm;
    u->raw_fwd_m = fwd_m;
    u->raw_left_m = left_m;
    u->raw_range_m = range_m;
    u->raw_bearing_rad = bearing;
    u->bearing_stale = false;

    bool accept = true;
    bool reinit = !had_filter || dt <= 0.0f || dt > 1.5f;

    if (!reinit) {
        const float jump = sqrtf((fwd_m - prev_fwd) * (fwd_m - prev_fwd) +
                                 (left_m - prev_left) * (left_m - prev_left));
        const float max_jump = FR_UWB_MAX_TARGET_SPEED_MPS * dt + FR_UWB_JUMP_MARGIN_M;
        if (jump > max_jump && u->consecutive_outliers < FR_UWB_REINIT_OUTLIERS) {
            accept = false;
        } else if (u->consecutive_outliers >= FR_UWB_REINIT_OUTLIERS) {
            reinit = true;
        }
    }

    u->last_frame_accepted = accept;

    if (accept) {
        float new_fwd = fwd_m;
        float new_left = left_m;

        if (!reinit) {
            float alpha = dt / (FR_UWB_SMOOTH_TAU_S + dt);
            alpha = clampf(alpha, 0.05f, 0.85f);
            new_fwd = prev_fwd + alpha * (fwd_m - prev_fwd);
            new_left = prev_left + alpha * (left_m - prev_left);
        }

        u->filter_initialized = true;
        u->valid = true;
        u->consecutive_outliers = 0;
        u->filt_fwd_m = new_fwd;
        u->filt_left_m = new_left;
        u->filt_range_m = sqrtf(new_fwd * new_fwd + new_left * new_left);
        u->filt_bearing_rad = atan2f(new_left, new_fwd);
        u->accepted_ts_us = t;

        if (had_filter && dt > 1e-3f) {
            const float move = sqrtf((new_fwd - prev_fwd) * (new_fwd - prev_fwd) +
                                     (new_left - prev_left) * (new_left - prev_left));
            u->speed_mps = move / dt;
            u->bearing_rate_rps = wrap_pi(u->filt_bearing_rad - prev_bearing) / dt;
        } else {
            u->speed_mps = 0.0f;
            u->bearing_rate_rps = 0.0f;
        }

        g_shared.tgt_distance_m = u->filt_range_m;
        g_shared.tgt_bearing_rad = u->filt_bearing_rad;
        g_shared.tgt_ts_us = t;
    } else {
        u->outlier_count++;
        u->consecutive_outliers++;
    }

    unlock();
}

static void uwb_publish_range_only(float distance_m)
{
    const uint64_t t = now_us();

    lock();
    wdbg_uwb_t *u = &g_shared.uwb;
    u->frame_count++;
    u->range_only_count++;
    u->frame_type = WDBG_UWB_FRAME_RANGE_ONLY;
    u->ts_us = t;
    u->raw_distance_cm = (int)(distance_m * 100.0f);
    u->raw_range_m = distance_m;
    u->bearing_stale = true;
    u->last_frame_accepted = true;

#if FR_UWB_RANGE_ONLY_REFRESH_TARGET
    g_shared.tgt_distance_m = distance_m;
    g_shared.tgt_ts_us = t;
#else
    (void)distance_m;
#endif
    unlock();
}

static void uwb_publish_parse_error(void)
{
    lock();
    g_shared.uwb.frame_count++;
    g_shared.uwb.parse_error_count++;
    g_shared.uwb.frame_type = WDBG_UWB_FRAME_PARSE_ERROR;
    g_shared.uwb.ts_us = now_us();
    g_shared.uwb.last_frame_accepted = false;
    unlock();
}

/* ---------------- UWB task ---------------- */
static void uwb_task(void *arg)
{
    (void)arg;
    char line[BU_UWB_LINE_MAX];

    while (1) {
        if (bu_uwb_read_line(line, sizeof(line), 200) != ESP_OK) {
            continue;
        }

        bu_uwb_twr_reading_t twr = {0};
        bu_uwb_distance_t dist = {0};

        if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
            uwb_publish_twr(twr.x_cm, twr.y_cm, twr.distance_cm);
        } else if (bu_uwb_parse_distance_line(line, &dist) && dist.valid) {
            uwb_publish_range_only(dist.distance_m);
        } else {
            uwb_publish_parse_error();
        }
    }
}

/* ---------------- IMU ---------------- */
static bool imu_read_yaw(float *yaw_rad)
{
#if FR_ENABLE_IMU
    if (!s_imu_ok) return false;
    imu_i2c_reading_t r;
    memset(&r, 0, sizeof(r));
    if (imu_i2c_read_all(&s_imu, &r) != ESP_OK || !r.valid) return false;
    *yaw_rad = FR_IMU_YAW_SIGN * DEG2RAD(r.euler_deg[2]);
    return true;
#else
    (void)yaw_rad;
    return false;
#endif
}

/* ---------------- Control task ---------------- */
static void control_task(void *arg)
{
    chassis_t *chassis = (chassis_t *)arg;

    const float follow_distance_m = CONFIG_FOLLOW_ONLY_FOLLOW_DISTANCE_MM / 1000.0f;
    const float stop_band_m = CONFIG_FOLLOW_ONLY_STOP_BAND_MM / 1000.0f;
    const float max_linear_mps = CONFIG_FOLLOW_ONLY_MAX_LINEAR_MMPS / 1000.0f;
    const float max_angular_rps = CONFIG_FOLLOW_ONLY_MAX_ANGULAR_MRADPS / 1000.0f;
    const float track_width_m = CONFIG_FOLLOW_ONLY_TRACK_WIDTH_MM / 1000.0f;
    const float max_wheel_mps = CONFIG_FOLLOW_ONLY_MAX_WHEEL_SPEED_MMPS / 1000.0f;
    const int esc_min_us = CONFIG_FOLLOW_ONLY_ESC_MIN_US;
    const int esc_mid_us = CONFIG_FOLLOW_ONLY_ESC_MID_US;
    const int esc_max_us = CONFIG_FOLLOW_ONLY_ESC_MAX_US;
    const float kp_dist = CONFIG_FOLLOW_ONLY_KP_DIST / 1000.0f;
    const float kp_bear = CONFIG_FOLLOW_ONLY_KP_BEAR / 1000.0f;
    const float max_lin_accel = 0.8f;
    const float max_lin_decel = 2.0f;
    const float max_ang_accel = 6.0f;
    const float search_rps = CONFIG_FOLLOW_ONLY_SEARCH_ANGULAR_MRADPS / 1000.0f;
    const float search_timeout_s = (float)CONFIG_FOLLOW_ONLY_SEARCH_TIMEOUT_S;
    const uint64_t target_fresh_us = (uint64_t)CONFIG_FOLLOW_ONLY_TARGET_FRESH_MS * 1000ULL;
    const float heading_kp = CONFIG_FOLLOW_ONLY_HEADING_KP_MILLI / 1000.0f;

    follow_state_t state = FOLLOW_STATE_IDLE;
    float ramp_v_cmd = 0.0f;
    float ramp_w_cmd = 0.0f;
    float lost_timer_s = 0.0f;
    float search_timer_s = 0.0f;
    float last_known_bearing = 0.0f;
    bool has_last_known = false;
    float yaw_ref = 0.0f;
    bool yaw_ref_set = false;

    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FOLLOW_ONLY_CONTROL_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t prev_us = now_us();
    int serial_log_div = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        const uint64_t t = now_us();
        const float dt = (float)(t - prev_us) / 1e6f;
        prev_us = t;

        /* ---- Fast-path E-stop: bypass all sensor/algorithm logic ---- */
        if (web_debug_estop_pending()) {
            ramp_v_cmd = 0.0f;
            ramp_w_cmd = 0.0f;
            chassis_stop(chassis);  /* immediate mid-pulse + PID reset */
            state = FOLLOW_STATE_IDLE;
            continue;
        }

        bool tgt_valid = false;
        float tgt_dist = 0.0f;
        float tgt_bear = 0.0f;
        uint32_t tgt_age_ms = 0xffffffffu;
        wdbg_uwb_t uwb;
        memset(&uwb, 0, sizeof(uwb));

        lock();
        if (g_shared.tgt_ts_us != 0 && t >= g_shared.tgt_ts_us) {
            uint64_t age_us = t - g_shared.tgt_ts_us;
            tgt_valid = age_us < target_fresh_us;
            tgt_age_ms = (uint32_t)(age_us / 1000ULL);
        }
        tgt_dist = g_shared.tgt_distance_m;
        tgt_bear = g_shared.tgt_bearing_rad;
        uwb = g_shared.uwb;
        unlock();

        if (tgt_valid) {
            lost_timer_s = 0.0f;
            search_timer_s = 0.0f;
            last_known_bearing = tgt_bear;
            has_last_known = true;
        } else {
            lost_timer_s += dt;
            if (state == FOLLOW_STATE_SEARCH) search_timer_s += dt;
        }

        float algo_v = 0.0f;
        float algo_w = 0.0f;

        if (!tgt_valid && lost_timer_s > 0.5f) {
            if (has_last_known && search_timer_s <= search_timeout_s) {
                state = FOLLOW_STATE_SEARCH;
                float dir = (last_known_bearing >= 0.0f) ? 1.0f : -1.0f;
                algo_v = 0.0f;
                algo_w = dir * search_rps;
                yaw_ref_set = false;
            } else {
                state = FOLLOW_STATE_IDLE;
                algo_v = 0.0f;
                algo_w = 0.0f;
                has_last_known = false;
                yaw_ref_set = false;
            }
        } else if (tgt_valid) {
            state = FOLLOW_STATE_FOLLOW;

            /* Speed geometry: use forward gap for linear, bearing for angular. */
            const float fwd_gap = uwb.filt_fwd_m - follow_distance_m;
            if (fwd_gap > 0.0f) {
                algo_v = kp_dist * fwd_gap;
            } else if (-fwd_gap <= stop_band_m) {
                algo_v = 0.0f;
            } else {
                algo_v = 0.0f;  /* too close, don't reverse */
            }
            algo_v = clampf(algo_v, 0.0f, max_linear_mps);

            algo_w = kp_bear * tgt_bear;
            algo_w = clampf(algo_w, -max_angular_rps, max_angular_rps);

            /* Slow down for sharp turns. */
            float turn_scale = clampf(1.0f - fabsf(tgt_bear) / (0.5f * (float)M_PI), 0.0f, 1.0f);
            algo_v *= turn_scale;

            /* Lightweight IMU heading trim: only corrects drift, does not
             * override the UWB bearing command. Falls back to UWB-only
             * if IMU read fails. */
            float yaw_meas;
            if (imu_read_yaw(&yaw_meas)) {
                if (!yaw_ref_set) {
                    yaw_ref = yaw_meas;
                    yaw_ref_set = true;
                }
                yaw_ref = wrap_pi(yaw_ref + algo_w * dt);
                float err_yaw = wrap_pi(yaw_ref - yaw_meas);
                algo_w = clampf(algo_w + heading_kp * err_yaw, -max_angular_rps, max_angular_rps);
            } else {
                yaw_ref_set = false;
            }
        } else {
            algo_v = 0.0f;
            algo_w = 0.0f;
        }

#if FR_USE_MOTION_DEBUG_LIMITS
        algo_v = clampf(algo_v, 0.0f, FR_MOTION_DEBUG_MAX_LINEAR_MPS);
        algo_w = clampf(algo_w, -FR_MOTION_DEBUG_MAX_ANGULAR_RPS,
                        FR_MOTION_DEBUG_MAX_ANGULAR_RPS);
#endif

        bool motion_allowed = web_debug_motion_allowed();
#if !FR_ENABLE_MOTION
        motion_allowed = false;
#endif

        if (motion_allowed) {
            float lin_rate = (algo_v >= ramp_v_cmd) ? max_lin_accel : max_lin_decel;
            ramp_v_cmd = ramp(ramp_v_cmd, algo_v, lin_rate, dt);
            ramp_w_cmd = ramp(ramp_w_cmd, algo_w, max_ang_accel, dt);
        } else {
            ramp_v_cmd = ramp(ramp_v_cmd, 0.0f, max_lin_decel, dt);
            ramp_w_cmd = ramp(ramp_w_cmd, 0.0f, max_ang_accel, dt);
        }

        const float applied_v = motion_allowed ? ramp_v_cmd : 0.0f;
        const float applied_w = motion_allowed ? ramp_w_cmd : 0.0f;

        int direct_left_us = esc_mid_us;
        int direct_right_us = esc_mid_us;
        float direct_left_mps = 0.0f;
        float direct_right_mps = 0.0f;
        velocity_to_direct_pulses(applied_v, applied_w,
                                  track_width_m, max_wheel_mps,
                                  esc_min_us, esc_mid_us, esc_max_us,
                                  &direct_left_us, &direct_right_us,
                                  &direct_left_mps, &direct_right_mps);

        esp_err_t update_ret = ESP_OK;
        esp_err_t pulse_ret = ESP_OK;

#if FR_USE_DIRECT_PULSE_CONTROL
        update_ret = chassis_update(chassis, dt);
        pulse_ret = chassis_set_pulse_us(chassis, direct_left_us, direct_right_us);
#else
        pulse_ret = chassis_set_velocity(chassis, applied_v, applied_w);
        update_ret = chassis_update(chassis, dt);
#endif

        float mv = 0.0f;
        float mw = 0.0f;
        chassis_get_measured(chassis, &mv, &mw, NULL, NULL);

        /* ---- Build telemetry record and publish to web_debug ---- */
        wdbg_record_t rec;
        memset(&rec, 0, sizeof(rec));
        rec.t_us = t;
        rec.remote_client_connected = false;   /* filled by web_debug internally if needed */
        rec.remote_estop_latched = false;
        rec.remote_motion_armed = false;
        rec.remote_motion_allowed = motion_allowed;
        rec.remote_hb_age_ms = 0;
        rec.remote_hb_count = 0;

        /* Map follow_state_t -> wdbg_state_t */
        switch (state) {
        case FOLLOW_STATE_IDLE:   rec.state = WDBG_STATE_IDLE;   break;
        case FOLLOW_STATE_SEARCH: rec.state = WDBG_STATE_SEARCH; break;
        case FOLLOW_STATE_FOLLOW: rec.state = WDBG_STATE_FOLLOW; break;
        default:                  rec.state = WDBG_STATE_IDLE;   break;
        }

        rec.target_valid = tgt_valid;
        rec.target_age_ms = tgt_age_ms;
        rec.target_distance_m = tgt_dist;
        rec.target_bearing_rad = tgt_bear;
        rec.uwb = uwb;

        rec.algo_v_mps = algo_v;
        rec.algo_w_rps = algo_w;
        rec.ramp_v_mps = ramp_v_cmd;
        rec.ramp_w_rps = ramp_w_cmd;
        rec.applied_v_mps = applied_v;
        rec.applied_w_rps = applied_w;

        /* Chassis telemetry: targets from set_velocity, actuals from
         * chassis internal state (cmd_pulse is what was last written to ESC). */
        rec.target_left_mps = chassis->target_left_mps;
        rec.target_right_mps = chassis->target_right_mps;
        rec.cmd_left_us = chassis->cmd_pulse_l_us;
        rec.cmd_right_us = chassis->cmd_pulse_r_us;
        rec.meas_left_mps = chassis->meas_left_mps;
        rec.meas_right_mps = chassis->meas_right_mps;
        rec.meas_v_mps = mv;
        rec.meas_w_rps = mw;
        rec.chassis_update_ret = (int)update_ret;
        rec.chassis_pulse_ret = (int)pulse_ret;

        web_debug_update(&rec);

        if (++serial_log_div >= CONFIG_FOLLOW_ONLY_CONTROL_HZ / 5) {
            serial_log_div = 0;
            ESP_LOGI(TAG,
                     "%s tgt=%s age=%lums fwd=%.2f br=%+.1fdeg | "
                     "algo v=%+.2f w=%+.2f | applied v=%+.2f w=%+.2f | "
                     "tgt L/R=%+.2f/%+.2f cmd L/R=%.0f/%.0fus meas L/R=%+.2f/%+.2f",
                     state_name(state), tgt_valid ? "Y" : "N", (unsigned long)tgt_age_ms,
                     uwb.filt_fwd_m, RAD2DEG(uwb.filt_bearing_rad),
                     algo_v, algo_w, applied_v, applied_w,
                     chassis->target_left_mps, chassis->target_right_mps,
                     chassis->cmd_pulse_l_us, chassis->cmd_pulse_r_us,
                     chassis->meas_left_mps, chassis->meas_right_mps);
        }
    }
}

/* ---------------- Bring-up helpers ---------------- */
static esp_err_t chassis_bringup(void)
{
#if !FR_ENABLE_CHASSIS
    ESP_LOGW(TAG, "chassis disabled by compile-time switch");
    return ESP_OK;
#else
    chassis_config_t cc = chassis_default_config();
    cc.esc_min_us = CONFIG_FOLLOW_ONLY_ESC_MIN_US;
    cc.esc_mid_us = CONFIG_FOLLOW_ONLY_ESC_MID_US;
    cc.esc_max_us = CONFIG_FOLLOW_ONLY_ESC_MAX_US;
    cc.left_esc_gpio = CONFIG_FOLLOW_ONLY_LEFT_ESC_GPIO;
    cc.right_esc_gpio = CONFIG_FOLLOW_ONLY_RIGHT_ESC_GPIO;
    cc.left_invert = FR_LEFT_INVERT;
    cc.right_invert = FR_RIGHT_INVERT;
    cc.left_enc_a_gpio = CONFIG_FOLLOW_ONLY_LEFT_ENC_A_GPIO;
    cc.left_enc_b_gpio = CONFIG_FOLLOW_ONLY_LEFT_ENC_B_GPIO;
    cc.right_enc_a_gpio = CONFIG_FOLLOW_ONLY_RIGHT_ENC_A_GPIO;
    cc.right_enc_b_gpio = CONFIG_FOLLOW_ONLY_RIGHT_ENC_B_GPIO;
    cc.left_enc_invert = FR_LEFT_ENC_INVERT;
    cc.right_enc_invert = FR_RIGHT_ENC_INVERT;
    cc.ticks_per_meter = (float)CONFIG_FOLLOW_ONLY_TICKS_PER_METER;
    cc.track_width_m = CONFIG_FOLLOW_ONLY_TRACK_WIDTH_MM / 1000.0f;
    cc.max_speed_mps = CONFIG_FOLLOW_ONLY_MAX_WHEEL_SPEED_MMPS / 1000.0f;
    cc.kp = (float)CONFIG_FOLLOW_ONLY_SPEED_KP;
    cc.ki = (float)CONFIG_FOLLOW_ONLY_SPEED_KI;
    cc.kd = (float)CONFIG_FOLLOW_ONLY_SPEED_KD;
    cc.pid_out_limit_us = (float)CONFIG_FOLLOW_ONLY_SPEED_PID_LIMIT_US;

    esp_err_t ret = chassis_init(&s_chassis, &cc);
    if (ret == ESP_OK) {
        chassis_stop(&s_chassis);
        ESP_LOGI(TAG, "chassis ready; arming ESC neutral for %d ms", CONFIG_FOLLOW_ONLY_ESC_ARM_MS);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_FOLLOW_ONLY_ESC_ARM_MS));
        chassis_stop(&s_chassis);
    } else {
        ESP_LOGE(TAG, "chassis init FAILED: %s", esp_err_to_name(ret));
    }
    return ret;
#endif
}

static esp_err_t uwb_bringup(void)
{
#if !FR_ENABLE_UWB
    ESP_LOGW(TAG, "UWB disabled by compile-time switch");
    return ESP_OK;
#else
    bu_uwb_config_t bu = bu_uwb_default_config(
        (uart_port_t)CONFIG_FOLLOW_ONLY_UWB_UART,
        CONFIG_FOLLOW_ONLY_UWB_RX_GPIO,
        CONFIG_FOLLOW_ONLY_UWB_TX_GPIO);
    bu.baudrate = CONFIG_FOLLOW_ONLY_UWB_BAUD;

    esp_err_t ret = bu_uwb_init(&bu);
    if (ret == ESP_OK) {
        xTaskCreate(uwb_task, "uwb", 4096, NULL, 6, NULL);
        ESP_LOGI(TAG, "uwb ready (RX=GPIO%d)", CONFIG_FOLLOW_ONLY_UWB_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "uwb init FAILED: %s", esp_err_to_name(ret));
    }
    return ret;
#endif
}

static void imu_bringup(void)
{
#if !FR_ENABLE_IMU
    s_imu_ok = false;
    ESP_LOGW(TAG, "IMU disabled for UWB-only debug");
#else
    static i2c_master_bus_handle_t i2c_bus;
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = 0,
        .sda_io_num = CONFIG_FOLLOW_ONLY_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_FOLLOW_ONLY_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (i2c_new_master_bus(&i2c_cfg, &i2c_bus) == ESP_OK) {
        imu_i2c_config_t imucfg = imu_i2c_default_config();
        imucfg.sda_gpio = CONFIG_FOLLOW_ONLY_I2C_SDA_GPIO;
        imucfg.scl_gpio = CONFIG_FOLLOW_ONLY_I2C_SCL_GPIO;
        imucfg.device_address = CONFIG_FOLLOW_ONLY_IMU_ADDR;
        imucfg.external_bus = i2c_bus;

        if (imu_i2c_init(&s_imu, &imucfg) == ESP_OK) {
            s_imu_ok = true;
            ESP_LOGI(TAG, "imu ready");
        } else {
            s_imu_ok = false;
            ESP_LOGE(TAG, "imu init FAILED - heading loop disabled");
        }
    } else {
        s_imu_ok = false;
        ESP_LOGE(TAG, "I2C bus init FAILED");
    }
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "Follow-only UWB debug firmware starting");

    memset(&g_shared, 0, sizeof(g_shared));
    g_shared.lock = xSemaphoreCreateMutex();
    if (!g_shared.lock) {
        ESP_LOGE(TAG, "shared mutex create FAILED");
        return;
    }

    /* 1. Start WiFi AP + HTTP + SPIFFS log. */
    wdbg_config_t wcfg = web_debug_default_config();
    if (web_debug_init(&wcfg) != ESP_OK) {
        ESP_LOGE(TAG, "web_debug init FAILED; robot bring-up aborted");
        return;
    }

    /* 2. Wait for phone to connect and send heartbeat. */
    web_debug_wait_startup(0);  /* 0 = wait forever */

    /* 3. Start chassis, UWB, IMU. */
    if (chassis_bringup() != ESP_OK) {
        ESP_LOGE(TAG, "chassis failed; control task not started");
        return;
    }

    if (uwb_bringup() != ESP_OK) {
        ESP_LOGE(TAG, "UWB failed; control task will still run but target stays invalid");
    }

    imu_bringup();

    /* 4. Start control loop. */
    xTaskCreate(control_task, "control", 6144, &s_chassis, 7, NULL);
    ESP_LOGI(TAG,
             "control loop running at %d Hz; FR_ENABLE_MOTION=%d, direct_pulse=%d, "
             "Kconfig max v=%.2f m/s w=%.2f rad/s",
             CONFIG_FOLLOW_ONLY_CONTROL_HZ, FR_ENABLE_MOTION,
             FR_USE_DIRECT_PULSE_CONTROL,
             CONFIG_FOLLOW_ONLY_MAX_LINEAR_MMPS / 1000.0f,
             CONFIG_FOLLOW_ONLY_MAX_ANGULAR_MRADPS / 1000.0f);
}
