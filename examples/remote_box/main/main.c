/*
 * remote_box - 2D web-controlled differential-drive box.
 * No UWB, no IMU. Chassis + web_control only.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "chassis.h"
#include "web_control.h"

static const char *TAG = "remote_box";

#define M_PI 3.14159265358979323846

static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float ramp(float cur, float tgt, float rate, float dt)
{
    if (rate <= 0.0f || dt <= 0.0f) return tgt;
    float step = rate * dt;
    float d = tgt - cur;
    if (d > step) d = step;
    else if (d < -step) d = -step;
    return cur + d;
}

static float apply_deadband(float x, float db)
{
    if (fabsf(x) < db) return 0.0f;
    float s = (x > 0.0f) ? 1.0f : -1.0f;
    return s * (fabsf(x) - db) / (1.0f - db);
}

static float expo(float x) { return x * fabsf(x); }

/* ---- Chassis ---- */
static chassis_t s_chassis;

static esp_err_t chassis_bringup(void)
{
    chassis_config_t cc = chassis_default_config();
    cc.esc_min_us = CONFIG_REMOTE_BOX_ESC_MIN_US;
    cc.esc_mid_us = CONFIG_REMOTE_BOX_ESC_MID_US;
    cc.esc_max_us = CONFIG_REMOTE_BOX_ESC_MAX_US;
    cc.left_esc_gpio = CONFIG_REMOTE_BOX_LEFT_ESC_GPIO;
    cc.right_esc_gpio = CONFIG_REMOTE_BOX_RIGHT_ESC_GPIO;
    cc.left_invert = CONFIG_REMOTE_BOX_LEFT_INVERT;
    cc.right_invert = CONFIG_REMOTE_BOX_RIGHT_INVERT;

#if CONFIG_REMOTE_BOX_USE_ENCODERS
    cc.left_enc_a_gpio = CONFIG_REMOTE_BOX_LEFT_ENC_A_GPIO;
    cc.left_enc_b_gpio = CONFIG_REMOTE_BOX_LEFT_ENC_B_GPIO;
    cc.right_enc_a_gpio = CONFIG_REMOTE_BOX_RIGHT_ENC_A_GPIO;
    cc.right_enc_b_gpio = CONFIG_REMOTE_BOX_RIGHT_ENC_B_GPIO;
    cc.left_enc_invert = CONFIG_REMOTE_BOX_LEFT_ENC_INVERT;
    cc.right_enc_invert = CONFIG_REMOTE_BOX_RIGHT_ENC_INVERT;
    cc.ticks_per_meter = (float)CONFIG_REMOTE_BOX_TICKS_PER_METER;
    cc.kp = (float)CONFIG_REMOTE_BOX_SPEED_KP;
    cc.ki = (float)CONFIG_REMOTE_BOX_SPEED_KI;
    cc.kd = (float)CONFIG_REMOTE_BOX_SPEED_KD;
    cc.pid_out_limit_us = (float)CONFIG_REMOTE_BOX_SPEED_PID_LIMIT_US;
#else
    /* Open-loop: zero PID, disable encoders by setting sign=0. */
    cc.kp = 0; cc.ki = 0; cc.kd = 0;
    cc.left_enc_a_gpio = -1; cc.left_enc_b_gpio = -1;
    cc.right_enc_a_gpio = -1; cc.right_enc_b_gpio = -1;
#endif

    cc.track_width_m = CONFIG_REMOTE_BOX_TRACK_WIDTH_MM / 1000.0f;
    cc.max_speed_mps = CONFIG_REMOTE_BOX_MAX_WHEEL_SPEED_MMPS / 1000.0f;

    esp_err_t ret = chassis_init(&s_chassis, &cc);
    if (ret == ESP_OK) {
        chassis_stop(&s_chassis);
        ESP_LOGI(TAG, "chassis ready; arming %d ms", CONFIG_REMOTE_BOX_ESC_ARM_MS);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_REMOTE_BOX_ESC_ARM_MS));
        chassis_stop(&s_chassis);
    } else {
        ESP_LOGE(TAG, "chassis FAILED: %s", esp_err_to_name(ret));
    }
    return ret;
}

/* ---- Control task ---- */
static void control_task(void *arg)
{
    chassis_t *chassis = (chassis_t *)arg;

    const float max_fwd = CONFIG_REMOTE_BOX_MAX_LINEAR_MMPS / 1000.0f;
    const float max_rev = CONFIG_REMOTE_BOX_MAX_REVERSE_MMPS / 1000.0f;
    const float max_w = CONFIG_REMOTE_BOX_MAX_ANGULAR_MRADPS / 1000.0f;
    const float lin_accel = CONFIG_REMOTE_BOX_LIN_ACCEL / 1000.0f;
    const float lin_decel = CONFIG_REMOTE_BOX_LIN_DECEL / 1000.0f;
    const float ang_accel = CONFIG_REMOTE_BOX_ANG_ACCEL / 1000.0f;
    const uint32_t cmd_timeout_ms = CONFIG_REMOTE_BOX_CMD_TIMEOUT_MS;

    float ramp_v = 0.0f;
    float ramp_w = 0.0f;
    bool was_armed = false;

    /* P0-3: ARM-after-zero latch. */
    bool await_zero = false;

    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_REMOTE_BOX_CONTROL_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t prev_us = now_us();
    int serial_div = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        const uint64_t t = now_us();
        const float dt = (float)(t - prev_us) / 1e6f;
        prev_us = t;

        /* ---- E-stop fast path ---- */
        if (web_control_estop_pending()) {
            ramp_v = 0.0f;
            ramp_w = 0.0f;
            chassis_stop(chassis);
            was_armed = false;
            await_zero = false;
            continue;
        }

        /* ---- Read command ---- */
        wctl_cmd_t cmd;
        bool has_cmd = web_control_get_remote_cmd(&cmd);
        bool allowed = web_control_motion_allowed();

        /* ---- P0-3: ARM-after-zero protection ---- */
        if (allowed && !was_armed) {
            /* Just armed: require zero command before accepting non-zero. */
            await_zero = true;
            ramp_v = 0.0f;
            ramp_w = 0.0f;
        }
        was_armed = allowed;

        if (await_zero && has_cmd) {
            /* Check if received a zero/deadman-off command. */
            if (cmd.x_norm == 0.0f && cmd.y_norm == 0.0f) {
                await_zero = false;
            }
        }

        /* ---- Map to v/omega ---- */
        float target_v = 0.0f;
        float target_w = 0.0f;
        const char *reason = "OK";

        if (!allowed) {
            reason = "DISARMED";
        } else if (await_zero) {
            reason = "WAIT_ZERO";
        } else if (!has_cmd) {
            reason = "NO_CMD";
        } else if (!cmd.deadman) {
            reason = "DEADMAN_OFF";
        } else if ((t - cmd.rx_us) / 1000ULL > cmd_timeout_ms) {
            reason = "TIMEOUT";
        } else {
            float y = expo(apply_deadband(cmd.y_norm, 0.05f));
            float x = expo(apply_deadband(cmd.x_norm, 0.05f));
            float scale = clampf(cmd.scale, 0.2f, 1.0f);

            target_v = (y >= 0.0f) ? y * max_fwd * scale : y * max_rev * scale;
            target_w = x * max_w * scale;

            /* Turn slowdown. */
            float ts = 1.0f - 0.35f * fabsf(x);
            target_v *= clampf(ts, 0.55f, 1.0f);
        }

        /* ---- Ramp ---- */
        bool active = allowed && !await_zero && cmd.deadman && has_cmd
                      && ((t - cmd.rx_us) / 1000ULL <= cmd_timeout_ms);
        if (active) {
            float lr = (target_v >= ramp_v) ? lin_accel : lin_decel;
            ramp_v = ramp(ramp_v, target_v, lr, dt);
            ramp_w = ramp(ramp_w, target_w, ang_accel, dt);
        } else {
            ramp_v = ramp(ramp_v, 0.0f, lin_decel, dt);
            ramp_w = ramp(ramp_w, 0.0f, ang_accel, dt);
        }

        /* ---- Drive ---- */
        chassis_set_velocity(chassis, ramp_v, ramp_w);
        chassis_update(chassis, dt);

        /* ---- Telemetry ---- */
        float mv = 0.0f, mw = 0.0f;
        chassis_get_measured(chassis, &mv, &mw, NULL, NULL);

        wctl_record_t rec;
        memset(&rec, 0, sizeof(rec));
        rec.t_us = t;
        rec.target_v_mps = target_v;  /* post-arbitration desired target */
        rec.target_w_rps = target_w;
        rec.applied_v_mps = ramp_v;   /* actual value sent to chassis */
        rec.applied_w_rps = ramp_w;
        rec.target_left_mps = chassis->target_left_mps;
        rec.target_right_mps = chassis->target_right_mps;
        rec.cmd_left_us = chassis->cmd_pulse_l_us;
        rec.cmd_right_us = chassis->cmd_pulse_r_us;
        rec.meas_left_mps = chassis->meas_left_mps;
        rec.meas_right_mps = chassis->meas_right_mps;
        rec.meas_v_mps = mv;
        rec.meas_w_rps = mw;
        rec.safety_reason = reason;
        rec.using_encoders = CONFIG_REMOTE_BOX_USE_ENCODERS;

        web_control_update(&rec);

        /* ---- Serial log ---- */
        if (++serial_div >= CONFIG_REMOTE_BOX_CONTROL_HZ / 5) {
            serial_div = 0;
            ESP_LOGI(TAG,
                     "cmd x=%+.2f y=%+.2f dead=%d | v=%+.2f w=%+.2f | "
                     "tgt L=%+.2f R=%+.2f | meas L=%+.2f R=%+.2f | %s%s",
                     has_cmd ? cmd.x_norm : 0.0f,
                     has_cmd ? cmd.y_norm : 0.0f,
                     has_cmd ? cmd.deadman : 0,
                     ramp_v, ramp_w,
                     chassis->target_left_mps, chassis->target_right_mps,
                     chassis->meas_left_mps, chassis->meas_right_mps,
                     reason,
                     CONFIG_REMOTE_BOX_USE_ENCODERS ? "" : " [open-loop]");
        }
    }
}

/* ---- App main ---- */
void app_main(void)
{
    ESP_LOGI(TAG, "Remote Box starting");

    wctl_config_t cfg = web_control_default_config();
    cfg.ap_ssid = CONFIG_REMOTE_BOX_AP_SSID;
    cfg.ap_pass = CONFIG_REMOTE_BOX_AP_PASS;
    cfg.ap_channel = CONFIG_REMOTE_BOX_AP_CHANNEL;
    cfg.hb_timeout_us = (uint64_t)CONFIG_REMOTE_BOX_HB_TIMEOUT_MS * 1000ULL;
#if !CONFIG_REMOTE_BOX_LOG_ENABLE
    cfg.log_hz = 0;
#else
    cfg.log_hz = CONFIG_REMOTE_BOX_LOG_HZ;
#endif

    if (web_control_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "web_control FAILED");
        return;
    }

    web_control_wait_startup(0);

    if (chassis_bringup() != ESP_OK) {
        ESP_LOGE(TAG, "chassis FAILED");
        return;
    }

    xTaskCreate(control_task, "control", 6144, &s_chassis, 7, NULL);
    ESP_LOGI(TAG, "control at %d Hz, encoders=%s",
             CONFIG_REMOTE_BOX_CONTROL_HZ,
             CONFIG_REMOTE_BOX_USE_ENCODERS ? "ON" : "OFF");
}
