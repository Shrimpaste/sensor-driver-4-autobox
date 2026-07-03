/*
 * follow_only - pure follow-me suitcase, no obstacle avoidance.
 *
 * Sensors used:
 *   UWB (BU0x)      -> follow target (range + bearing to the user's tag)
 *   IMU             -> heading closed-loop (yaw error trims the turn command)
 *   chassis         -> rear diff-drive: APO-DL ESC (RC PWM) + AB encoder PID
 *
 * Difference vs 算法4:
 *   - No lidar, no ultrasonic sensors
 *   - No obstacle avoidance (VFH/AVOID/ESTOP states removed)
 *   - Simple 3-state machine: IDLE -> SEARCH -> FOLLOW
 *   - Clean, minimal code with only essential following logic
 */

#include <math.h>
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

#include "driver/i2c_master.h"
#include "imu_i2c.h"

static const char *TAG = "follow_only";

#define M_PI 3.14159265358979323846
#define DEG2RAD(d) ((float)(d) * (float)M_PI / 180.0f)

#define FR_LEFT_INVERT true
#define FR_RIGHT_INVERT true
#define FR_LEFT_ENC_INVERT true
#define FR_RIGHT_ENC_INVERT true
#define FR_UWB_LEFT_SIGN 1.0f
#define FR_IMU_YAW_SIGN -1.0f

/* ----------------------------------------------------- shared snapshot */

typedef struct {
    SemaphoreHandle_t lock;
    float tgt_distance_m;
    float tgt_bearing_rad;
    uint64_t tgt_ts_us;
} shared_t;

static shared_t g_shared;
static imu_i2c_t s_imu;
static bool s_imu_ok = false;
static chassis_t s_chassis;

/* ----------------------------------------------------- state machine */

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

/* ----------------------------------------------------- helpers */

static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }
static void lock(void) { xSemaphoreTake(g_shared.lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(g_shared.lock); }

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

/* ----------------------------------------------------- UWB task */

static void uwb_task(void *arg)
{
    (void)arg;
    char line[BU_UWB_LINE_MAX];
    const float left_sign = FR_UWB_LEFT_SIGN;

    while (1) {
        if (bu_uwb_read_line(line, sizeof(line), 200) != ESP_OK) continue;

        bu_uwb_twr_reading_t twr = {0};
        bu_uwb_distance_t dist = {0};

        if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
            float fwd_m = (float)twr.y_cm / 100.0f;
            float left_m = left_sign * (float)twr.x_cm / 100.0f;
            float range = (twr.distance_cm > 0)
                              ? (float)twr.distance_cm / 100.0f
                              : sqrtf(fwd_m * fwd_m + left_m * left_m);
            float bearing = 0.0f;
            if (fabsf(fwd_m) > 1e-3f || fabsf(left_m) > 1e-3f)
                bearing = atan2f(left_m, fwd_m);

            lock();
            g_shared.tgt_distance_m = range;
            g_shared.tgt_bearing_rad = bearing;
            g_shared.tgt_ts_us = now_us();
            unlock();
        } else if (bu_uwb_parse_distance_line(line, &dist) && dist.valid) {
            lock();
            g_shared.tgt_distance_m = dist.distance_m;
            g_shared.tgt_ts_us = now_us();
            unlock();
        }
    }
}

/* ----------------------------------------------------- IMU */

static bool imu_read_yaw(float *yaw_rad)
{
    if (!s_imu_ok) return false;
    imu_i2c_reading_t r;
    memset(&r, 0, sizeof(r));
    if (imu_i2c_read_all(&s_imu, &r) != ESP_OK || !r.valid) return false;
    *yaw_rad = FR_IMU_YAW_SIGN * DEG2RAD(r.euler_deg[2]);
    return true;
}

/* ----------------------------------------------------- control task */

static void control_task(void *arg)
{
    chassis_t *chassis = (chassis_t *)arg;

    const float follow_distance_m = CONFIG_FOLLOW_ONLY_FOLLOW_DISTANCE_MM / 1000.0f;
    const float stop_band_m = CONFIG_FOLLOW_ONLY_STOP_BAND_MM / 1000.0f;
    const float max_linear_mps = CONFIG_FOLLOW_ONLY_MAX_LINEAR_MMPS / 1000.0f;
    const float max_angular_rps = CONFIG_FOLLOW_ONLY_MAX_ANGULAR_MRADPS / 1000.0f;
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
    float cmd_v = 0.0f;
    float cmd_w = 0.0f;
    float lost_timer_s = 0.0f;
    float search_timer_s = 0.0f;
    float last_known_bearing = 0.0f;
    bool has_last_known = false;
    float yaw_ref = 0.0f;
    bool yaw_ref_set = false;

    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FOLLOW_ONLY_CONTROL_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t prev_us = now_us();
    int log_div = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        const uint64_t t = now_us();
        const float dt = (float)(t - prev_us) / 1e6f;
        prev_us = t;

        /* Snapshot UWB target data */
        bool tgt_valid;
        float tgt_dist;
        float tgt_bear;
        lock();
        tgt_valid = (t - g_shared.tgt_ts_us) < target_fresh_us;
        tgt_dist = g_shared.tgt_distance_m;
        tgt_bear = g_shared.tgt_bearing_rad;
        unlock();

        /* Track target loss timer */
        if (tgt_valid) {
            lost_timer_s = 0.0f;
            search_timer_s = 0.0f;
            last_known_bearing = tgt_bear;
            has_last_known = true;
        } else {
            lost_timer_s += dt;
            if (state == FOLLOW_STATE_SEARCH) search_timer_s += dt;
        }

        float v_des = 0.0f;
        float w_des = 0.0f;

        /* ---- State: Target lost -> SEARCH or IDLE ---- */
        if (!tgt_valid && lost_timer_s > 0.5f) {
            if (has_last_known && search_timer_s <= search_timeout_s) {
                state = FOLLOW_STATE_SEARCH;
                float dir = (last_known_bearing >= 0.0f) ? 1.0f : -1.0f;
                v_des = 0.0f;
                w_des = dir * search_rps;
                yaw_ref_set = false;
            } else {
                state = FOLLOW_STATE_IDLE;
                v_des = 0.0f;
                w_des = 0.0f;
                has_last_known = false;
                yaw_ref_set = false;
            }
        }
        /* ---- State: Target found -> FOLLOW ---- */
        else if (tgt_valid) {
            state = FOLLOW_STATE_FOLLOW;

            float err = tgt_dist - follow_distance_m;
            if (err > 0.0f) {
                v_des = kp_dist * err;
            } else if (-err <= stop_band_m) {
                v_des = 0.0f;
            } else {
                v_des = 0.0f;
            }
            v_des = clampf(v_des, 0.0f, max_linear_mps);

            w_des = kp_bear * tgt_bear;
            w_des = clampf(w_des, -max_angular_rps, max_angular_rps);

            float turn_scale = clampf(1.0f - fabsf(tgt_bear) / (0.5f * (float)M_PI),
                                      0.0f, 1.0f);
            v_des *= turn_scale;

            /* IMU heading closed-loop */
            float yaw_meas;
            if (imu_read_yaw(&yaw_meas)) {
                if (!yaw_ref_set) {
                    yaw_ref = yaw_meas;
                    yaw_ref_set = true;
                }
                yaw_ref = wrap_pi(yaw_ref + w_des * dt);
                float err_yaw = wrap_pi(yaw_ref - yaw_meas);
                w_des = w_des + heading_kp * err_yaw;
                w_des = clampf(w_des, -max_angular_rps, max_angular_rps);
            } else {
                yaw_ref_set = false;
            }
        }
        /* ---- Briefly lost: hold ---- */
        else {
            v_des = 0.0f;
            w_des = 0.0f;
        }

        /* Acceleration limiting */
        float lin_rate = (v_des >= cmd_v) ? max_lin_accel : max_lin_decel;
        cmd_v = ramp(cmd_v, v_des, lin_rate, dt);
        cmd_w = ramp(cmd_w, w_des, max_ang_accel, dt);

        chassis_set_velocity(chassis, cmd_v, cmd_w);
        chassis_update(chassis, dt);

        /* Logging at ~5 Hz */
        if (++log_div >= CONFIG_FOLLOW_ONLY_CONTROL_HZ / 5) {
            log_div = 0;
            float mv = 0.0f;
            float mw = 0.0f;
            chassis_get_measured(chassis, &mv, &mw, NULL, NULL);
            ESP_LOGI(TAG,
                     "%-6s tgt=%s d=%.2f br=%+.2f | cmd v=%+.2f w=%+.2f | "
                     "meas v=%+.2f w=%+.2f",
                     state_name(state), tgt_valid ? "Y" : "N",
                     tgt_dist, tgt_bear, cmd_v, cmd_w, mv, mw);
        }
    }
}

/* ----------------------------------------------------- bring-up */

void app_main(void)
{
    ESP_LOGI(TAG, "Follow-only suitcase starting");

    /* 1. Shared state */
    memset(&g_shared, 0, sizeof(g_shared));
    g_shared.lock = xSemaphoreCreateMutex();

    /* 2. Chassis init */
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

    if (chassis_init(&s_chassis, &cc) == ESP_OK) {
        chassis_stop(&s_chassis);
        ESP_LOGI(TAG, "chassis ready; arming ESC (hold neutral %d ms)",
                 CONFIG_FOLLOW_ONLY_ESC_ARM_MS);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_FOLLOW_ONLY_ESC_ARM_MS));
    } else {
        ESP_LOGE(TAG, "chassis init FAILED - check ESC/encoder GPIOs");
    }

    /* 3. UWB init */
    bu_uwb_config_t bu = bu_uwb_default_config(
        (uart_port_t)CONFIG_FOLLOW_ONLY_UWB_UART,
        CONFIG_FOLLOW_ONLY_UWB_RX_GPIO,
        CONFIG_FOLLOW_ONLY_UWB_TX_GPIO);
    bu.baudrate = CONFIG_FOLLOW_ONLY_UWB_BAUD;

    if (bu_uwb_init(&bu) == ESP_OK) {
        xTaskCreate(uwb_task, "uwb", 4096, NULL, 6, NULL);
        ESP_LOGI(TAG, "uwb ready (RX=GPIO%d)", CONFIG_FOLLOW_ONLY_UWB_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "uwb init FAILED");
    }

    /* 4. IMU init */
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
            ESP_LOGI(TAG, "imu ready (heading loop ON)");
        } else {
            ESP_LOGE(TAG, "imu init FAILED - heading loop disabled");
        }
    } else {
        ESP_LOGE(TAG, "I2C bus init FAILED");
    }

    /* 5. Start control loop */
    xTaskCreate(control_task, "control", 4096, &s_chassis, 7, NULL);
    ESP_LOGI(TAG, "control loop running at %d Hz", CONFIG_FOLLOW_ONLY_CONTROL_HZ);
}
