/*
 * follow_only_uwb_debug_estop_flashlog
 *
 * 基于 follow_only 版本改造的 UWB-only 调试固件：
 *   1. ESP32 先启动 SoftAP + HTTP 页面；必须手机连接并产生 heartbeat 后才继续初始化机器人。
 *   2. 允许低速电机运动；仍需手机网页 CLEAR / ARM，STOP 后停机；CLEAR / ARM 后保持允许运动，直到再次按 STOP。
 *   3. 手机网页可实时查看 UWB 原始值、滤波值、控制算法输出、电机下发值和编码器测量值。
 *   4. 控制任务只做轻量入队，Flash 日志由低优先级任务异步写入 SPIFFS。
 *   5. UWB 使用 TWR 坐标做 EMA 平滑，并带简单跳点剔除。
 *
 * 重要安全约束：
 *   - HTTP/WiFi 任务绝不直接操作底盘。
 *   - 当前调试版绕过速度闭环：control_task 直接调用 chassis_set_pulse_us() 写 ESC 脉宽。
 *   - FR_ENABLE_MOTION 已开启；网页未 ARM 或已 STOP 时，实际下发速度强制为 0。
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "bu_uwb.h"
#include "chassis.h"

#include "driver/i2c_master.h"
#include "imu_i2c.h"

static const char *TAG = "follow_only_dbg";

#define M_PI 3.14159265358979323846
#define DEG2RAD(d) ((float)(d) * (float)M_PI / 180.0f)
#define RAD2DEG(r) ((float)(r) * 180.0f / (float)M_PI)

/* ---------------- Compile-time debug switches ----------------
 * 当前阶段允许电机运动。运动仍受网页 CLEAR/ARM、STOP 和心跳超时保护。
 * 速度不再使用额外的调试限幅，只使用 menuconfig/Kconfig 中的正常速度上限。
 */
#define FR_ENABLE_UWB            1
#define FR_ENABLE_IMU            0
#define FR_ENABLE_CHASSIS        1
#define FR_ENABLE_MOTION         1
#define FR_ENABLE_FLASH_LOG      1
#define FR_ENABLE_LIVE_STATUS    1

/* Set to 1 only when you want an extra hard cap during early bench testing.
 * Normal field testing should keep this at 0 so speed follows Kconfig limits:
 *   CONFIG_FOLLOW_ONLY_MAX_LINEAR_MMPS
 *   CONFIG_FOLLOW_ONLY_MAX_ANGULAR_MRADPS
 */
#define FR_USE_MOTION_DEBUG_LIMITS       0
#define FR_MOTION_DEBUG_MAX_LINEAR_MPS   0.15f
#define FR_MOTION_DEBUG_MAX_ANGULAR_RPS  0.35f

/* Direct ESC pulse mode for motor-chain debugging.
 * 1 = bypass chassis_set_velocity()/wheel-speed PID and call chassis_set_pulse_us() directly.
 * This is useful when meas_v_mps stays 0 and we need to verify ESC/PWM/motor response.
 * Safety gates still apply: no ARM / STOP => neutral pulses.
 */
#define FR_USE_DIRECT_PULSE_CONTROL      1

#define FR_LEFT_INVERT true
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

/* For motion safety, range-only frames are logged but do not refresh the
 * control target. A range-only frame has no fresh bearing; using it for motion
 * could drive with a stale direction.
 */
#define FR_UWB_RANGE_ONLY_REFRESH_TARGET 0

/* Remote E-stop / heartbeat. */
#define FR_REMOTE_AP_SSID            "FollowRobot-UWB"
#define FR_REMOTE_AP_PASS            "12345678"
#define FR_REMOTE_AP_CHANNEL         6
#define FR_REMOTE_AP_MAX_CONN        2
#define FR_REMOTE_HB_TIMEOUT_US      3600000000ULL

/* Flash log. 需要分区表中存在 label=logs 的 SPIFFS 分区。 */
#define FR_LOG_PARTITION_LABEL       "logs"
#define FR_LOG_BASE_PATH             "/spiffs"
#define FR_LOG_PATH                  "/spiffs/follow_log.csv"
#define FR_LOG_QUEUE_LEN             96
#define FR_LOG_HZ                    5
#define FR_LIVE_JSON_BUF_SIZE        8192

/* Startup gate. */
#define FR_STARTUP_LOG_INTERVAL_MS   1000

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

    /* Keep the curvature while respecting the configured max wheel speed. */
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

/* ---------------- UWB detailed snapshot ---------------- */
typedef enum {
    UWB_FRAME_NONE = 0,
    UWB_FRAME_TWR,
    UWB_FRAME_RANGE_ONLY,
    UWB_FRAME_PARSE_ERROR,
} uwb_frame_type_t;

static const char *uwb_frame_type_name(uwb_frame_type_t t)
{
    switch (t) {
    case UWB_FRAME_TWR:        return "twr";
    case UWB_FRAME_RANGE_ONLY: return "range_only";
    case UWB_FRAME_PARSE_ERROR:return "parse_error";
    case UWB_FRAME_NONE:
    default:                  return "none";
    }
}

typedef struct {
    bool valid;                    /* filtered target currently usable */
    bool filter_initialized;
    bool last_frame_accepted;
    bool bearing_stale;
    uwb_frame_type_t frame_type;

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

    uint64_t ts_us;                /* last raw frame time */
    uint64_t accepted_ts_us;       /* last accepted filtered pose time */
    uint32_t frame_count;
    uint32_t twr_count;
    uint32_t range_only_count;
    uint32_t parse_error_count;
    uint32_t outlier_count;
    uint32_t consecutive_outliers;
} uwb_debug_t;

typedef struct {
    SemaphoreHandle_t lock;

    float tgt_distance_m;
    float tgt_bearing_rad;
    uint64_t tgt_ts_us;

    uwb_debug_t uwb;
} shared_t;

static shared_t g_shared;
static imu_i2c_t s_imu;
static bool s_imu_ok = false;
static chassis_t s_chassis;

static void lock(void) { xSemaphoreTake(g_shared.lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(g_shared.lock); }

/* ---------------- Remote state ---------------- */
typedef struct {
    bool client_connected;
    bool estop_latched;
    bool motion_armed;
    uint64_t last_heartbeat_us;
    uint64_t first_heartbeat_us;
    uint64_t last_cmd_us;
    uint32_t heartbeat_count;
    uint32_t estop_count;
    uint32_t clear_count;
    uint32_t motion_off_count;
    uint32_t timeout_count;
    uint32_t connect_count;
    uint32_t disconnect_count;
} remote_state_t;

static remote_state_t s_remote = {
    .client_connected = false,
    .estop_latched = true,
    .motion_armed = false,
};
static portMUX_TYPE s_remote_mux = portMUX_INITIALIZER_UNLOCKED;
static httpd_handle_t s_httpd = NULL;

/* Immediate E-stop flag: set by HTTP handler, polled by control_task every
 * cycle BEFORE any other logic.  This bypasses the portMUX snapshot path so
 * that a STOP command never waits behind an in-flight /live response. */
static volatile bool s_estop_pending = false;

static void remote_get_snapshot(remote_state_t *out)
{
    portENTER_CRITICAL(&s_remote_mux);
    *out = s_remote;
    portEXIT_CRITICAL(&s_remote_mux);
}

static void remote_set_client_connected(bool connected, uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.client_connected = connected;
    s_remote.last_cmd_us = t;
    if (connected) {
        s_remote.connect_count++;
    } else {
        s_remote.disconnect_count++;
        /* 按需求：断连不再自动清除 ARM；只有 STOP 才会把 motion_armed 置 false。 */
    }
    portEXIT_CRITICAL(&s_remote_mux);
}

static void remote_heartbeat(uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.last_heartbeat_us = t;
    if (s_remote.first_heartbeat_us == 0) {
        s_remote.first_heartbeat_us = t;
    }
    s_remote.heartbeat_count++;
    portEXIT_CRITICAL(&s_remote_mux);
}

static void remote_estop(uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.estop_latched = true;
    s_remote.motion_armed = false;
    s_remote.last_cmd_us = t;
    s_remote.estop_count++;
    portEXIT_CRITICAL(&s_remote_mux);
}

static void remote_clear_and_arm(uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.estop_latched = false;
    s_remote.motion_armed = true;
    s_remote.last_heartbeat_us = t;
    if (s_remote.first_heartbeat_us == 0) {
        s_remote.first_heartbeat_us = t;
    }
    s_remote.last_cmd_us = t;
    s_remote.clear_count++;
    portEXIT_CRITICAL(&s_remote_mux);
}

static void remote_motion_off(uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.motion_armed = false;
    s_remote.last_cmd_us = t;
    s_remote.motion_off_count++;
    portEXIT_CRITICAL(&s_remote_mux);
}

static bool remote_startup_ready(void)
{
    remote_state_t r;
    remote_get_snapshot(&r);
    return r.client_connected && r.first_heartbeat_us != 0;
}

static bool remote_motion_allowed(uint64_t t)
{
    (void)t;

    remote_state_t r;
    remote_get_snapshot(&r);

    /*
     * 锁存逻辑：
     *   默认：estop_latched=true, motion_armed=false => false
     *   CLEAR / ARM：estop_latched=false, motion_armed=true => true
     *   STOP：estop_latched=true, motion_armed=false => false
     *
     * 注意：按需求，这里不再因为 heartbeat 超时或 WiFi 断连自动变 false。
     */
    return (!r.estop_latched && r.motion_armed);
}

/* ---------------- Live telemetry / Flash log ---------------- */
typedef struct {
    uint64_t t_us;
    bool remote_client_connected;
    bool remote_estop_latched;
    bool remote_motion_armed;
    bool remote_motion_allowed;
    uint32_t remote_hb_age_ms;
    uint32_t remote_hb_count;

    follow_state_t state;
    bool target_valid;
    uint32_t target_age_ms;
    float target_distance_m;
    float target_bearing_rad;

    uwb_debug_t uwb;

    float algo_v_mps;
    float algo_w_rps;
    float ramp_v_mps;
    float ramp_w_rps;
    float applied_v_mps;
    float applied_w_rps;
    float direct_left_mps;
    float direct_right_mps;
    int direct_left_us;
    int direct_right_us;
    int chassis_update_ret;
    int chassis_pulse_ret;
    float meas_v_mps;
    float meas_w_rps;

    uint32_t log_dropped;
} telemetry_record_t;

static telemetry_record_t s_live;
static portMUX_TYPE s_live_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_log_q = NULL;
static uint32_t s_log_dropped = 0;
static volatile bool s_clear_log_requested = false;

static void live_set(const telemetry_record_t *rec)
{
    portENTER_CRITICAL(&s_live_mux);
    s_live = *rec;
    portEXIT_CRITICAL(&s_live_mux);
}

static void live_get(telemetry_record_t *out)
{
    portENTER_CRITICAL(&s_live_mux);
    *out = s_live;
    portEXIT_CRITICAL(&s_live_mux);
}

static void flash_log_enqueue(const telemetry_record_t *rec)
{
#if FR_ENABLE_FLASH_LOG
    if (!s_log_q) {
        return;
    }
    if (xQueueSend(s_log_q, rec, 0) != pdTRUE) {
        s_log_dropped++;
    }
#else
    (void)rec;
#endif
}

static void flash_log_write_header(FILE *f)
{
    fprintf(f,
            "t_ms,remote_client,estop,armed,remote_ok,hb_age_ms,"
            "state,target_valid,target_age_ms,"
            "uwb_frame,uwb_valid,uwb_raw_x_cm,uwb_raw_y_cm,uwb_raw_dist_cm,"
            "raw_fwd_m,raw_left_m,raw_range_m,raw_bearing_deg,"
            "filt_fwd_m,filt_left_m,filt_range_m,filt_bearing_deg,"
            "uwb_speed_mps,uwb_bearing_rate_rps,uwb_frame_count,uwb_twr_count,"
            "uwb_range_only_count,uwb_parse_error_count,uwb_outlier_count,"
            "algo_v_mps,algo_w_rps,ramp_v_mps,ramp_w_rps,"
            "applied_v_mps,applied_w_rps,direct_left_mps,direct_right_mps,"
            "direct_left_us,direct_right_us,chassis_update_ret,chassis_pulse_ret,"
            "meas_v_mps,meas_w_rps,log_dropped\n");
}

static FILE *flash_log_open_append(void)
{
    struct stat st;
    bool need_header = (stat(FR_LOG_PATH, &st) != 0) || (st.st_size == 0);

    FILE *f = fopen(FR_LOG_PATH, "a");
    if (!f) {
        ESP_LOGE(TAG, "open log failed: %s", strerror(errno));
        return NULL;
    }

    if (need_header) {
        flash_log_write_header(f);
        fflush(f);
    }
    return f;
}

static void flash_log_task(void *arg)
{
    (void)arg;
    FILE *f = flash_log_open_append();
    telemetry_record_t rec;
    int flush_div = 0;

    while (1) {
        if (s_clear_log_requested) {
            s_clear_log_requested = false;
            if (f) {
                fclose(f);
                f = NULL;
            }
            remove(FR_LOG_PATH);
            f = flash_log_open_append();
        }

        if (xQueueReceive(s_log_q, &rec, pdMS_TO_TICKS(500)) != pdTRUE) {
            if (f) {
                fflush(f);
            }
            continue;
        }

        if (!f) {
            f = flash_log_open_append();
            if (!f) {
                continue;
            }
        }

        fprintf(f,
                "%llu,%d,%d,%d,%d,%lu,"
                "%s,%d,%lu,"
                "%s,%d,%d,%d,%d,"
                "%.3f,%.3f,%.3f,%.2f,"
                "%.3f,%.3f,%.3f,%.2f,"
                "%.3f,%.3f,%lu,%lu,%lu,%lu,%lu,"
                "%.3f,%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%.3f,%.3f,%lu\n",
                (unsigned long long)(rec.t_us / 1000ULL),
                rec.remote_client_connected,
                rec.remote_estop_latched,
                rec.remote_motion_armed,
                rec.remote_motion_allowed,
                (unsigned long)rec.remote_hb_age_ms,
                state_name(rec.state),
                rec.target_valid,
                (unsigned long)rec.target_age_ms,
                uwb_frame_type_name(rec.uwb.frame_type),
                rec.uwb.valid,
                rec.uwb.raw_x_cm,
                rec.uwb.raw_y_cm,
                rec.uwb.raw_distance_cm,
                rec.uwb.raw_fwd_m,
                rec.uwb.raw_left_m,
                rec.uwb.raw_range_m,
                RAD2DEG(rec.uwb.raw_bearing_rad),
                rec.uwb.filt_fwd_m,
                rec.uwb.filt_left_m,
                rec.uwb.filt_range_m,
                RAD2DEG(rec.uwb.filt_bearing_rad),
                rec.uwb.speed_mps,
                rec.uwb.bearing_rate_rps,
                (unsigned long)rec.uwb.frame_count,
                (unsigned long)rec.uwb.twr_count,
                (unsigned long)rec.uwb.range_only_count,
                (unsigned long)rec.uwb.parse_error_count,
                (unsigned long)rec.uwb.outlier_count,
                rec.algo_v_mps,
                rec.algo_w_rps,
                rec.ramp_v_mps,
                rec.ramp_w_rps,
                rec.applied_v_mps,
                rec.applied_w_rps,
                rec.direct_left_mps,
                rec.direct_right_mps,
                rec.direct_left_us,
                rec.direct_right_us,
                rec.chassis_update_ret,
                rec.chassis_pulse_ret,
                rec.meas_v_mps,
                rec.meas_w_rps,
                (unsigned long)rec.log_dropped);

        if (++flush_div >= FR_LOG_HZ) {
            flush_div = 0;
            fflush(f);
        }
    }
}

static esp_err_t flash_log_start(void)
{
#if !FR_ENABLE_FLASH_LOG
    return ESP_OK;
#else
    esp_vfs_spiffs_conf_t conf = {
        .base_path = FR_LOG_BASE_PATH,
        .partition_label = FR_LOG_PARTITION_LABEL,
        .max_files = 4,
        .format_if_mount_failed = true,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    size_t total = 0;
    size_t used = 0;
    ret = esp_spiffs_info(FR_LOG_PARTITION_LABEL, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "log SPIFFS mounted: total=%u used=%u", (unsigned)total, (unsigned)used);
    }

    s_log_q = xQueueCreate(FR_LOG_QUEUE_LEN, sizeof(telemetry_record_t));
    if (!s_log_q) {
        ESP_LOGE(TAG, "log queue create FAILED");
        return ESP_ERR_NO_MEM;
    }

    xTaskCreate(flash_log_task, "flash_log", 6144, NULL, 2, NULL);
    return ESP_OK;
#endif
}

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
    uwb_debug_t *u = &g_shared.uwb;

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
    u->frame_type = UWB_FRAME_TWR;
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
    uwb_debug_t *u = &g_shared.uwb;
    u->frame_count++;
    u->range_only_count++;
    u->frame_type = UWB_FRAME_RANGE_ONLY;
    u->ts_us = t;
    u->raw_distance_cm = (int)(distance_m * 100.0f);
    u->raw_range_m = distance_m;
    u->bearing_stale = true;
    u->last_frame_accepted = true;

#if FR_UWB_RANGE_ONLY_REFRESH_TARGET
    /* Optional legacy behavior: range-only refreshes distance but keeps bearing.
     * Not recommended when motion is enabled because bearing may be stale.
     */
    g_shared.tgt_distance_m = distance_m;
    g_shared.tgt_ts_us = t;
#else
    /* Motion-safe behavior: log range-only frames, but do not refresh the
     * control target timestamp. The control loop continues to use the last
     * accepted TWR-filtered pose until it becomes stale and then stops/searches.
     */
    (void)distance_m;
#endif
    unlock();
}

static void uwb_publish_parse_error(void)
{
    lock();
    g_shared.uwb.frame_count++;
    g_shared.uwb.parse_error_count++;
    g_shared.uwb.frame_type = UWB_FRAME_PARSE_ERROR;
    g_shared.uwb.ts_us = now_us();
    g_shared.uwb.last_frame_accepted = false;
    unlock();
}

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

/* ---------------- HTTP server ---------------- */
static esp_err_t http_send_text(httpd_req_t *req, const char *text, const char *type)
{
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
}

static const char s_index_html[] =
"<!doctype html><html><head>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Follow UWB Debug</title>"
"<style>"
"body{font-family:Arial,sans-serif;background:#101214;color:#eee;margin:18px;}"
"button{font-size:20px;padding:14px 18px;margin:6px;border:0;border-radius:10px;}"
"#stop{background:#d00000;color:white;width:100%;height:92px;font-size:38px;font-weight:bold;}"
"#arm{background:#008f5a;color:white;}#off{background:#555;color:white;}#clearlog{background:#7655d9;color:white;}"
"pre{white-space:pre-wrap;background:#1e2329;padding:12px;border-radius:10px;font-size:14px;}"
"#summary{background:#18202a;border:1px solid #38506a;border-radius:10px;padding:12px;margin:12px 0;font-size:18px;line-height:1.45;}"
"#summary b{font-size:26px;color:#8cc8ff;}"
"a{color:#8cc8ff;} .warn{color:#ffd166;}"
"</style></head><body>"
"<h2>Follow-only UWB Debug</h2>"
"<p class='warn'>CLEAR / ARM latches motion_allowed=true. STOP latches it false.</p>"
"<button id='stop' onclick='estop()'>STOP</button><br>"
"<button id='arm' onclick='arm()'>CLEAR / ARM</button>"
"<button id='off' onclick='motionOff()'>MOTION OFF</button>"
"<button id='clearlog' onclick='clearLog()'>CLEAR LOG</button>"
"<p><a href='/log' target='_blank'>Download CSV log</a></p>"
"<div id='summary'>waiting for live data...</div>"
"<div id='errmsg' style='color:red;font-weight:bold'></div>"
"<pre id='live'>loading...</pre>"
"<script>"
"function f(x,d){return (x===undefined||x===null||!isFinite(Number(x)))?'--':Number(x).toFixed(d);}"
"function render(j){let r=j.remote||{},t=j.target||{},u=j.uwb||{},c=j.control||{};"
"document.getElementById('errmsg').textContent='';"
"document.getElementById('summary').innerHTML="
"'<b>Distance '+f(t.distance_m,2)+' m</b> &nbsp; Bearing '+f(t.bearing_deg,1)+' deg<br>' +"
"'UWB filtered: d='+f(u.filtered_distance_m,2)+' m, bearing='+f(u.bearing_deg,1)+' deg; raw: d='+f(u.raw_distance_m,2)+' m, bearing='+f(u.raw_bearing_deg,1)+' deg<br>' +"
"'target_valid='+t.valid+', age='+t.age_ms+' ms, frame='+u.frame_type+', stale_bearing='+u.bearing_stale+'<br>' +"
"'applied v='+f(c.applied_v_mps,2)+' m/s, w='+f(c.applied_w_rps,2)+' rad/s, armed='+r.motion_armed+', motion_allowed='+r.motion_allowed+'<br>' +"
"'direct pulse: L='+f(c.direct_left_mps,2)+' m/s '+c.direct_left_us+' us, R='+f(c.direct_right_mps,2)+' m/s '+c.direct_right_us+' us, ret pulse='+c.chassis_pulse_ret;"
"}"
"async function post(p){try{let r=await fetch(p,{method:'POST',cache:'no-store'});if(!r.ok)throw new Error(r.status);return true;}catch(e){document.getElementById('errmsg').textContent='CMD FAIL: '+p+' ('+e+')';return false;}}"
"async function estop(){if(await post('/estop'))await poll();}"
"async function arm(){if(await post('/clear'))await poll();}"
"async function motionOff(){if(await post('/motion_off'))await poll();}"
"async function clearLog(){if(await post('/clear_log'))await poll();}"
"async function hb(){await post('/hb');}"
"var _pollCtrl=null;"
"async function poll(){"
"if(_pollCtrl)_pollCtrl.abort();_pollCtrl=new AbortController();"
"try{let r=await fetch('/live',{cache:'no-store',signal:_pollCtrl.signal});if(!r.ok)throw new Error('HTTP '+r.status);"
"let txt=await r.text();let j=JSON.parse(txt);render(j);document.getElementById('live').textContent=JSON.stringify(j,null,2);"
"}catch(e){if(e.name!=='AbortError'){document.getElementById('summary').textContent='Live connection problem';document.getElementById('live').textContent='connection lost: '+e;}}}"
"setInterval(hb,300);setInterval(poll,300);hb();poll();"
"</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    return http_send_text(req, s_index_html, "text/html");
}

static esp_err_t heartbeat_handler(httpd_req_t *req)
{
    remote_heartbeat(now_us());
    return http_send_text(req, "OK\n", "text/plain");
}

static esp_err_t estop_handler(httpd_req_t *req)
{
    s_estop_pending = true;           /* immediate flag — control_task sees it next cycle */
    remote_estop(now_us());           /* also latch in the full state struct */
    ESP_LOGW(TAG, "REMOTE E-STOP latched");
    return http_send_text(req, "ESTOP\n", "text/plain");
}

static esp_err_t clear_handler(httpd_req_t *req)
{
    remote_clear_and_arm(now_us());
    ESP_LOGW(TAG, "REMOTE E-STOP cleared; motion armed by page");
    return http_send_text(req, "ARMED\n", "text/plain");
}

static esp_err_t motion_off_handler(httpd_req_t *req)
{
    remote_motion_off(now_us());
    ESP_LOGW(TAG, "motion armed flag cleared; data collection continues");
    return http_send_text(req, "MOTION_OFF\n", "text/plain");
}

static esp_err_t clear_log_handler(httpd_req_t *req)
{
    s_clear_log_requested = true;
    return http_send_text(req, "CLEAR_LOG_REQUESTED\n", "text/plain");
}

static char s_live_json_buf[FR_LIVE_JSON_BUF_SIZE];

static esp_err_t live_handler(httpd_req_t *req)
{
    const uint64_t t = now_us();
    const bool remote_ok = remote_motion_allowed(t);

    /* Snapshot both data sources quickly, then release locks before formatting. */
    telemetry_record_t rec;
    remote_state_t r;
    live_get(&rec);
    remote_get_snapshot(&r);

    uint32_t hb_age_ms = 0xffffffffu;
    if (r.last_heartbeat_us != 0 && t >= r.last_heartbeat_us) {
        hb_age_ms = (uint32_t)((t - r.last_heartbeat_us) / 1000ULL);
    }

    char *buf = s_live_json_buf;

    int n = snprintf(buf, FR_LIVE_JSON_BUF_SIZE,
             "{"
             "\"remote\":{"
             "\"client_connected\":%s,\"estop_latched\":%s,\"motion_armed\":%s,"
             "\"motion_allowed\":%s,\"compile_motion_enabled\":%s,"
             "\"heartbeat_age_ms\":%lu,\"heartbeat_timeout_ms\":%lu,"
             "\"heartbeat_count\":%lu,\"connect_count\":%lu,\"disconnect_count\":%lu,"
             "\"timeout_count\":%lu},"
             "\"target\":{\"valid\":%s,\"age_ms\":%lu,"
             "\"distance_m\":%.3f,\"bearing_rad\":%.4f,\"bearing_deg\":%.2f},"
             "\"uwb\":{"
             "\"frame_type\":\"%s\",\"valid\":%s,\"last_frame_accepted\":%s,\"bearing_stale\":%s,"
             "\"distance_m\":%.3f,\"bearing_deg\":%.2f,\"raw_distance_m\":%.3f,\"filtered_distance_m\":%.3f,"
             "\"raw_x_cm\":%d,\"raw_y_cm\":%d,\"raw_distance_cm\":%d,"
             "\"raw_fwd_m\":%.3f,\"raw_left_m\":%.3f,\"raw_range_m\":%.3f,"
             "\"raw_bearing_rad\":%.4f,\"raw_bearing_deg\":%.2f,"
             "\"filt_fwd_m\":%.3f,\"filt_left_m\":%.3f,\"filt_range_m\":%.3f,"
             "\"filt_bearing_rad\":%.4f,\"filt_bearing_deg\":%.2f,"
             "\"speed_mps\":%.3f,\"bearing_rate_rps\":%.3f,"
             "\"frame_count\":%lu,\"twr_count\":%lu,\"range_only_count\":%lu,"
             "\"parse_error_count\":%lu,\"outlier_count\":%lu},"
             "\"control\":{\"state\":\"%s\","
             "\"algo_v_mps\":%.3f,\"algo_w_rps\":%.3f,"
             "\"ramp_v_mps\":%.3f,\"ramp_w_rps\":%.3f,"
             "\"applied_v_mps\":%.3f,\"applied_w_rps\":%.3f,"
             "\"direct_pulse_mode\":%s,"
             "\"direct_left_mps\":%.3f,\"direct_right_mps\":%.3f,"
             "\"direct_left_us\":%d,\"direct_right_us\":%d,"
             "\"chassis_update_ret\":%d,\"chassis_pulse_ret\":%d,"
             "\"meas_v_mps\":%.3f,\"meas_w_rps\":%.3f},"
             "\"log\":{\"dropped\":%lu,\"path\":\"%s\"}"
             "}\n",
             r.client_connected ? "true" : "false",
             r.estop_latched ? "true" : "false",
             r.motion_armed ? "true" : "false",
             remote_ok ? "true" : "false",
#if FR_ENABLE_MOTION
             "true",
#else
             "false",
#endif
             (unsigned long)hb_age_ms,
             (unsigned long)(FR_REMOTE_HB_TIMEOUT_US / 1000ULL),
             (unsigned long)r.heartbeat_count,
             (unsigned long)r.connect_count,
             (unsigned long)r.disconnect_count,
             (unsigned long)r.timeout_count,
             rec.target_valid ? "true" : "false",
             (unsigned long)rec.target_age_ms,
             rec.target_distance_m,
             rec.target_bearing_rad,
             RAD2DEG(rec.target_bearing_rad),
             uwb_frame_type_name(rec.uwb.frame_type),
             rec.uwb.valid ? "true" : "false",
             rec.uwb.last_frame_accepted ? "true" : "false",
             rec.uwb.bearing_stale ? "true" : "false",
             rec.uwb.filt_range_m,
             RAD2DEG(rec.uwb.filt_bearing_rad),
             rec.uwb.raw_range_m,
             rec.uwb.filt_range_m,
             rec.uwb.raw_x_cm,
             rec.uwb.raw_y_cm,
             rec.uwb.raw_distance_cm,
             rec.uwb.raw_fwd_m,
             rec.uwb.raw_left_m,
             rec.uwb.raw_range_m,
             rec.uwb.raw_bearing_rad,
             RAD2DEG(rec.uwb.raw_bearing_rad),
             rec.uwb.filt_fwd_m,
             rec.uwb.filt_left_m,
             rec.uwb.filt_range_m,
             rec.uwb.filt_bearing_rad,
             RAD2DEG(rec.uwb.filt_bearing_rad),
             rec.uwb.speed_mps,
             rec.uwb.bearing_rate_rps,
             (unsigned long)rec.uwb.frame_count,
             (unsigned long)rec.uwb.twr_count,
             (unsigned long)rec.uwb.range_only_count,
             (unsigned long)rec.uwb.parse_error_count,
             (unsigned long)rec.uwb.outlier_count,
             state_name(rec.state),
             rec.algo_v_mps,
             rec.algo_w_rps,
             rec.ramp_v_mps,
             rec.ramp_w_rps,
             rec.applied_v_mps,
             rec.applied_w_rps,
#if FR_USE_DIRECT_PULSE_CONTROL
             "true",
#else
             "false",
#endif
             rec.direct_left_mps,
             rec.direct_right_mps,
             rec.direct_left_us,
             rec.direct_right_us,
             rec.chassis_update_ret,
             rec.chassis_pulse_ret,
             rec.meas_v_mps,
             rec.meas_w_rps,
             (unsigned long)s_log_dropped,
             FR_LOG_PATH);

    if (n < 0 || n >= FR_LIVE_JSON_BUF_SIZE) {
        return http_send_text(req, "{\"error\":\"live_json_truncated\"}\n", "application/json");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t log_download_handler(httpd_req_t *req)
{
    FILE *f = fopen(FR_LOG_PATH, "r");
    if (!f) {
        return http_send_text(req, "log file not available\n", "text/plain");
    }

    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=follow_log.csv");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char chunk[768];
    while (!feof(f)) {
        size_t n = fread(chunk, 1, sizeof(chunk), f);
        if (n > 0) {
            if (httpd_resp_send_chunk(req, chunk, n) != ESP_OK) {
                fclose(f);
                return ESP_FAIL;
            }
        }
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t remote_http_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true;
    config.stack_size = 12288;
    config.max_uri_handlers = 12;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    esp_err_t ret = httpd_start(&s_httpd, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_uri_t uri_index      = {.uri = "/",           .method = HTTP_GET,  .handler = index_handler};
    httpd_uri_t uri_hb         = {.uri = "/hb",         .method = HTTP_POST, .handler = heartbeat_handler};
    httpd_uri_t uri_estop      = {.uri = "/estop",      .method = HTTP_POST, .handler = estop_handler};
    httpd_uri_t uri_clear      = {.uri = "/clear",      .method = HTTP_POST, .handler = clear_handler};
    httpd_uri_t uri_motion_off = {.uri = "/motion_off", .method = HTTP_POST, .handler = motion_off_handler};
    httpd_uri_t uri_live       = {.uri = "/live",       .method = HTTP_GET,  .handler = live_handler};
    httpd_uri_t uri_status     = {.uri = "/status",     .method = HTTP_GET,  .handler = live_handler};
    httpd_uri_t uri_log        = {.uri = "/log",        .method = HTTP_GET,  .handler = log_download_handler};
    httpd_uri_t uri_clear_log  = {.uri = "/clear_log",  .method = HTTP_POST, .handler = clear_log_handler};

    httpd_register_uri_handler(s_httpd, &uri_index);
    httpd_register_uri_handler(s_httpd, &uri_hb);
    httpd_register_uri_handler(s_httpd, &uri_estop);
    httpd_register_uri_handler(s_httpd, &uri_clear);
    httpd_register_uri_handler(s_httpd, &uri_motion_off);
    httpd_register_uri_handler(s_httpd, &uri_live);
    httpd_register_uri_handler(s_httpd, &uri_status);
    httpd_register_uri_handler(s_httpd, &uri_log);
    httpd_register_uri_handler(s_httpd, &uri_clear_log);

    ESP_LOGI(TAG, "HTTP ready: http://192.168.4.1");
    return ESP_OK;
}

static void remote_wifi_event_handler(void *arg,
                                      esp_event_base_t event_base,
                                      int32_t event_id,
                                      void *event_data)
{
    (void)arg;
    if (event_base != WIFI_EVENT) {
        return;
    }

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "phone/client connected, AID=%d", e->aid);
        remote_set_client_connected(true, now_us());
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGW(TAG, "phone/client disconnected, AID=%d -> E-STOP", e->aid);
        remote_set_client_connected(false, now_us());
    }
}

static esp_err_t remote_wifi_start_softap(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) return ret;

    ret = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &remote_wifi_event_handler,
                                              NULL,
                                              NULL);
    if (ret != ESP_OK) return ret;

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strncpy((char *)wifi_config.ap.ssid, FR_REMOTE_AP_SSID, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, FR_REMOTE_AP_PASS, sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.ssid_len = strlen(FR_REMOTE_AP_SSID);
    wifi_config.ap.channel = FR_REMOTE_AP_CHANNEL;
    wifi_config.ap.max_connection = FR_REMOTE_AP_MAX_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.pmf_cfg.required = false;

    if (strlen(FR_REMOTE_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP ready: SSID=%s PASS=%s URL=http://192.168.4.1",
             FR_REMOTE_AP_SSID, FR_REMOTE_AP_PASS);

    return remote_http_start();
}

static void remote_wait_for_startup_client(void)
{
    uint32_t last_log_ms = 0;
    while (!remote_startup_ready()) {
        uint32_t now_ms = (uint32_t)(now_us() / 1000ULL);
        if (now_ms - last_log_ms >= FR_STARTUP_LOG_INTERVAL_MS) {
            last_log_ms = now_ms;
            ESP_LOGW(TAG, "waiting for phone hotspot client + web heartbeat before robot bring-up...");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "phone connected and heartbeat received; continuing robot bring-up");
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
    int flash_log_div = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        const uint64_t t = now_us();
        const float dt = (float)(t - prev_us) / 1e6f;
        prev_us = t;

        /* ---- Fast-path E-stop: bypass all sensor/algorithm logic ---- */
        if (s_estop_pending) {
            s_estop_pending = false;
            ramp_v_cmd = 0.0f;
            ramp_w_cmd = 0.0f;
            chassis_set_velocity(chassis, 0.0f, 0.0f);
            chassis_update(chassis, dt);
            state = FOLLOW_STATE_IDLE;
            continue;   /* skip the rest of this cycle */
        }

        bool tgt_valid = false;
        float tgt_dist = 0.0f;
        float tgt_bear = 0.0f;
        uint32_t tgt_age_ms = 0xffffffffu;
        uwb_debug_t uwb;
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

            float err = tgt_dist - follow_distance_m;
            if (err > 0.0f) {
                algo_v = kp_dist * err;
            } else if (-err <= stop_band_m) {
                algo_v = 0.0f;
            } else {
                algo_v = 0.0f;
            }
            algo_v = clampf(algo_v, 0.0f, max_linear_mps);

            algo_w = kp_bear * tgt_bear;
            algo_w = clampf(algo_w, -max_angular_rps, max_angular_rps);

            float turn_scale = clampf(1.0f - fabsf(tgt_bear) / (0.5f * (float)M_PI), 0.0f, 1.0f);
            algo_v *= turn_scale;

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

        /* Optional extra debug limits before ramping.
         * Disabled by default: normal speed is governed by Kconfig limits
         * max_linear_mps / max_angular_rps above.
         */
#if FR_USE_MOTION_DEBUG_LIMITS
        algo_v = clampf(algo_v, 0.0f, FR_MOTION_DEBUG_MAX_LINEAR_MPS);
        algo_w = clampf(algo_w, -FR_MOTION_DEBUG_MAX_ANGULAR_RPS,
                        FR_MOTION_DEBUG_MAX_ANGULAR_RPS);
#endif

        bool motion_allowed = remote_motion_allowed(t);
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
        /* Direct PWM debug mode:
         * - chassis_update() is called first only to refresh encoder-derived meas_v/meas_w.
         * - Then chassis_set_pulse_us() writes final ESC pulses directly.
         * - Do not call chassis_set_velocity() here, otherwise speed PID would own the output.
         */
        update_ret = chassis_update(chassis, dt);
        pulse_ret = chassis_set_pulse_us(chassis, direct_left_us, direct_right_us);
#else
        pulse_ret = chassis_set_velocity(chassis, applied_v, applied_w);
        update_ret = chassis_update(chassis, dt);
#endif

        float mv = 0.0f;
        float mw = 0.0f;
        chassis_get_measured(chassis, &mv, &mw, NULL, NULL);

        remote_state_t r;
        remote_get_snapshot(&r);
        uint32_t hb_age_ms = 0xffffffffu;
        if (r.last_heartbeat_us != 0 && t >= r.last_heartbeat_us) {
            hb_age_ms = (uint32_t)((t - r.last_heartbeat_us) / 1000ULL);
        }

        telemetry_record_t rec;
        memset(&rec, 0, sizeof(rec));
        rec.t_us = t;
        rec.remote_client_connected = r.client_connected;
        rec.remote_estop_latched = r.estop_latched;
        rec.remote_motion_armed = r.motion_armed;
        rec.remote_motion_allowed = motion_allowed;
        rec.remote_hb_age_ms = hb_age_ms;
        rec.remote_hb_count = r.heartbeat_count;
        rec.state = state;
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
        rec.direct_left_mps = direct_left_mps;
        rec.direct_right_mps = direct_right_mps;
        rec.direct_left_us = direct_left_us;
        rec.direct_right_us = direct_right_us;
        rec.chassis_update_ret = (int)update_ret;
        rec.chassis_pulse_ret = (int)pulse_ret;
        rec.meas_v_mps = mv;
        rec.meas_w_rps = mw;
        rec.log_dropped = s_log_dropped;

        live_set(&rec);

        if (++flash_log_div >= CONFIG_FOLLOW_ONLY_CONTROL_HZ / FR_LOG_HZ) {
            flash_log_div = 0;
            flash_log_enqueue(&rec);
        }

        if (++serial_log_div >= CONFIG_FOLLOW_ONLY_CONTROL_HZ / 5) {
            serial_log_div = 0;
            ESP_LOGI(TAG,
                     "%s tgt=%s age=%lums raw=(%.2f,%.2f) filt d=%.2f br=%+.1fdeg | "
                     "algo v=%+.2f w=%+.2f | applied v=%+.2f w=%+.2f | "
                     "direct L=%+.2fm/s %dus R=%+.2fm/s %dus | "
                     "meas v=%+.2f w=%+.2f | ret upd=%d pulse=%d | armed=%d motion=%d",
                     state_name(state), tgt_valid ? "Y" : "N", (unsigned long)tgt_age_ms,
                     uwb.raw_fwd_m, uwb.raw_left_m,
                     uwb.filt_range_m, RAD2DEG(uwb.filt_bearing_rad),
                     algo_v, algo_w, applied_v, applied_w,
                     direct_left_mps, direct_left_us, direct_right_mps, direct_right_us,
                     mv, mw, (int)update_ret, (int)pulse_ret,
                     r.motion_armed, motion_allowed);
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

    /* 1. 先启动手机连接入口。未连接页面前，不初始化底盘和传感器。 */
    if (remote_wifi_start_softap() != ESP_OK) {
        ESP_LOGE(TAG, "remote WiFi/HTTP start FAILED; robot bring-up aborted");
        return;
    }

    remote_wait_for_startup_client();

    /* 2. 手机在线后才启动日志、底盘、UWB。 */
    if (flash_log_start() != ESP_OK) {
        ESP_LOGE(TAG, "flash log disabled because SPIFFS/log init failed");
    }

    if (chassis_bringup() != ESP_OK) {
        ESP_LOGE(TAG, "chassis failed; control task not started");
        return;
    }

    if (uwb_bringup() != ESP_OK) {
        ESP_LOGE(TAG, "UWB failed; control task will still run but target stays invalid");
    }

    imu_bringup();

    /* 3. 默认 motion_armed=false / estop_latched=true；CLEAR / ARM 后保持 true，STOP 后 false。 */
    remote_motion_off(now_us());
    remote_estop(now_us());

    xTaskCreate(control_task, "control", 6144, &s_chassis, 7, NULL);
    ESP_LOGI(TAG,
             "control loop running at %d Hz; FR_ENABLE_MOTION=%d, direct_pulse=%d, extra_debug_limits=%d, "
             "Kconfig max v=%.2f m/s w=%.2f rad/s",
             CONFIG_FOLLOW_ONLY_CONTROL_HZ, FR_ENABLE_MOTION,
             FR_USE_DIRECT_PULSE_CONTROL,
             FR_USE_MOTION_DEBUG_LIMITS,
             CONFIG_FOLLOW_ONLY_MAX_LINEAR_MMPS / 1000.0f,
             CONFIG_FOLLOW_ONLY_MAX_ANGULAR_MRADPS / 1000.0f);
}
