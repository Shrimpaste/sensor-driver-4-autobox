/*
 * web_debug.c - WiFi SoftAP + HTTP debug dashboard implementation.
 */

#include "web_debug.h"

#include <errno.h>
#include <math.h>
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

static const char *TAG = "web_debug";

#define RAD2DEG(r) ((float)(r) * 180.0f / (float)M_PI)

/* ================================================================ config */

wdbg_config_t web_debug_default_config(void)
{
    wdbg_config_t c;
    memset(&c, 0, sizeof(c));
    c.ap_ssid = "FollowRobot-UWB";
    c.ap_pass = "12345678";
    c.ap_channel = 6;
    c.ap_max_conn = 1;
    c.log_partition_label = "logs";
    c.log_base_path = "/spiffs";
    c.log_file_path = "/spiffs/follow_log.csv";
    c.log_queue_len = 96;
    c.log_hz = 5;
    c.hb_timeout_us = 1500000ULL;  /* 1.5 seconds */
    return c;
}

/* ================================================================ internal state */

static wdbg_config_t s_cfg;
static httpd_handle_t s_httpd = NULL;

/* ---- Remote state (spinlock-protected) ---- */
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

/* Immediate E-stop flag: set by HTTP handler, polled by control_task. */
static volatile bool s_estop_pending = false;

/* ---- Telemetry (spinlock-protected) ---- */
static wdbg_record_t s_live;
static portMUX_TYPE s_live_mux = portMUX_INITIALIZER_UNLOCKED;

/* ---- Flash log ---- */
static QueueHandle_t s_log_q = NULL;
static uint32_t s_log_dropped = 0;
static volatile bool s_clear_log_requested = false;

/* ================================================================ remote helpers */

static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

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
        /* WiFi 断连 = 立即 E-stop，不依赖 heartbeat 超时。 */
        s_remote.estop_latched = true;
        s_remote.motion_armed = false;
    }
    portEXIT_CRITICAL(&s_remote_mux);
    if (!connected) {
        s_estop_pending = true;
    }
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

static bool remote_motion_allowed(void)
{
    const uint64_t t = now_us();
    remote_state_t r;
    remote_get_snapshot(&r);

    if (r.estop_latched || !r.motion_armed) return false;
    if (!r.client_connected) return false;
    if (r.last_heartbeat_us == 0) return false;

    /* Heartbeat timeout: latch E-stop and trigger hard stop. */
    if (t - r.last_heartbeat_us > s_cfg.hb_timeout_us) {
        remote_estop(t);
        s_estop_pending = true;
        ESP_LOGW(TAG, "heartbeat timeout -> E-STOP latched");
        return false;
    }

    return true;
}

/* ================================================================ telemetry helpers */

static void live_set(const wdbg_record_t *rec)
{
    portENTER_CRITICAL(&s_live_mux);
    s_live = *rec;
    portEXIT_CRITICAL(&s_live_mux);
}

static void live_get(wdbg_record_t *out)
{
    portENTER_CRITICAL(&s_live_mux);
    *out = s_live;
    portEXIT_CRITICAL(&s_live_mux);
}

/* ================================================================ flash log */

static const char *state_name(wdbg_state_t s)
{
    switch (s) {
    case WDBG_STATE_IDLE:   return "IDLE";
    case WDBG_STATE_SEARCH: return "SEARCH";
    case WDBG_STATE_FOLLOW: return "FOLLOW";
    default:                return "?";
    }
}

static const char *uwb_frame_type_name(wdbg_uwb_frame_type_t t)
{
    switch (t) {
    case WDBG_UWB_FRAME_TWR:        return "twr";
    case WDBG_UWB_FRAME_RANGE_ONLY: return "range_only";
    case WDBG_UWB_FRAME_PARSE_ERROR:return "parse_error";
    case WDBG_UWB_FRAME_NONE:
    default:                        return "none";
    }
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
            "applied_v_mps,applied_w_rps,"
            "target_left_mps,target_right_mps,cmd_left_us,cmd_right_us,"
            "meas_left_mps,meas_right_mps,chassis_update_ret,chassis_pulse_ret,"
            "log_dropped\n");
}

static FILE *flash_log_open_append(void)
{
    struct stat st;
    bool need_header = (stat(s_cfg.log_file_path, &st) != 0) || (st.st_size == 0);

    FILE *f = fopen(s_cfg.log_file_path, "a");
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
    wdbg_record_t rec;
    int flush_div = 0;

    while (1) {
        if (s_clear_log_requested) {
            s_clear_log_requested = false;
            if (f) {
                fclose(f);
                f = NULL;
            }
            remove(s_cfg.log_file_path);
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
                "%.3f,%.3f,"
                "%.3f,%.3f,%.1f,%.1f,"
                "%.3f,%.3f,%d,%d,"
                "%lu\n",
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
                rec.target_left_mps,
                rec.target_right_mps,
                rec.cmd_left_us,
                rec.cmd_right_us,
                rec.meas_left_mps,
                rec.meas_right_mps,
                rec.chassis_update_ret,
                rec.chassis_pulse_ret,
                (unsigned long)rec.log_dropped);

        if (++flush_div >= s_cfg.log_hz) {
            flush_div = 0;
            fflush(f);
        }
    }
}

static esp_err_t flash_log_start(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = s_cfg.log_base_path,
        .partition_label = s_cfg.log_partition_label,
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
    ret = esp_spiffs_info(s_cfg.log_partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "log SPIFFS mounted: total=%u used=%u", (unsigned)total, (unsigned)used);
    }

    s_log_q = xQueueCreate(s_cfg.log_queue_len, sizeof(wdbg_record_t));
    if (!s_log_q) {
        ESP_LOGE(TAG, "log queue create FAILED");
        return ESP_ERR_NO_MEM;
    }

    xTaskCreate(flash_log_task, "flash_log", 6144, NULL, 2, NULL);
    return ESP_OK;
}

static void flash_log_enqueue(const wdbg_record_t *rec)
{
    if (!s_log_q) {
        return;
    }
    if (xQueueSend(s_log_q, rec, 0) != pdTRUE) {
        s_log_dropped++;
    }
}

/* ================================================================ HTTP handlers */

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
"<p class='warn'>Motion requires ARM + fresh heartbeat. STOP / disconnect / timeout latches E-stop.</p>"
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
"'chassis: tgt L/R='+f(c.target_left_mps,2)+'/'+f(c.target_right_mps,2)+' m/s, cmd L/R='+f(c.cmd_left_us,1)+'/'+f(c.cmd_right_us,1)+' us, meas L/R='+f(c.meas_left_mps,2)+'/'+f(c.meas_right_mps,2)+' m/s';"
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
    s_estop_pending = true;
    remote_estop(now_us());
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

static char s_live_json_buf[8192];

static esp_err_t live_handler(httpd_req_t *req)
{
    const uint64_t t = now_us();
    const bool mot_ok = remote_motion_allowed();

    wdbg_record_t rec;
    remote_state_t r;
    live_get(&rec);
    remote_get_snapshot(&r);

    uint32_t hb_age_ms = 0xffffffffu;
    if (r.last_heartbeat_us != 0 && t >= r.last_heartbeat_us) {
        hb_age_ms = (uint32_t)((t - r.last_heartbeat_us) / 1000ULL);
    }

    char *buf = s_live_json_buf;
    int n = snprintf(buf, 8192,
             "{"
             "\"remote\":{"
             "\"client_connected\":%s,\"estop_latched\":%s,\"motion_armed\":%s,"
             "\"motion_allowed\":%s,"
             "\"heartbeat_age_ms\":%lu,\"heartbeat_count\":%lu,"
             "\"connect_count\":%lu,\"disconnect_count\":%lu},"
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
             "\"target_left_mps\":%.3f,\"target_right_mps\":%.3f,"
             "\"cmd_left_us\":%.1f,\"cmd_right_us\":%.1f,"
             "\"meas_left_mps\":%.3f,\"meas_right_mps\":%.3f,"
             "\"chassis_update_ret\":%d,\"chassis_pulse_ret\":%d},"
             "\"log\":{\"dropped\":%lu,\"path\":\"%s\"}"
             "}\n",
             r.client_connected ? "true" : "false",
             r.estop_latched ? "true" : "false",
             r.motion_armed ? "true" : "false",
             mot_ok ? "true" : "false",
             (unsigned long)hb_age_ms,
             (unsigned long)r.heartbeat_count,
             (unsigned long)r.connect_count,
             (unsigned long)r.disconnect_count,
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
             rec.target_left_mps,
             rec.target_right_mps,
             rec.cmd_left_us,
             rec.cmd_right_us,
             rec.meas_left_mps,
             rec.meas_right_mps,
             rec.chassis_update_ret,
             rec.chassis_pulse_ret,
             (unsigned long)s_log_dropped,
             s_cfg.log_file_path);

    if (n < 0 || n >= 8192) {
        return http_send_text(req, "{\"error\":\"live_json_truncated\"}\n", "application/json");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t log_download_handler(httpd_req_t *req)
{
    FILE *f = fopen(s_cfg.log_file_path, "r");
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

/* ================================================================ WiFi + HTTP startup */

static void wifi_event_handler(void *arg,
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
        ESP_LOGW(TAG, "phone/client disconnected, AID=%d", e->aid);
        remote_set_client_connected(false, now_us());
    }
}

static esp_err_t http_start(void)
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

static esp_err_t wifi_start_softap(void)
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
                                              &wifi_event_handler,
                                              NULL,
                                              NULL);
    if (ret != ESP_OK) return ret;

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strncpy((char *)wifi_config.ap.ssid, s_cfg.ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, s_cfg.ap_pass, sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.ssid_len = strlen(s_cfg.ap_ssid);
    wifi_config.ap.channel = s_cfg.ap_channel;
    wifi_config.ap.max_connection = s_cfg.ap_max_conn;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.pmf_cfg.required = false;

    if (strlen(s_cfg.ap_pass) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP ready: SSID=%s PASS=%s URL=http://192.168.4.1",
             s_cfg.ap_ssid, s_cfg.ap_pass);

    return http_start();
}

/* ================================================================ public API */

esp_err_t web_debug_init(const wdbg_config_t *cfg)
{
    if (cfg) {
        s_cfg = *cfg;
    } else {
        s_cfg = web_debug_default_config();
    }

    memset(&s_live, 0, sizeof(s_live));

    esp_err_t ret = wifi_start_softap();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi/HTTP start FAILED");
        return ret;
    }

    ret = flash_log_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "flash log disabled (SPIFFS init failed)");
        /* non-fatal */
    }

    /* Default: motion disarmed until CLEAR/ARM from web page. */
    remote_motion_off(now_us());
    remote_estop(now_us());

    return ESP_OK;
}

esp_err_t web_debug_wait_startup(uint32_t timeout_ms)
{
    uint32_t last_log_ms = 0;
    uint64_t start = now_us();
    while (!remote_startup_ready()) {
        if (timeout_ms > 0) {
            uint32_t elapsed_ms = (uint32_t)((now_us() - start) / 1000ULL);
            if (elapsed_ms >= timeout_ms) {
                ESP_LOGE(TAG, "startup timeout");
                return ESP_ERR_TIMEOUT;
            }
        }
        uint32_t now_ms = (uint32_t)(now_us() / 1000ULL);
        if (now_ms - last_log_ms >= 1000) {
            last_log_ms = now_ms;
            ESP_LOGW(TAG, "waiting for phone hotspot client + web heartbeat...");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "phone connected and heartbeat received; continuing robot bring-up");
    return ESP_OK;
}

void web_debug_update(const wdbg_record_t *rec)
{
    /* Fill remote fields from the component's internal state so that the
     * CSV log and /live endpoint both have accurate remote diagnostics. */
    wdbg_record_t enriched = *rec;
    remote_state_t r;
    remote_get_snapshot(&r);
    enriched.remote_client_connected = r.client_connected;
    enriched.remote_estop_latched = r.estop_latched;
    enriched.remote_motion_armed = r.motion_armed;
    enriched.remote_motion_allowed = remote_motion_allowed();
    uint64_t t = now_us();
    if (r.last_heartbeat_us != 0 && t >= r.last_heartbeat_us) {
        enriched.remote_hb_age_ms = (uint32_t)((t - r.last_heartbeat_us) / 1000ULL);
    }
    enriched.remote_hb_count = r.heartbeat_count;
    enriched.log_dropped = s_log_dropped;

    /* Set the telemetry snapshot for /live endpoint. */
    live_set(&enriched);

    /* Enqueue to flash log at the configured rate (time-based). */
    static uint64_t last_log_us = 0;
    const uint64_t log_interval_us = 1000000ULL / s_cfg.log_hz;
    if (t - last_log_us >= log_interval_us) {
        last_log_us = t;
        flash_log_enqueue(&enriched);
    }
}

bool web_debug_motion_allowed(void)
{
    return remote_motion_allowed();
}

bool web_debug_estop_pending(void)
{
    if (s_estop_pending) {
        s_estop_pending = false;
        return true;
    }
    return false;
}

void web_debug_arm(void)
{
    remote_clear_and_arm(now_us());
}

void web_debug_estop(void)
{
    s_estop_pending = true;
    remote_estop(now_us());
}

void web_debug_motion_off(void)
{
    remote_motion_off(now_us());
}

void web_debug_clear_log(void)
{
    s_clear_log_requested = true;
}
