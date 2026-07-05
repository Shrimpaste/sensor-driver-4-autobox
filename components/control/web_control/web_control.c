/*
 * web_control.c - WiFi SoftAP + HTTP 2D remote control implementation.
 * Fixes: session/seq protocol, force-zero on state transitions, /cmd hardening.
 */

#include "web_control.h"

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

static const char *TAG = "web_control";

/* ================================================================ config */

wctl_config_t web_control_default_config(void)
{
    wctl_config_t c;
    memset(&c, 0, sizeof(c));
    c.ap_ssid = "RemoteBox";
    c.ap_pass = "12345678";
    c.ap_channel = 6;
    c.ap_max_conn = 1;
    c.hb_timeout_us = 1500000ULL;
    c.log_partition_label = "logs";
    c.log_base_path = "/spiffs";
    c.log_file_path = "/spiffs/remote_log.csv";
    c.log_queue_len = 96;
    c.log_hz = 10;
    c.cmd_timeout_ms = 500;
    return c;
}

/* ================================================================ internal state */

static wctl_config_t s_cfg;
static httpd_handle_t s_httpd = NULL;

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

static volatile bool s_estop_pending = false;

/* ---- Remote command (spinlock-protected) ---- */
static wctl_cmd_t s_cmd;
static portMUX_TYPE s_cmd_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_cmd_session = 0;

/* ---- Telemetry ---- */
static wctl_record_t s_live;
static portMUX_TYPE s_live_mux = portMUX_INITIALIZER_UNLOCKED;

/* ---- Flash log ---- */
static QueueHandle_t s_log_q = NULL;
static uint32_t s_log_dropped = 0;
static volatile bool s_clear_log_requested = false;

/* ================================================================ helpers */

static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Force the remote command to zero for the current session. */
static void force_zero_cmd_locked(void)
{
    /* Caller must hold s_cmd_mux or s_remote_mux (s_cmd_session update). */
    s_cmd.x_norm = 0.0f;
    s_cmd.y_norm = 0.0f;
    s_cmd.deadman = false;
    s_cmd.valid = true;
    s_cmd.seq = 0;               /* reset seq for new session context */
    s_cmd.rx_us = now_us();
    /* session stays the same — caller sets it if needed */
}

/* ================================================================ remote state */

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

    /* P0-2: force zero command on estop. */
    portENTER_CRITICAL(&s_cmd_mux);
    force_zero_cmd_locked();
    portEXIT_CRITICAL(&s_cmd_mux);
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
    s_cmd_session++;
    portEXIT_CRITICAL(&s_remote_mux);

    /* P0-2: force zero command, new session, seq=0. */
    portENTER_CRITICAL(&s_cmd_mux);
    s_cmd.session = s_cmd_session;
    force_zero_cmd_locked();
    portEXIT_CRITICAL(&s_cmd_mux);
}

static void remote_motion_off(uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.motion_armed = false;
    s_remote.last_cmd_us = t;
    s_remote.motion_off_count++;
    portEXIT_CRITICAL(&s_remote_mux);

    /* P0-2: force zero command. */
    portENTER_CRITICAL(&s_cmd_mux);
    force_zero_cmd_locked();
    portEXIT_CRITICAL(&s_cmd_mux);
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

    if (t - r.last_heartbeat_us > s_cfg.hb_timeout_us) {
        remote_estop(t);
        s_estop_pending = true;
        ESP_LOGW(TAG, "heartbeat timeout -> E-STOP");
        return false;
    }
    return true;
}

/* ================================================================ remote command */

void web_control_set_remote_cmd(const wctl_cmd_t *cmd)
{
    portENTER_CRITICAL(&s_cmd_mux);
    s_cmd = *cmd;
    portEXIT_CRITICAL(&s_cmd_mux);
}

bool web_control_get_remote_cmd(wctl_cmd_t *cmd)
{
    portENTER_CRITICAL(&s_cmd_mux);
    *cmd = s_cmd;
    portEXIT_CRITICAL(&s_cmd_mux);
    return cmd->valid;
}

uint32_t web_control_get_session(void)
{
    return s_cmd_session;
}

/* ================================================================ telemetry */

static void live_set(const wctl_record_t *rec)
{
    portENTER_CRITICAL(&s_live_mux);
    s_live = *rec;
    portEXIT_CRITICAL(&s_live_mux);
}

static void live_get(wctl_record_t *out)
{
    portENTER_CRITICAL(&s_live_mux);
    *out = s_live;
    portEXIT_CRITICAL(&s_live_mux);
}

/* ================================================================ flash log */

static void flash_log_write_header(FILE *f)
{
    fprintf(f,
            "t_ms,client,estop,armed,ok,hb_age_ms,session,"
            "cmd_x,cmd_y,cmd_seq,cmd_age_ms,cmd_deadman,"
            "target_v,target_w,applied_v,applied_w,"
            "tgt_left,tgt_right,cmd_left_us,cmd_right_us,"
            "meas_left,meas_right,safety,encoders,log_dropped\n");
}

static FILE *flash_log_open_append(void)
{
    struct stat st;
    bool need_header = (stat(s_cfg.log_file_path, &st) != 0) || (st.st_size == 0);
    FILE *f = fopen(s_cfg.log_file_path, "a");
    if (!f) { ESP_LOGE(TAG, "open log: %s", strerror(errno)); return NULL; }
    if (need_header) { flash_log_write_header(f); fflush(f); }
    return f;
}

static void flash_log_task(void *arg)
{
    (void)arg;
    FILE *f = flash_log_open_append();
    wctl_record_t rec;
    int flush_div = 0;

    while (1) {
        if (s_clear_log_requested) {
            s_clear_log_requested = false;
            if (f) { fclose(f); f = NULL; }
            remove(s_cfg.log_file_path);
            f = flash_log_open_append();
        }
        if (xQueueReceive(s_log_q, &rec, pdMS_TO_TICKS(500)) != pdTRUE) {
            if (f) fflush(f);
            continue;
        }
        if (!f) { f = flash_log_open_append(); if (!f) continue; }

        fprintf(f,
                "%llu,%d,%d,%d,%d,%lu,%lu,"
                "%.3f,%.3f,%lu,%lu,%d,"
                "%.3f,%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.1f,%.1f,"
                "%.3f,%.3f,%s,%d,%lu\n",
                (unsigned long long)(rec.t_us / 1000ULL),
                rec.client_connected, rec.estop_latched, rec.motion_armed,
                rec.motion_allowed, (unsigned long)rec.hb_age_ms,
                (unsigned long)rec.cmd_session,
                rec.cmd_x, rec.cmd_y, (unsigned long)rec.cmd_seq,
                (unsigned long)rec.cmd_age_ms, rec.cmd_deadman,
                rec.target_v_mps, rec.target_w_rps,
                rec.applied_v_mps, rec.applied_w_rps,
                rec.target_left_mps, rec.target_right_mps,
                rec.cmd_left_us, rec.cmd_right_us,
                rec.meas_left_mps, rec.meas_right_mps,
                rec.safety_reason ? rec.safety_reason : "?",
                rec.using_encoders,
                (unsigned long)rec.log_dropped);

        if (++flush_div >= s_cfg.log_hz) { flush_div = 0; fflush(f); }
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
    size_t total = 0, used = 0;
    esp_spiffs_info(s_cfg.log_partition_label, &total, &used);
    ESP_LOGI(TAG, "SPIFFS: total=%u used=%u", (unsigned)total, (unsigned)used);

    s_log_q = xQueueCreate(s_cfg.log_queue_len, sizeof(wctl_record_t));
    if (!s_log_q) return ESP_ERR_NO_MEM;
    xTaskCreate(flash_log_task, "flash_log", 6144, NULL, 2, NULL);
    return ESP_OK;
}

/* ================================================================ HTTP handlers */

static esp_err_t http_send_text(httpd_req_t *req, const char *text, const char *type)
{
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
}

/* P0-2 + P1-1 + P1-2: /cmd hardened. */
static esp_err_t cmd_handler(httpd_req_t *req)
{
    /* P0-2: reject oversized bodies. */
    int total = req->content_len;
    if (total <= 0 || total > 511) {
        goto bad;
    }

    char buf[513];  /* +1 for NUL, spare room */
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, buf + received, total - received);
        if (r <= 0) goto bad;
        received += r;
    }
    buf[received] = '\0';

    /* P1-2: parse with found flags; reject if ANY field missing. */
    wctl_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.rx_us = now_us();

    bool f_session = false, f_seq = false, f_x = false;
    bool f_y = false, f_scale = false, f_deadman = false;

    #define FIND_INT(key, dst, found) do { \
        const char *p = strstr(buf, "\"" key "\""); \
        if (p) { p = strchr(p, ':'); if (p) { dst = (uint32_t)atol(p + 1); found = true; } } \
    } while(0)
    #define FIND_FLOAT(key, dst, found) do { \
        const char *p = strstr(buf, "\"" key "\""); \
        if (p) { p = strchr(p, ':'); if (p) { dst = (float)atof(p + 1); found = true; } } \
    } while(0)
    #define FIND_BOOL(key, dst, found) do { \
        const char *p = strstr(buf, "\"" key "\""); \
        if (p) { p = strchr(p, ':'); if (p) { dst = (strncmp(p+1, "true", 4) == 0); found = true; } } \
    } while(0)

    FIND_INT("session", cmd.session, f_session);
    FIND_INT("seq", cmd.seq, f_seq);
    FIND_FLOAT("x", cmd.x_norm, f_x);
    FIND_FLOAT("y", cmd.y_norm, f_y);
    FIND_FLOAT("scale", cmd.scale, f_scale);
    FIND_BOOL("deadman", cmd.deadman, f_deadman);

    #undef FIND_INT
    #undef FIND_FLOAT
    #undef FIND_BOOL

    /* P1-2: all fields required. */
    if (!f_session || !f_seq || !f_x || !f_y || !f_scale || !f_deadman) {
        goto bad;
    }

    /* P1-2: validate numeric values. */
    if (!isfinite(cmd.x_norm) || !isfinite(cmd.y_norm) || !isfinite(cmd.scale)) {
        goto bad;
    }

    cmd.x_norm = clampf(cmd.x_norm, -1.0f, 1.0f);
    cmd.y_norm = clampf(cmd.y_norm, -1.0f, 1.0f);
    cmd.scale = clampf(cmd.scale, 0.0f, 1.0f);

    /* Session check. */
    uint32_t cur_session = web_control_get_session();
    if (cmd.session != cur_session) {
        goto bad;
    }

    /* deadman=false forces zero. */
    if (!cmd.deadman) {
        cmd.x_norm = 0.0f;
        cmd.y_norm = 0.0f;
    }

    /* Per-session seq check. */
    {
        portENTER_CRITICAL(&s_cmd_mux);
        if (cmd.session == s_cmd.session && cmd.seq <= s_cmd.seq) {
            portEXIT_CRITICAL(&s_cmd_mux);
            goto bad;
        }
        portEXIT_CRITICAL(&s_cmd_mux);
    }

    /* Command accepted. */
    cmd.valid = true;
    web_control_set_remote_cmd(&cmd);

    {
        char resp[200];
        snprintf(resp, sizeof(resp),
                 "{\"ok\":true,\"accepted\":true,\"seq\":%lu,\"session\":%lu,"
                 "\"motion_allowed\":%s}",
                 (unsigned long)cmd.seq, (unsigned long)cur_session,
                 remote_motion_allowed() ? "true" : "false");
        return http_send_text(req, resp, "application/json");
    }

bad:
    /* P1-1: ALL error paths force zero to prevent stale non-zero. */
    {
        portENTER_CRITICAL(&s_cmd_mux);
        force_zero_cmd_locked();
        portEXIT_CRITICAL(&s_cmd_mux);
    }
    httpd_resp_set_status(req, "400 Bad Request");
    return http_send_text(req, "{\"ok\":false,\"err\":\"bad\"}\n", "application/json");
}

static esp_err_t cmd_zero_handler(httpd_req_t *req)
{
    wctl_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.rx_us = now_us();
    cmd.valid = true;
    cmd.deadman = false;
    cmd.session = web_control_get_session();

    portENTER_CRITICAL(&s_cmd_mux);
    cmd.seq = s_cmd.seq + 1;
    portEXIT_CRITICAL(&s_cmd_mux);

    web_control_set_remote_cmd(&cmd);

    char resp[120];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"session\":%lu,\"seq\":%lu}",
             (unsigned long)cmd.session, (unsigned long)cmd.seq);
    return http_send_text(req, resp, "application/json");
}

static esp_err_t hb_handler(httpd_req_t *req)
{
    remote_heartbeat(now_us());
    return http_send_text(req, "OK\n", "text/plain");
}

static esp_err_t estop_handler(httpd_req_t *req)
{
    s_estop_pending = true;
    remote_estop(now_us());
    return http_send_text(req, "ESTOP\n", "text/plain");
}

/* P0-1: /clear returns JSON with session. */
static esp_err_t clear_handler(httpd_req_t *req)
{
    remote_clear_and_arm(now_us());
    uint32_t session = web_control_get_session();
    char resp[80];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"armed\":true,\"session\":%lu}",
             (unsigned long)session);
    return http_send_text(req, resp, "application/json");
}

static esp_err_t motion_off_handler(httpd_req_t *req)
{
    remote_motion_off(now_us());
    return http_send_text(req, "MOTION_OFF\n", "text/plain");
}

static esp_err_t clear_log_handler(httpd_req_t *req)
{
    s_clear_log_requested = true;
    return http_send_text(req, "CLEAR_LOG_REQUESTED\n", "text/plain");
}

static char s_live_json_buf[4096];

static esp_err_t live_handler(httpd_req_t *req)
{
    wctl_record_t rec;
    live_get(&rec);

    remote_state_t r;
    remote_get_snapshot(&r);

    uint64_t t = now_us();
    uint32_t hb_age_ms = 0xffffffffu;
    if (r.last_heartbeat_us != 0 && t >= r.last_heartbeat_us) {
        hb_age_ms = (uint32_t)((t - r.last_heartbeat_us) / 1000ULL);
    }

    char *buf = s_live_json_buf;
    int n = snprintf(buf, 4096,
        "{"
        "\"remote\":{\"connected\":%s,\"estop\":%s,\"armed\":%s,\"ok\":%s,"
        "\"hb_age_ms\":%lu,\"hb_count\":%lu,\"session\":%lu},"
        "\"cmd\":{\"x\":%.3f,\"y\":%.3f,\"seq\":%lu,\"age_ms\":%lu,\"deadman\":%s},"
        "\"control\":{\"target_v\":%.3f,\"target_w\":%.3f,"
        "\"applied_v\":%.3f,\"applied_w\":%.3f},"
        "\"chassis\":{\"tgt_left\":%.3f,\"tgt_right\":%.3f,"
        "\"cmd_left_us\":%.1f,\"cmd_right_us\":%.1f,"
        "\"meas_left\":%.3f,\"meas_right\":%.3f},"
        "\"safety\":\"%s\",\"encoders\":%s}"
        "\n",
        r.client_connected ? "true" : "false",
        r.estop_latched ? "true" : "false",
        r.motion_armed ? "true" : "false",
        rec.motion_allowed ? "true" : "false",
        (unsigned long)hb_age_ms, (unsigned long)r.heartbeat_count,
        (unsigned long)rec.cmd_session,
        rec.cmd_x, rec.cmd_y, (unsigned long)rec.cmd_seq,
        (unsigned long)rec.cmd_age_ms, rec.cmd_deadman ? "true" : "false",
        rec.target_v_mps, rec.target_w_rps,
        rec.applied_v_mps, rec.applied_w_rps,
        rec.target_left_mps, rec.target_right_mps,
        rec.cmd_left_us, rec.cmd_right_us,
        rec.meas_left_mps, rec.meas_right_mps,
        rec.safety_reason ? rec.safety_reason : "?",
        rec.using_encoders ? "true" : "false");

    if (n < 0 || n >= 4096) {
        return http_send_text(req, "{\"error\":\"truncated\"}\n", "application/json");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t log_handler(httpd_req_t *req)
{
    FILE *f = fopen(s_cfg.log_file_path, "r");
    if (!f) return http_send_text(req, "no log\n", "text/plain");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=remote_log.csv");
    char chunk[512];
    while (!feof(f)) {
        size_t n = fread(chunk, 1, sizeof(chunk), f);
        if (n > 0 && httpd_resp_send_chunk(req, chunk, n) != ESP_OK) { fclose(f); return ESP_FAIL; }
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* ---- Embedded HTML (P0-1: session from server, P0-4: clearInputs) ---- */
static const char s_index_html[] =
"<!doctype html><html><head>"
"<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>"
"<title>Remote Box</title>"
"<style>"
"*{box-sizing:border-box;-webkit-user-select:none;user-select:none;}"
"body{font-family:Arial,sans-serif;background:#101214;color:#eee;margin:0;padding:12px;}"
"h2{text-align:center;margin:8px 0;}"
"#pad{display:grid;grid-template-columns:72px 72px 72px;grid-template-rows:72px 72px 72px;gap:4px;justify-content:center;margin:12px 0;}"
".btn{border:0;border-radius:12px;font-size:28px;font-weight:bold;display:flex;align-items:center;justify-content:center;touch-action:none;}"
"#b_fwd{background:#2d8cf0;grid-column:2;grid-row:1;}"
"#b_left{background:#555;grid-column:1;grid-row:2;}"
"#b_stop{background:#d00000;grid-column:2;grid-row:2;font-size:16px;}"
"#b_right{background:#555;grid-column:3;grid-row:2;}"
"#b_back{background:#555;grid-column:2;grid-row:3;}"
"#ctl{display:flex;gap:8px;justify-content:center;flex-wrap:wrap;margin:8px 0;}"
"#ctl button{font-size:16px;padding:10px 16px;border:0;border-radius:8px;}"
"#arm{background:#008f5a;color:white;}#off{background:#555;color:white;}"
"#clearlog{background:#7655d9;color:white;}"
"label{font-size:14px;}input[type=range]{width:140px;}"
"#info{background:#18202a;border:1px solid #38506a;border-radius:8px;padding:8px;margin:8px 0;font-size:13px;line-height:1.4;}"
"#err{color:#f66;font-weight:bold;font-size:13px;min-height:1.2em;}"
"a{color:#8cc8ff;}"
"</style></head><body>"
"<h2>Remote Box</h2>"
"<div id='pad'>"
"<div class='btn' id='b_fwd'>&#9650;</div>"
"<div class='btn' id='b_left'>&#9664;</div>"
"<div class='btn' id='b_stop'>STOP</div>"
"<div class='btn' id='b_right'>&#9654;</div>"
"<div class='btn' id='b_back'>&#9660;</div>"
"</div>"
"<div id='ctl'>"
"<label>Speed <input type='range' id='slider' min='20' max='100' value='40'></label>"
"<span id='sval'>40%</span>"
"<button id='arm'>CLEAR/ARM</button>"
"<button id='off'>MOT OFF</button>"
"<button id='clearlog'>CLR LOG</button>"
"<a href='/log' target='_blank'>CSV</a>"
"</div>"
"<div id='err'></div>"
"<div id='info'>connecting...</div>"
"<script>"
"const S={session:0,seq:0,scale:0.4,deadman:false};"
"const B={fwd:new Set(),back:new Set(),left:new Set(),right:new Set()};"
"function active(n){return B[n].size>0;}"
"function axes(){let x=0,y=0;"
"if(active('fwd')&&!active('back'))y=1;else if(active('back')&&!active('fwd'))y=-1;"
"if(active('left')&&!active('right'))x=1;else if(active('right')&&!active('left'))x=-1;"
"return{x,y};}"
"function clearInputs(){for(const k in B)B[k].clear();S.deadman=false;lastSig='';}"
"function bind(el,n){el.style.touchAction='none';"
"el.addEventListener('pointerdown',e=>{e.preventDefault();el.setPointerCapture(e.pointerId);B[n].add(e.pointerId);sendNow();});"
"function rel(e){e.preventDefault();B[n].delete(e.pointerId);sendNow();}"
"el.addEventListener('pointerup',rel);el.addEventListener('pointercancel',rel);el.addEventListener('lostpointercapture',rel);}"
"bind(document.getElementById('b_fwd'),'fwd');"
"bind(document.getElementById('b_back'),'back');"
"bind(document.getElementById('b_left'),'left');"
"bind(document.getElementById('b_right'),'right');"
"document.getElementById('b_stop').addEventListener('pointerdown',e=>{e.preventDefault();doStop();});"
"document.getElementById('slider').addEventListener('input',e=>{S.scale=e.target.value/100;document.getElementById('sval').textContent=e.target.value+'%';});"
"let inFlight=false,pending=false,lastSig='';"
"async function sendCmd(force){"
"const{x,y}=axes();"
"const deadman=S.deadman||x!==0||y!==0;"
"const sig=JSON.stringify([x,y,deadman,S.scale]);"
"if(!force&&sig===lastSig)return;"
"if(inFlight){pending=true;return;}"
"inFlight=true;pending=false;lastSig=sig;"
"try{const ac=new AbortController();const t=setTimeout(()=>ac.abort(),120);"
"const r=await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({session:S.session,seq:++S.seq,x,y,scale:S.scale,deadman}),"
"cache:'no-store',signal:ac.signal});clearTimeout(t);"
"const j=await r.json();if(j.session!==undefined)S.session=j.session;"
"}catch(_){}finally{inFlight=false;if(pending)sendCmd(true);}}"
"function sendNow(){sendCmd(true);}"
"setInterval(()=>{const{x,y}=axes();if(x||y||S.deadman)sendCmd(true);},50);"
"function sendZero(){clearInputs();"
"fetch('/cmd_zero',{method:'POST',keepalive:true}).then(r=>r.json()).then(j=>{"
"if(j.session!==undefined)S.session=j.session;if(j.seq!==undefined)S.seq=j.seq;"
"}).catch(()=>{});}"
"window.addEventListener('blur',sendZero);"
"window.addEventListener('beforeunload',sendZero);"
"document.addEventListener('visibilitychange',()=>{if(document.hidden)sendZero();});"
"async function doStop(){clearInputs();"
"await fetch('/estop',{method:'POST'}).catch(()=>{});poll();}"
"async function doArm(){clearInputs();"
"try{const r=await fetch('/clear',{method:'POST'});const j=await r.json();"
"if(j.session!==undefined)S.session=j.session;S.seq=0;"
"}catch(_){}poll();}"
"async function doOff(){clearInputs();"
"await fetch('/motion_off',{method:'POST'}).catch(()=>{});poll();}"
"function doClearLog(){fetch('/clear_log',{method:'POST'}).catch(()=>{});}"
"let _pc=null;"
"async function poll(){"
"if(_pc)_pc.abort();_pc=new AbortController();"
"try{const r=await fetch('/live',{cache:'no-store',signal:_pc.signal});"
"const j=await r.json();"
"const rm=j.remote||{},cm=j.cmd||{},ct=j.control||{},ch=j.chassis||{};"
"document.getElementById('err').textContent='';"
"document.getElementById('info').innerHTML="
"'session='+rm.session+' cmd x='+cm.x.toFixed(2)+' y='+cm.y.toFixed(2)+' seq='+cm.seq+' age='+cm.age_ms+'ms<br>'+'"
"v='+ct.target_v.toFixed(2)+' w='+ct.target_w.toFixed(2)+'<br>'+'"
"meas L/R='+ch.meas_left.toFixed(2)+'/'+ch.meas_right.toFixed(2)+' m/s<br>'+'"
"safety: '+j.safety+' | encoders='+j.encoders;"
"}catch(e){if(e.name!=='AbortError')document.getElementById('info').textContent='连接中断';}}"
"setInterval(()=>fetch('/hb',{method:'POST'}).catch(()=>{}),500);"
"setInterval(poll,200);poll();"
"</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    return http_send_text(req, s_index_html, "text/html");
}

/* ================================================================ WiFi + HTTP */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base != WIFI_EVENT) return;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "client connected AID=%d", e->aid);
        remote_set_client_connected(true, now_us());
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGW(TAG, "client disconnected AID=%d -> E-STOP", e->aid);
        remote_set_client_connected(false, now_us());
    }
}

/* Captive portal: Android sends this to detect internet. Return 204. */
static esp_err_t captive_portal_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* 404 handler: redirect all unknown URIs to root page (solves captive portal). */
static esp_err_t notfound_handler(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    return http_send_text(req, s_index_html, "text/html");
}

static esp_err_t http_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true;
    config.stack_size = 12288;
    config.max_uri_handlers = 14;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    esp_err_t ret = httpd_start(&s_httpd, &config);
    if (ret != ESP_OK) return ret;

    /* Register 404 error handler (catches all unmatched URIs). */
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, notfound_handler);

    httpd_uri_t u;
    memset(&u, 0, sizeof(u));

    /* Captive portal detection endpoints. */
    u.uri = "/generate_204"; u.method = HTTP_GET; u.handler = captive_portal_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/gen_204"; u.method = HTTP_GET; u.handler = captive_portal_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/ncsi.txt"; u.method = HTTP_GET; u.handler = captive_portal_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/hotspot-detect.html"; u.method = HTTP_GET; u.handler = captive_portal_handler;
    httpd_register_uri_handler(s_httpd, &u);

    u.uri = "/"; u.method = HTTP_GET; u.handler = index_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/cmd"; u.method = HTTP_POST; u.handler = cmd_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/cmd_zero"; u.method = HTTP_POST; u.handler = cmd_zero_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/hb"; u.method = HTTP_POST; u.handler = hb_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/estop"; u.method = HTTP_POST; u.handler = estop_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/clear"; u.method = HTTP_POST; u.handler = clear_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/motion_off"; u.method = HTTP_POST; u.handler = motion_off_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/live"; u.method = HTTP_GET; u.handler = live_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/log"; u.method = HTTP_GET; u.handler = log_handler;
    httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/clear_log"; u.method = HTTP_POST; u.handler = clear_log_handler;
    httpd_register_uri_handler(s_httpd, &u);

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
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) return ret;

    wifi_config_t wc;
    memset(&wc, 0, sizeof(wc));
    strncpy((char *)wc.ap.ssid, s_cfg.ap_ssid, sizeof(wc.ap.ssid) - 1);
    strncpy((char *)wc.ap.password, s_cfg.ap_pass, sizeof(wc.ap.password) - 1);
    wc.ap.ssid_len = strlen(s_cfg.ap_ssid);
    wc.ap.channel = s_cfg.ap_channel;
    wc.ap.max_connection = s_cfg.ap_max_conn;
    wc.ap.authmode = (strlen(s_cfg.ap_pass) > 0) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wc.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "SoftAP: SSID=%s", s_cfg.ap_ssid);
    return http_start();
}

/* ================================================================ public API */

esp_err_t web_control_init(const wctl_config_t *cfg)
{
    if (cfg) s_cfg = *cfg; else s_cfg = web_control_default_config();
    memset(&s_live, 0, sizeof(s_live));
    memset(&s_cmd, 0, sizeof(s_cmd));

    esp_err_t ret = wifi_start_softap();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "WiFi/HTTP FAILED"); return ret; }
    if (s_cfg.log_hz > 0) {
        flash_log_start();
    }
    remote_motion_off(now_us());
    remote_estop(now_us());
    return ESP_OK;
}

esp_err_t web_control_wait_startup(uint32_t timeout_ms)
{
    uint32_t last_log_ms = 0;
    uint64_t start = now_us();
    while (!remote_startup_ready()) {
        if (timeout_ms > 0 && (uint32_t)((now_us() - start) / 1000ULL) >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        uint32_t now_ms = (uint32_t)(now_us() / 1000ULL);
        if (now_ms - last_log_ms >= 1000) {
            last_log_ms = now_ms;
            ESP_LOGW(TAG, "waiting for phone + heartbeat...");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "phone connected");
    return ESP_OK;
}

void web_control_update(const wctl_record_t *rec)
{
    wctl_record_t enriched = *rec;

    remote_state_t r;
    remote_get_snapshot(&r);
    enriched.client_connected = r.client_connected;
    enriched.estop_latched = r.estop_latched;
    enriched.motion_armed = r.motion_armed;
    enriched.motion_allowed = remote_motion_allowed();
    uint64_t t = now_us();
    if (r.last_heartbeat_us != 0 && t >= r.last_heartbeat_us) {
        enriched.hb_age_ms = (uint32_t)((t - r.last_heartbeat_us) / 1000ULL);
    }

    enriched.cmd_session = s_cmd_session;
    wctl_cmd_t cmd;
    web_control_get_remote_cmd(&cmd);
    enriched.cmd_x = cmd.x_norm;
    enriched.cmd_y = cmd.y_norm;
    enriched.cmd_seq = cmd.seq;
    enriched.cmd_deadman = cmd.deadman;
    if (cmd.rx_us != 0 && t >= cmd.rx_us) {
        enriched.cmd_age_ms = (uint32_t)((t - cmd.rx_us) / 1000ULL);
    }
    enriched.log_dropped = s_log_dropped;

    live_set(&enriched);

    /* Flash log: only if enabled. */
    if (s_cfg.log_hz > 0 && s_log_q != NULL) {
        static uint64_t last_log_us = 0;
        const uint64_t interval = 1000000ULL / (uint64_t)s_cfg.log_hz;
        if (t - last_log_us >= interval) {
            last_log_us = t;
            if (xQueueSend(s_log_q, &enriched, 0) != pdTRUE) {
                s_log_dropped++;
            }
        }
    }
}

bool web_control_motion_allowed(void) { return remote_motion_allowed(); }

bool web_control_estop_pending(void)
{
    if (s_estop_pending) { s_estop_pending = false; return true; }
    return false;
}

void web_control_arm(void)        { remote_clear_and_arm(now_us()); }
void web_control_estop(void)      { s_estop_pending = true; remote_estop(now_us()); }
void web_control_motion_off(void) { remote_motion_off(now_us()); }
void web_control_clear_log(void)  { s_clear_log_requested = true; }
