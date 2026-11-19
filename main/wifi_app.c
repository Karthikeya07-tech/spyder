#include "wifi_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "imu.h"
#include "lidar.h"
#include "live_map.h"

static const char *TAG = "wifi_app";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

static httpd_handle_t s_httpd;
static bool s_up;
static char s_ip[16] = "192.168.4.1";
static char s_json[6144];
static uint8_t s_pgm[LIVE_MAP_W * LIVE_MAP_H + 64];
static SemaphoreHandle_t s_json_lock;

static void cors(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static void drain(httpd_req_t *req)
{
    char tmp[64];
    int left = req->content_len;
    while (left > 0) {
        int n = httpd_req_recv(req, tmp, sizeof(tmp) < (size_t)left ? sizeof(tmp) : (size_t)left);
        if (n <= 0) {
            break;
        }
        left -= n;
    }
}

static esp_err_t send_text(httpd_req_t *req, const char *type, const char *body)
{
    cors(req);
    httpd_resp_set_type(req, type);
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static int fill_live_json(char *buf, size_t cap)
{
    live_state_t st;
    live_map_get_state(&st);
    uint16_t mm[LIVE_SCAN_BINS];
    live_map_copy_scan_mm(mm, LIVE_SCAN_BINS);
    float roll = 0, pitch = 0, yaw = st.yaw;
    if (st.imu) {
        imu_get_rpy(&roll, &pitch, &yaw);
    }

    char *p = buf;
    char *end = buf + cap;
#define PUT(fmt, ...) do { \
        int _n = snprintf(p, (size_t)(end - p), fmt, ##__VA_ARGS__); \
        if (_n < 0 || p + _n >= end) { return -1; } \
        p += _n; \
    } while (0)

    PUT("{\"seq\":%lu,\"map_version\":%lu,\"streaming\":%s,\"lidar\":%s,\"frame\":{"
        "\"x\":\"forward after calibrate\",\"y\":\"left\",\"yaw\":\"CCW deg, 0=front arrow\"},",
        (unsigned long)st.scan_seq, (unsigned long)st.map_version,
        st.streaming ? "true" : "false", st.lidar ? "true" : "false");
    PUT("\"pose\":{\"x\":%.3f,\"y\":%.3f,\"yaw_deg\":%.2f},", st.x, st.y, yaw * 180.0f / 3.14159265f);
    PUT("\"imu\":{\"ready\":%s,\"calibrated\":%s,\"still\":%s,\"roll_deg\":%.1f,\"pitch_deg\":%.1f,"
        "\"mount_deg\":%.0f,\"status\":\"%s\"},",
        st.imu ? "true" : "false", st.imu_calibrated ? "true" : "false",
        imu_is_still() ? "true" : "false",
        roll * 180.0f / 3.14159265f, pitch * 180.0f / 3.14159265f,
        imu_mount_yaw_deg(), imu_status());
    PUT("\"map\":{\"w\":%d,\"h\":%d,\"res\":%.3f,\"ox\":%.2f,\"oy\":%.2f},",
        LIVE_MAP_W, LIVE_MAP_H, LIVE_MAP_RES, LIVE_MAP_OX, LIVE_MAP_OY);
    PUT("\"scan_mm\":[");
    for (int i = 0; i < LIVE_SCAN_BINS; i++) {
        PUT(i ? ",%u" : "%u", (unsigned)mm[i]);
    }
    PUT("]}");
#undef PUT
    return (int)(p - buf);
}

static esp_err_t h_root(httpd_req_t *req)
{
    cors(req);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start,
                           (ssize_t)(index_html_end - index_html_start));
}

static esp_err_t h_api(httpd_req_t *req)
{
    return send_text(req, "application/json",
        "{"
        "\"name\":\"Twist-auto\","
        "\"ip\":\"192.168.4.1\","
        "\"port\":80,"
        "\"ssid\":\"" TWIST_WIFI_SSID "\","
        "\"password\":\"" TWIST_WIFI_PASS "\","
        "\"how_to\":\"Phone: join Wi-Fi Twist-auto, open http://192.168.4.1/  Native app: same host, endpoints below.\","
        "\"frame\":{\"x\":\"robot forward after POST /api/imu/calibrate\",\"y\":\"robot left\","
        "\"yaw_deg\":\"counter-clockwise, 0 is the front arrow\"},"
        "\"endpoints\":["
          "{\"method\":\"GET\",\"path\":\"/\",\"use\":\"built-in live map page\"},"
          "{\"method\":\"GET\",\"path\":\"/api\",\"use\":\"this catalog\"},"
          "{\"method\":\"GET\",\"path\":\"/api/status\",\"use\":\"health, pose, imu, lidar, stream flags\"},"
          "{\"method\":\"GET\",\"path\":\"/api/live.json\",\"use\":\"poll ~5 Hz: pose + scan_mm[360] + map_version\"},"
          "{\"method\":\"GET\",\"path\":\"/ws/live\",\"use\":\"WebSocket. Send any text (e.g. g) to receive live.json. Optional cmds: calibrate, zero, reset, start, stop\"},"
          "{\"method\":\"GET\",\"path\":\"/api/map.bin\",\"use\":\"int8 log-odds, row-major y*w+x, length 40000. >0 occupied, <0 free, 0 unknown. Fetch when map_version changes\"},"
          "{\"method\":\"GET\",\"path\":\"/api/map.pgm\",\"use\":\"P5 PGM image of the live map\"},"
          "{\"method\":\"GET\",\"path\":\"/api/preview.bin\",\"use\":\"80x80 uint8 grayscale (20=wall, 220=free, 80=unknown)\"},"
          "{\"method\":\"GET\",\"path\":\"/api/scan.json\",\"use\":\"latest 360 ranges in metres\"},"
          "{\"method\":\"POST\",\"path\":\"/api/imu/calibrate\",\"use\":\"hold still: gyro bias + current heading becomes front arrow\"},"
          "{\"method\":\"POST\",\"path\":\"/api/imu/zero\",\"use\":\"current heading becomes front without re-bias\"},"
          "{\"method\":\"POST\",\"path\":\"/api/map/reset\",\"use\":\"clear occupancy\"},"
          "{\"method\":\"POST\",\"path\":\"/api/stream/start\",\"use\":\"spin LiDAR and update the map\"},"
          "{\"method\":\"POST\",\"path\":\"/api/stream/stop\",\"use\":\"stop LiDAR motor\"}"
        "]"
        "}");
}

static esp_err_t h_status(httpd_req_t *req)
{
    live_state_t st;
    live_map_get_state(&st);
    char buf[768];
    snprintf(buf, sizeof(buf),
             "{\"ok\":true,\"ip\":\"%s\",\"ssid\":\"%s\",\"lidar\":%s,\"imu\":%s,"
             "\"imu_calibrated\":%s,\"imu_status\":\"%s\",\"streaming\":%s,"
             "\"seq\":%lu,\"map_version\":%lu,\"yaw_deg\":%.2f,"
             "\"lidar_yaw_deg\":%.0f,\"imu_mount_deg\":%.0f}",
             s_ip, TWIST_WIFI_SSID,
             st.lidar ? "true" : "false", st.imu ? "true" : "false",
             st.imu_calibrated ? "true" : "false", imu_status(),
             st.streaming ? "true" : "false",
             (unsigned long)st.scan_seq, (unsigned long)st.map_version,
             st.yaw * 180.0f / 3.14159265f, lidar_yaw_deg(), imu_mount_yaw_deg());
    return send_text(req, "application/json", buf);
}

static esp_err_t h_live(httpd_req_t *req)
{
    if (s_json_lock) {
        xSemaphoreTake(s_json_lock, portMAX_DELAY);
    }
    int n = fill_live_json(s_json, sizeof(s_json));
    esp_err_t err = n < 0
                        ? send_text(req, "application/json", "{\"error\":\"live json overflow\"}")
                        : send_text(req, "application/json", s_json);
    if (s_json_lock) {
        xSemaphoreGive(s_json_lock);
    }
    return err;
}

static esp_err_t h_map_bin(httpd_req_t *req)
{
    cors(req);
    httpd_resp_set_type(req, "application/octet-stream");
    return httpd_resp_send(req, (const char *)live_map_cells(), LIVE_MAP_W * LIVE_MAP_H);
}

static esp_err_t h_map_pgm(httpd_req_t *req)
{
    const int8_t *c = live_map_cells();
    int hdr = snprintf((char *)s_pgm, sizeof(s_pgm), "P5\n%d %d\n255\n", LIVE_MAP_W, LIVE_MAP_H);
    uint8_t *pix = s_pgm + hdr;
    for (int i = 0; i < LIVE_MAP_W * LIVE_MAP_H; i++) {
        int8_t v = c[i];
        pix[i] = v > 0 ? 0 : (v < 0 ? 254 : 205);
    }
    cors(req);
    httpd_resp_set_type(req, "image/x-portable-graymap");
    return httpd_resp_send(req, (const char *)s_pgm, hdr + LIVE_MAP_W * LIVE_MAP_H);
}

static esp_err_t h_preview(httpd_req_t *req)
{
    uint8_t prev[80 * 80];
    live_map_preview(prev, 80, 80);
    cors(req);
    httpd_resp_set_type(req, "application/octet-stream");
    return httpd_resp_send(req, (const char *)prev, sizeof(prev));
}

static esp_err_t h_scan(httpd_req_t *req)
{
    uint16_t mm[LIVE_SCAN_BINS];
    live_map_copy_scan_mm(mm, LIVE_SCAN_BINS);
    if (s_json_lock) {
        xSemaphoreTake(s_json_lock, portMAX_DELAY);
    }
    char *p = s_json;
    char *end = s_json + sizeof(s_json);
    int n = snprintf(p, (size_t)(end - p), "{\"angle_min_deg\":0,\"angle_increment_deg\":1,\"ranges_m\":[");
    p += n;
    for (int i = 0; i < LIVE_SCAN_BINS; i++) {
        if (mm[i] == 0) {
            n = snprintf(p, (size_t)(end - p), i ? ",null" : "null");
        } else {
            n = snprintf(p, (size_t)(end - p), i ? ",%.3f" : "%.3f", mm[i] / 1000.0f);
        }
        p += n;
        if (p >= end - 8) {
            break;
        }
    }
    snprintf(p, (size_t)(end - p), "]}");
    esp_err_t err = send_text(req, "application/json", s_json);
    if (s_json_lock) {
        xSemaphoreGive(s_json_lock);
    }
    return err;
}

static esp_err_t h_ok(httpd_req_t *req, const char *msg)
{
    drain(req);
    char buf[192];
    snprintf(buf, sizeof(buf), "{\"ok\":true,\"msg\":\"%s\"}", msg);
    return send_text(req, "application/json", buf);
}

static esp_err_t h_calib(httpd_req_t *req)
{
    drain(req);
    esp_err_t err = imu_calibrate(8000);
    return send_text(req, "application/json",
                     err == ESP_OK ? "{\"ok\":true,\"msg\":\"front is now the arrow (yaw 0)\"}"
                                   : "{\"ok\":false,\"msg\":\"calibrate failed; hold still\"}");
}

static esp_err_t h_zero(httpd_req_t *req)
{
    imu_zero_yaw();
    return h_ok(req, "yaw zeroed; current heading is front");
}

static esp_err_t h_reset(httpd_req_t *req)
{
    live_map_reset();
    return h_ok(req, "map cleared");
}

static esp_err_t h_start(httpd_req_t *req)
{
    live_map_set_streaming(true);
    return h_ok(req, "stream on");
}

static esp_err_t h_stop(httpd_req_t *req)
{
    live_map_set_streaming(false);
    return h_ok(req, "stream off");
}

static esp_err_t h_options(httpd_req_t *req)
{
    cors(req);
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static void handle_ws_cmd(const char *msg)
{
    if (msg == NULL) {
        return;
    }
    if (strstr(msg, "calibrate")) {
        (void)imu_calibrate(8000);
    } else if (strstr(msg, "zero")) {
        imu_zero_yaw();
    } else if (strstr(msg, "reset")) {
        live_map_reset();
    } else if (strstr(msg, "start")) {
        live_map_set_streaming(true);
    } else if (strstr(msg, "stop")) {
        live_map_set_streaming(false);
    }
}

static esp_err_t h_ws(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        return ESP_OK;
    }
    httpd_ws_frame_t ws = {0};
    ws.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t err = httpd_ws_recv_frame(req, &ws, 0);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t *in = NULL;
    if (ws.len) {
        in = calloc(1, ws.len + 1);
        if (!in) {
            return ESP_ERR_NO_MEM;
        }
        ws.payload = in;
        err = httpd_ws_recv_frame(req, &ws, ws.len);
        if (err != ESP_OK) {
            free(in);
            return err;
        }
        handle_ws_cmd((const char *)in);
    }
    free(in);
    if (s_json_lock) {
        xSemaphoreTake(s_json_lock, portMAX_DELAY);
    }
    int n = fill_live_json(s_json, sizeof(s_json));
    esp_err_t send_err = ESP_ERR_NO_MEM;
    if (n >= 0) {
        httpd_ws_frame_t out = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)s_json,
            .len = (size_t)n,
        };
        send_err = httpd_ws_send_frame(req, &out);
    }
    if (s_json_lock) {
        xSemaphoreGive(s_json_lock);
    }
    return send_err;
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "phone joined " MACSTR, MAC2STR(e->mac));
        printf("App joined Wi-Fi. Open http://%s/\n", s_ip);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "phone left " MACSTR, MAC2STR(e->mac));
    }
}

static void register_uri(httpd_handle_t h, const char *uri, httpd_method_t m,
                         esp_err_t (*fn)(httpd_req_t *), bool ws)
{
    httpd_uri_t u = {
        .uri = uri,
        .method = m,
        .handler = fn,
        .is_websocket = ws,
    };
    if (httpd_register_uri_handler(h, &u) != ESP_OK) {
        ESP_LOGE(TAG, "uri %s failed", uri);
    }
}

static esp_err_t start_httpd(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = TWIST_HTTP_PORT;
    cfg.lru_purge_enable = true;
    cfg.max_uri_handlers = 20;
    cfg.stack_size = 8192;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd failed");
        return ESP_FAIL;
    }
    register_uri(s_httpd, "/", HTTP_GET, h_root, false);
    register_uri(s_httpd, "/api", HTTP_GET, h_api, false);
    register_uri(s_httpd, "/api/status", HTTP_GET, h_status, false);
    register_uri(s_httpd, "/api/live.json", HTTP_GET, h_live, false);
    register_uri(s_httpd, "/api/map.bin", HTTP_GET, h_map_bin, false);
    register_uri(s_httpd, "/api/map.pgm", HTTP_GET, h_map_pgm, false);
    register_uri(s_httpd, "/api/preview.bin", HTTP_GET, h_preview, false);
    register_uri(s_httpd, "/api/scan.json", HTTP_GET, h_scan, false);
    register_uri(s_httpd, "/api/imu/calibrate", HTTP_POST, h_calib, false);
    register_uri(s_httpd, "/api/imu/zero", HTTP_POST, h_zero, false);
    register_uri(s_httpd, "/api/map/reset", HTTP_POST, h_reset, false);
    register_uri(s_httpd, "/api/stream/start", HTTP_POST, h_start, false);
    register_uri(s_httpd, "/api/stream/stop", HTTP_POST, h_stop, false);
    register_uri(s_httpd, "/api/*", HTTP_OPTIONS, h_options, false);
    register_uri(s_httpd, "/ws/live", HTTP_GET, h_ws, true);
    ESP_LOGI(TAG, "HTTP on :%d", TWIST_HTTP_PORT);
    return ESP_OK;
}

esp_err_t wifi_app_start(void)
{
    s_json_lock = xSemaphoreCreateMutex();
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "netif %s", esp_err_to_name(err));
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop %s", esp_err_to_name(err));
        return err;
    }
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi init failed (%s). On P4 this needs the ESP32-C6 (ESP-Hosted).",
                 esp_err_to_name(err));
        printf("Wi-Fi not started (%s). LiDAR/IMU still work from the serial console.\n",
               esp_err_to_name(err));
        return err;
    }

    (void)esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event, NULL, NULL);

    wifi_config_t ap = {0};
    strncpy((char *)ap.ap.ssid, TWIST_WIFI_SSID, sizeof(ap.ap.ssid) - 1);
    strncpy((char *)ap.ap.password, TWIST_WIFI_PASS, sizeof(ap.ap.password) - 1);
    ap.ap.ssid_len = strlen(TWIST_WIFI_SSID);
    ap.ap.channel = TWIST_WIFI_CHANNEL;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.pmf_cfg.required = false;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_AP, &ap);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi config %s", esp_err_to_name(err));
        return err;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi start %s", esp_err_to_name(err));
        printf("Wi-Fi start failed (%s). Is the ESP32-C6 present and flashed with ESP-Hosted?\n",
               esp_err_to_name(err));
        return err;
    }

    s_up = true;
    if (start_httpd() != ESP_OK) {
        return ESP_FAIL;
    }
    wifi_app_print();
    return ESP_OK;
}

bool wifi_app_is_up(void)
{
    return s_up;
}

const char *wifi_app_ip(void)
{
    return s_ip;
}

void wifi_app_print(void)
{
    printf("Wi-Fi AP  SSID=%s  pass=%s  http://%s/\n", TWIST_WIFI_SSID, TWIST_WIFI_PASS, s_ip);
    printf("  GET  /api              endpoint catalog for the mobile app\n");
    printf("  GET  /api/live.json    pose + scan (poll 5 Hz)\n");
    printf("  GET  /ws/live          WebSocket live map (send g)\n");
    printf("  GET  /api/map.bin      occupancy int8 200x200\n");
    printf("  POST /api/imu/calibrate  hold still, front becomes the arrow\n");
}
