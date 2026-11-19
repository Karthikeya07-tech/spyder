#include "lidar.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.h"
#include "usb/vcp_cp210x.h"

static const char *TAG = "lidar";

#define RPLIDAR_SYNC     0xA5
#define CMD_STOP         0x25
#define CMD_RESET        0x40
#define CMD_SCAN         0x20
#define CMD_GET_INFO     0x50
#define CMD_GET_HEALTH   0x52
#define ANS_INFO         0x04
#define ANS_HEALTH       0x06

#define RX_BUF_SIZE      8192
#define SCAN_BINS        LIDAR_BINS

static cdc_acm_dev_hdl_t s_cdc;
static portMUX_TYPE s_rx_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_rx_sem;
static SemaphoreHandle_t s_ready_sem;
static uint8_t s_rx[RX_BUF_SIZE];
static size_t s_rx_len;
static volatile bool s_ready;
static float s_ranges[SCAN_BINS];
static int s_scan_n;
static SemaphoreHandle_t s_lock;
static float s_yaw_deg = LIDAR_YAW_DEG;
static volatile bool s_stream;
static volatile uint32_t s_seq;
static uint32_t s_baud = 115200;
static int s_arm_fails;

static const uint32_t s_bauds[] = {115200, 256000, 460800, 1000000};

static bool handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
    (void)arg;
    if (data == NULL || data_len == 0) {
        return true;
    }
    portENTER_CRITICAL(&s_rx_lock);
    size_t room = RX_BUF_SIZE - s_rx_len;
    size_t n = data_len < room ? data_len : room;
    memcpy(s_rx + s_rx_len, data, n);
    s_rx_len += n;
    portEXIT_CRITICAL(&s_rx_lock);
    if (s_rx_sem) {
        xSemaphoreGive(s_rx_sem);
    }
    return true;
}

static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC error %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "LiDAR USB adapter disconnected");
        s_ready = false;
        s_cdc = NULL;
        cdc_acm_host_close(event->data.cdc_hdl);
        break;
    default:
        break;
    }
}

static void rx_flush(void)
{
    portENTER_CRITICAL(&s_rx_lock);
    s_rx_len = 0;
    portEXIT_CRITICAL(&s_rx_lock);
    if (s_rx_sem) {
        xSemaphoreTake(s_rx_sem, 0);
    }
}

static bool read_byte(uint8_t *out, TickType_t deadline)
{
    while (s_ready && s_cdc) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) {
            return false;
        }
        portENTER_CRITICAL(&s_rx_lock);
        if (s_rx_len > 0) {
            *out = s_rx[0];
            memmove(s_rx, s_rx + 1, s_rx_len - 1);
            s_rx_len--;
            portEXIT_CRITICAL(&s_rx_lock);
            return true;
        }
        portEXIT_CRITICAL(&s_rx_lock);
        xSemaphoreTake(s_rx_sem, deadline - now);
    }
    return false;
}

static bool read_exact(uint8_t *dst, size_t len, uint32_t timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    for (size_t i = 0; i < len; i++) {
        if (!read_byte(&dst[i], deadline)) {
            return false;
        }
    }
    return true;
}

static bool read_descriptor(uint8_t desc[7], uint32_t timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    int matched = 0;
    uint8_t byte = 0;
    while (read_byte(&byte, deadline)) {
        if (matched == 0) {
            matched = (byte == 0xA5) ? 1 : 0;
        } else if (byte == 0x5A) {
            desc[0] = 0xA5;
            desc[1] = 0x5A;
            return read_exact(desc + 2, 5, 200);
        } else {
            matched = (byte == 0xA5) ? 1 : 0;
        }
    }
    return false;
}

static esp_err_t send_cmd(uint8_t cmd)
{
    uint8_t pkt[2] = {RPLIDAR_SYNC, cmd};
    return cdc_acm_host_data_tx_blocking(s_cdc, pkt, sizeof(pkt), 200);
}

static bool expect_response(uint8_t type, uint8_t length, uint8_t *payload, uint32_t timeout_ms)
{
    uint8_t desc[7];
    if (!read_descriptor(desc, timeout_ms)) {
        return false;
    }
    if (desc[2] != length || desc[3] != 0 || desc[4] != 0 || desc[5] != 0 || desc[6] != type) {
        ESP_LOGW(TAG, "bad descriptor %02X %02X %02X %02X %02X %02X %02X",
                 desc[0], desc[1], desc[2], desc[3], desc[4], desc[5], desc[6]);
        return false;
    }
    return read_exact(payload, length, 300);
}

static const char *model_name(uint8_t model)
{
    switch (model >> 4) {
    case 1: return "A1";
    case 2: return "A2";
    case 3: return "A3";
    case 4: return "C1/S";
    case 6: return "S2";
    case 7: return "S3";
    default: return "unknown";
    }
}

static const char *health_name(uint8_t status)
{
    switch (status) {
    case 0: return "Good";
    case 1: return "Warning";
    case 2: return "Error";
    default: return "unknown";
    }
}

static bool query_health(void)
{
    rx_flush();
    if (send_cmd(CMD_GET_HEALTH) != ESP_OK) {
        return false;
    }
    uint8_t payload[3];
    if (!expect_response(ANS_HEALTH, 3, payload, 700)) {
        return false;
    }
    uint16_t err = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
    printf("  health %s (%u), error_code %u\n", health_name(payload[0]), payload[0], err);
    ESP_LOGI(TAG, "health %s (%u), error_code %u", health_name(payload[0]), payload[0], err);
    return true;
}

static bool probe_baud(uint32_t baud, bool do_reset)
{
    cdc_acm_line_coding_t lc = {
        .dwDTERate = baud,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 8,
    };
    esp_err_t err = cdc_acm_host_line_coding_set(s_cdc, &lc);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "baud %" PRIu32 " rejected: %s", baud, esp_err_to_name(err));
        return false;
    }
    /* DTR true keeps the A1 adapter motor off during ping. */
    cdc_acm_host_set_control_line_state(s_cdc, true, true);
    vTaskDelay(pdMS_TO_TICKS(50));
    rx_flush();

    if (do_reset) {
        send_cmd(CMD_RESET);
        vTaskDelay(pdMS_TO_TICKS(1200));
        rx_flush();
    }

    send_cmd(CMD_STOP);
    vTaskDelay(pdMS_TO_TICKS(80));
    rx_flush();

    if (send_cmd(CMD_GET_INFO) != ESP_OK) {
        return false;
    }

    uint8_t info[20];
    if (!expect_response(ANS_INFO, 20, info, 800)) {
        return false;
    }

    printf("RPLIDAR %s (model 0x%02X) firmware %u.%u hardware %u  baud %" PRIu32 "\n",
           model_name(info[0]), info[0], info[2], info[1], info[3], baud);
    ESP_LOGI(TAG, "RPLIDAR %s firmware %u.%u hw %u baud %" PRIu32,
             model_name(info[0]), info[2], info[1], info[3], baud);
    query_health();
    s_baud = baud;
    return true;
}

esp_err_t lidar_ping(void)
{
    if (s_stream) {
        printf("LiDAR is streaming. Type: live stop   then   lping\n");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_cdc == NULL) {
        printf("LiDAR USB adapter not open. Plug CP2102 into USB-OTG, USB2AX stays on GPIO26/27.\n");
        return ESP_ERR_INVALID_STATE;
    }
    for (size_t i = 0; i < sizeof(s_bauds) / sizeof(s_bauds[0]); i++) {
        if (probe_baud(s_bauds[i], false)) {
            return ESP_OK;
        }
    }
    ESP_LOGW(TAG, "no GET_INFO yet, sending RESET");
    for (size_t i = 0; i < sizeof(s_bauds) / sizeof(s_bauds[0]); i++) {
        if (probe_baud(s_bauds[i], true)) {
            return ESP_OK;
        }
    }
    printf("LiDAR USB adapter open, but RPLIDAR did not answer GET_INFO (A5 50).\n");
    return ESP_ERR_TIMEOUT;
}

static bool parse_node(const uint8_t b[5], bool *new_rev, float *angle_deg, float *dist_m)
{
    unsigned s = b[0] & 1u;
    unsigned ns = (b[0] >> 1) & 1u;
    unsigned c = b[1] & 1u;
    if (s == ns || c != 1u) {
        return false;
    }
    *new_rev = s != 0;
    *angle_deg = ((b[1] >> 1) | (b[2] << 7)) / 64.0f;
    *dist_m = (float)(b[3] | (b[4] << 8)) / 4000.0f;
    return true;
}

static int angle_to_bin(float ang_cw)
{
    int bin = (int)floorf(-ang_cw + s_yaw_deg + 0.5f);
    bin %= SCAN_BINS;
    if (bin < 0) {
        bin += SCAN_BINS;
    }
    return bin;
}

static void publish_scan(const float *src, int valid)
{
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    memcpy(s_ranges, src, sizeof(s_ranges));
    s_scan_n = SCAN_BINS;
    s_seq++;
    uint32_t seq = s_seq;
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
    (void)valid;
    if (seq == 1 || (seq % 50) == 0) {
        ESP_LOGI(TAG, "live revolution %lu published", (unsigned long)seq);
    }
}

static void print_scan_summary(void)
{
    int filled = 0;
    float rmin = 99.0f, rmax = 0.0f;
    for (int i = 0; i < SCAN_BINS; i++) {
        if (isfinite(s_ranges[i])) {
            filled++;
            if (s_ranges[i] < rmin) {
                rmin = s_ranges[i];
            }
            if (s_ranges[i] > rmax) {
                rmax = s_ranges[i];
            }
        }
    }
    printf("LiDAR scan: %d / %d bins  min %.2f m  max %.2f m  (0=front)\n",
           filled, SCAN_BINS, filled ? rmin : 0.0f, filled ? rmax : 0.0f);
    printf("  front(0) %.2f  left(90) %.2f  back(180) %.2f  right(270) %.2f m\n",
           isfinite(s_ranges[0]) ? s_ranges[0] : -1.0f,
           isfinite(s_ranges[90]) ? s_ranges[90] : -1.0f,
           isfinite(s_ranges[180]) ? s_ranges[180] : -1.0f,
           isfinite(s_ranges[270]) ? s_ranges[270] : -1.0f);
}

esp_err_t lidar_scan(void)
{
    if (!s_ready || s_cdc == NULL) {
        printf("LiDAR not open. Plug CP2102 into USB-OTG.\n");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_stream) {
        if (s_seq == 0) {
            printf("Live stream on, waiting for the first revolution...\n");
            return ESP_ERR_INVALID_STATE;
        }
        print_scan_summary();
        return ESP_OK;
    }
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }

    /* A1 adapter: DTR false = motor on. */
    cdc_acm_host_set_control_line_state(s_cdc, false, true);
    vTaskDelay(pdMS_TO_TICKS(800));
    send_cmd(CMD_STOP);
    vTaskDelay(pdMS_TO_TICKS(50));
    rx_flush();

    for (int i = 0; i < SCAN_BINS; i++) {
        s_ranges[i] = INFINITY;
    }
    s_scan_n = 0;

    esp_err_t result = ESP_ERR_TIMEOUT;
    if (send_cmd(CMD_SCAN) != ESP_OK) {
        result = ESP_FAIL;
        goto stop;
    }
    uint8_t desc[7];
    if (!read_descriptor(desc, 1000)) {
        printf("LiDAR SCAN: no descriptor\n");
        goto stop;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2500);
    bool seen_start = false;
    int valid = 0;
    uint8_t node[5];
    while ((int32_t)(deadline - xTaskGetTickCount()) > 0) {
        if (!read_exact(node, 5, 80)) {
            continue;
        }
        bool new_rev = false;
        float ang = 0, dist = 0;
        if (!parse_node(node, &new_rev, &ang, &dist)) {
            continue;
        }
        if (new_rev) {
            if (seen_start && valid > 50) {
                break;
            }
            seen_start = true;
        }
        if (!seen_start) {
            continue;
        }
        int bin = angle_to_bin(ang);
        if (dist > 0.05f) {
            s_ranges[bin] = dist;
            valid++;
        }
    }

    int filled = 0;
    float rmin = 99.0f, rmax = 0.0f;
    for (int i = 0; i < SCAN_BINS; i++) {
        if (isfinite(s_ranges[i])) {
            filled++;
            if (s_ranges[i] < rmin) {
                rmin = s_ranges[i];
            }
            if (s_ranges[i] > rmax) {
                rmax = s_ranges[i];
            }
        }
    }
    s_scan_n = SCAN_BINS;
    if (filled > 20) {
        s_seq++;
        result = ESP_OK;
    } else {
        result = ESP_ERR_TIMEOUT;
    }
    printf("LiDAR scan: %d / %d bins  min %.2f m  max %.2f m  (0=front)\n",
           filled, SCAN_BINS, filled ? rmin : 0.0f, filled ? rmax : 0.0f);
    printf("  front(0) %.2f  left(90) %.2f  back(180) %.2f  right(270) %.2f m\n",
           isfinite(s_ranges[0]) ? s_ranges[0] : -1.0f,
           isfinite(s_ranges[90]) ? s_ranges[90] : -1.0f,
           isfinite(s_ranges[180]) ? s_ranges[180] : -1.0f,
           isfinite(s_ranges[270]) ? s_ranges[270] : -1.0f);

stop:
    send_cmd(CMD_STOP);
    vTaskDelay(pdMS_TO_TICKS(20));
    cdc_acm_host_set_control_line_state(s_cdc, true, true);
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
    return result;
}

int lidar_last_n(const float **ranges_m)
{
    if (ranges_m) {
        *ranges_m = s_ranges;
    }
    return s_scan_n;
}

int lidar_copy_ranges(float *dst, int n)
{
    if (dst == NULL || n <= 0) {
        return 0;
    }
    if (n > SCAN_BINS) {
        n = SCAN_BINS;
    }
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    memcpy(dst, s_ranges, (size_t)n * sizeof(float));
    int got = s_scan_n < n ? s_scan_n : n;
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
    return got;
}

uint32_t lidar_scan_seq(void)
{
    return s_seq;
}

static bool apply_baud(uint32_t baud)
{
    cdc_acm_line_coding_t lc = {
        .dwDTERate = baud,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 8,
    };
    return cdc_acm_host_line_coding_set(s_cdc, &lc) == ESP_OK;
}

static bool stream_resync(void)
{
    apply_baud(s_baud);
    cdc_acm_host_set_control_line_state(s_cdc, true, true);
    send_cmd(CMD_STOP);
    vTaskDelay(pdMS_TO_TICKS(80));
    send_cmd(CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(1200));
    rx_flush();
    for (size_t i = 0; i < sizeof(s_bauds) / sizeof(s_bauds[0]); i++) {
        if (probe_baud(s_bauds[i], false)) {
            return true;
        }
    }
    return probe_baud(s_baud, true);
}

static bool stream_arm_scan(void)
{
    if (s_cdc == NULL) {
        return false;
    }
    if (s_arm_fails > 0 && (s_arm_fails % 3) == 0) {
        ESP_LOGW(TAG, "live scan: RESET + GET_INFO, then SCAN again");
        if (!stream_resync()) {
            return false;
        }
    }

    apply_baud(s_baud);
    send_cmd(CMD_STOP);
    vTaskDelay(pdMS_TO_TICKS(80));
    rx_flush();

    /* A1 adapter: DTR false = motor on. Same order as robot/lidar_rplidar.py. */
    cdc_acm_host_set_control_line_state(s_cdc, false, true);
    vTaskDelay(pdMS_TO_TICKS(1200));
    rx_flush();

    if (send_cmd(CMD_SCAN) != ESP_OK) {
        ESP_LOGW(TAG, "SCAN command TX failed");
        return false;
    }
    uint8_t desc[7];
    if (!read_descriptor(desc, 2500)) {
        return false;
    }
    s_arm_fails = 0;
    ESP_LOGI(TAG, "live SCAN ok  desc %02X %02X  type 0x%02X", desc[0], desc[1], desc[6]);
    return true;
}

static void stream_task(void *arg)
{
    (void)arg;
    float bins[SCAN_BINS];
    while (1) {
        while (!s_stream || !s_ready || s_cdc == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        ESP_LOGI(TAG, "live scan start (0 deg = robot front, yaw offset %.0f, baud %" PRIu32 ")",
                 s_yaw_deg, s_baud);
        if (!stream_arm_scan()) {
            s_arm_fails++;
            if ((s_arm_fails % 5) == 1) {
                ESP_LOGW(TAG, "live SCAN descriptor missing (try %d)", s_arm_fails);
            }
            send_cmd(CMD_STOP);
            vTaskDelay(pdMS_TO_TICKS(400));
            continue;
        }
        for (int i = 0; i < SCAN_BINS; i++) {
            bins[i] = INFINITY;
        }
        bool seen_start = false;
        int valid = 0;
        uint8_t node[5];
        while (s_stream && s_ready && s_cdc) {
            if (!read_exact(node, 5, 80)) {
                continue;
            }
            bool new_rev = false;
            float ang = 0, dist = 0;
            if (!parse_node(node, &new_rev, &ang, &dist)) {
                continue;
            }
            if (new_rev) {
                if (seen_start && valid > 20) {
                    publish_scan(bins, valid);
                }
                seen_start = true;
                for (int i = 0; i < SCAN_BINS; i++) {
                    bins[i] = INFINITY;
                }
                valid = 0;
            }
            if (!seen_start) {
                continue;
            }
            if (dist > 0.05f) {
                bins[angle_to_bin(ang)] = dist;
                valid++;
            }
        }
        send_cmd(CMD_STOP);
        vTaskDelay(pdMS_TO_TICKS(20));
        if (s_cdc) {
            cdc_acm_host_set_control_line_state(s_cdc, true, true);
        }
        ESP_LOGI(TAG, "live scan stop");
    }
}

esp_err_t lidar_stream_start(void)
{
    if (!s_ready || s_cdc == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_stream = true;
    return ESP_OK;
}

void lidar_stream_stop(void)
{
    s_stream = false;
}

bool lidar_is_streaming(void)
{
    return s_stream && s_ready;
}

void lidar_set_yaw_deg(float deg)
{
    s_yaw_deg = deg;
}

float lidar_yaw_deg(void)
{
    return s_yaw_deg;
}

static const cdc_acm_host_device_config_t s_dev_config = {
    .connection_timeout_ms = 1500,
    .out_buffer_size = 512,
    .in_buffer_size = 2048,
    .event_cb = handle_event,
    .data_cb = handle_rx,
    .user_arg = NULL,
};

static esp_err_t open_lidar(void)
{
    cdc_acm_dev_hdl_t cdc = NULL;
    ESP_LOGI(TAG, "Opening CP210x (RPLIDAR adapter)");
    esp_err_t err = cp210x_vcp_open(CP210X_PID_AUTO, 0, &s_dev_config, &cdc);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Opening CH34x");
        err = ch34x_vcp_open(CH34X_PID_AUTO, 0, &s_dev_config, &cdc);
    }
    if (err != ESP_OK) {
        return err;
    }
    s_cdc = cdc;
    ESP_LOGI(TAG, "LiDAR USB serial open");
    lidar_ping();
    s_ready = true;
    if (s_ready_sem) {
        xSemaphoreGive(s_ready_sem);
    }
    return ESP_OK;
}

static void lidar_open_task(void *arg)
{
    (void)arg;
    while (1) {
        if (s_ready) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        if (open_lidar() != ESP_OK) {
            ESP_LOGW(TAG, "LiDAR not found on USB (CP2102/CH34x). USB2AX on GPIO26/27 is separate.");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

esp_err_t lidar_start(void)
{
    s_rx_sem = xSemaphoreCreateBinary();
    s_ready_sem = xSemaphoreCreateBinary();
    s_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_rx_sem && s_ready_sem && s_lock, ESP_ERR_NO_MEM, TAG, "sem");
    ESP_LOGI(TAG, "RPLIDAR host: plug CP2102 into USB-OTG HS; servos stay on GPIO26/27");
    BaseType_t ok = xTaskCreate(lidar_open_task, "lidar_open", 4096, NULL, 5, NULL);
    ESP_RETURN_ON_FALSE(ok == pdTRUE, ESP_ERR_NO_MEM, TAG, "task");
    ok = xTaskCreate(stream_task, "lidar_live", 4096, NULL, 12, NULL);
    ESP_RETURN_ON_FALSE(ok == pdTRUE, ESP_ERR_NO_MEM, TAG, "stream task");
    return ESP_OK;
}

esp_err_t lidar_wait_ready(uint32_t timeout_ms)
{
    if (s_ready) {
        return ESP_OK;
    }
    if (xSemaphoreTake(s_ready_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return s_ready ? ESP_OK : ESP_ERR_TIMEOUT;
    }
    return ESP_ERR_TIMEOUT;
}

bool lidar_is_ready(void)
{
    return s_ready && s_cdc != NULL;
}
