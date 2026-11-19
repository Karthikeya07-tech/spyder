#include "dxl_p1.h"
#include "u2d2.h"

#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "dxl";

#define DXL_PKT_MAX      64
#define DXL_PING_MS      25
#define DXL_XFER_MS      40

static uint16_t s_model[DXL_MAX_ID + 1];
static SemaphoreHandle_t s_lock;

static void lock_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
    }
}

static uint8_t dxl_checksum(const uint8_t *data, size_t n)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < n; i++) {
        sum += data[i];
    }
    return (uint8_t)~sum;
}

static size_t dxl_build(uint8_t *out, uint8_t id, uint8_t inst,
                        const uint8_t *params, size_t nparams)
{
    out[0] = 0xFF;
    out[1] = 0xFF;
    out[2] = id;
    out[3] = (uint8_t)(nparams + 2);
    out[4] = inst;
    if (nparams && params) {
        memcpy(&out[5], params, nparams);
    }
    out[5 + nparams] = dxl_checksum(&out[2], nparams + 3);
    return nparams + 6;
}

static bool dxl_packet_ready(const uint8_t *buf, size_t len, size_t *start, size_t *pkt_len)
{
    for (size_t i = 0; i + 4 <= len; i++) {
        if (buf[i] == 0xFF && buf[i + 1] == 0xFF && buf[i + 2] != 0xFF) {
            size_t total = 4u + buf[i + 3];
            if (total < 6 || total > DXL_PKT_MAX) {
                continue;
            }
            if (i + total <= len) {
                *start = i;
                *pkt_len = total;
                return true;
            }
            return false;
        }
    }
    return false;
}

static esp_err_t dxl_xfer(uint8_t id, uint8_t inst, const uint8_t *params, size_t nparams,
                          uint8_t *status, size_t *status_len, uint32_t timeout_ms)
{
    uint8_t tx[DXL_PKT_MAX];
    uint8_t rx[DXL_PKT_MAX];
    size_t tx_len = dxl_build(tx, id, inst, params, nparams);

    lock_init();
    xSemaphoreTake(s_lock, portMAX_DELAY);

    u2d2_rx_flush();
    esp_err_t err = u2d2_write(tx, tx_len);
    if (err != ESP_OK) {
        xSemaphoreGive(s_lock);
        return err;
    }

    if (id == DXL_BROADCAST_ID) {
        xSemaphoreGive(s_lock);
        if (status_len) {
            *status_len = 0;
        }
        return ESP_OK;
    }

    TickType_t start_tick = xTaskGetTickCount();
    TickType_t wait = pdMS_TO_TICKS(timeout_ms);
    size_t pkt_start = 0, pkt_len = 0;
    bool got = false;

    while ((xTaskGetTickCount() - start_tick) < wait) {
        uint32_t left_ms = timeout_ms - (uint32_t)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS);
        if (left_ms < 2) {
            left_ms = 2;
        }
        size_t n = u2d2_read(rx, sizeof(rx), left_ms);
        if (n > 0 && dxl_packet_ready(rx, n, &pkt_start, &pkt_len)) {
            got = true;
            break;
        }
    }

    xSemaphoreGive(s_lock);

    if (!got) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t chk = dxl_checksum(&rx[pkt_start + 2], pkt_len - 3);
    if (chk != rx[pkt_start + pkt_len - 1]) {
        ESP_LOGW(TAG, "bad checksum from id %u", id);
        return ESP_ERR_INVALID_RESPONSE;
    };
    if (rx[pkt_start + 4] != 0) {
        ESP_LOGW(TAG, "id %u error byte 0x%02X", id, rx[pkt_start + 4]);
    }
    if (status && status_len) {
        size_t copy = pkt_len < *status_len ? pkt_len : *status_len;
        memcpy(status, &rx[pkt_start], copy);
        *status_len = copy;
    }
    return ESP_OK;
}

bool dxl_is_mx(uint16_t model)
{
    switch (model) {
    case 29:   /* MX-28 */
    case 310:  /* MX-64 */
    case 320:  /* MX-106 */
    case 360:  /* MX-12W */
        return true;
    default:
        return false;
    }
}

const char *dxl_model_name(uint16_t model)
{
    switch (model) {
    case 12:  return "AX-12";
    case 18:  return "AX-18";
    case 24:  return "RX-24F";
    case 28:  return "RX-28";
    case 29:  return "MX-28";
    case 64:  return "RX-64";
    case 107: return "EX-106+";
    case 300: return "AX-12W";
    case 310: return "MX-64";
    case 320: return "MX-106";
    case 360: return "MX-12W";
    default:  return "unknown";
    }
}

uint16_t dxl_cached_model(uint8_t id)
{
    if (id > DXL_MAX_ID) {
        return 0;
    }
    return s_model[id];
}

static uint16_t angle_to_pos(uint16_t model, float deg)
{
    if (dxl_is_mx(model)) {
        if (deg < 0.0f) {
            deg = 0.0f;
        }
        if (deg > 360.0f) {
            deg = 360.0f;
        }
        return (uint16_t)(deg * 4095.0f / 360.0f + 0.5f);
    }
    if (deg < 0.0f) {
        deg = 0.0f;
    }
    if (deg > 300.0f) {
        deg = 300.0f;
    }
    return (uint16_t)(deg * 1023.0f / 300.0f + 0.5f);
}

static float pos_to_angle(uint16_t model, uint16_t pos)
{
    if (dxl_is_mx(model)) {
        return (float)pos * 360.0f / 4095.0f;
    }
    return (float)pos * 300.0f / 1023.0f;
}

esp_err_t dxl_ping(uint8_t id)
{
    uint8_t st[16];
    size_t n = sizeof(st);
    return dxl_xfer(id, DXL_INSTR_PING, NULL, 0, st, &n, DXL_PING_MS);
}

esp_err_t dxl_read_u16(uint8_t id, uint8_t addr, uint16_t *value)
{
    uint8_t params[2] = {addr, 2};
    uint8_t st[16];
    size_t n = sizeof(st);
    ESP_RETURN_ON_ERROR(dxl_xfer(id, DXL_INSTR_READ, params, 2, st, &n, DXL_XFER_MS), TAG, "read");
    if (n < 8) {
        return ESP_ERR_INVALID_SIZE;
    }
    *value = (uint16_t)st[5] | ((uint16_t)st[6] << 8);
    return ESP_OK;
}

esp_err_t dxl_write_u8(uint8_t id, uint8_t addr, uint8_t value)
{
    uint8_t params[2] = {addr, value};
    uint8_t st[16];
    size_t n = sizeof(st);
    return dxl_xfer(id, DXL_INSTR_WRITE, params, 2, st, &n, DXL_XFER_MS);
}

esp_err_t dxl_write_u16(uint8_t id, uint8_t addr, uint16_t value)
{
    uint8_t params[3] = {addr, (uint8_t)(value & 0xFF), (uint8_t)(value >> 8)};
    uint8_t st[16];
    size_t n = sizeof(st);
    return dxl_xfer(id, DXL_INSTR_WRITE, params, 3, st, &n, DXL_XFER_MS);
}

esp_err_t dxl_sync_write_goal(const uint8_t *ids, const uint16_t *pos, size_t n)
{
    ESP_RETURN_ON_FALSE(ids && pos && n > 0 && n <= 16, ESP_ERR_INVALID_ARG, TAG, "sync");
    uint8_t tx[8 + 3 * 16];
    size_t nparams = 2 + 3 * n;
    tx[0] = 0xFF;
    tx[1] = 0xFF;
    tx[2] = DXL_BROADCAST_ID;
    tx[3] = (uint8_t)(nparams + 2);
    tx[4] = DXL_INSTR_SYNC_WRITE;
    tx[5] = DXL_ADDR_GOAL_POS;
    tx[6] = 2;
    size_t o = 7;
    for (size_t i = 0; i < n; i++) {
        tx[o++] = ids[i];
        tx[o++] = (uint8_t)(pos[i] & 0xFF);
        tx[o++] = (uint8_t)(pos[i] >> 8);
    }
    tx[o] = dxl_checksum(&tx[2], o - 2);
    o++;

    lock_init();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    u2d2_rx_flush();
    esp_err_t err = u2d2_write(tx, o);
    xSemaphoreGive(s_lock);
    return err;
}

int dxl_scan(uint8_t max_id)
{
    if (max_id > DXL_MAX_ID) {
        max_id = DXL_MAX_ID;
    }
    int found = 0;
    memset(s_model, 0, sizeof(s_model));
    printf("Scanning Dynamixel Protocol 1 @ %d baud (id 0-%u)...\n", U2D2_BAUDRATE, max_id);

    for (uint8_t id = 0; id <= max_id; id++) {
        if (id == 253) {
            continue; /* USB2AX answers as Dynamixel 0xFD; not a servo */
        }
        if (dxl_ping(id) != ESP_OK) {
            continue;
        }
        uint16_t model = 0;
        uint16_t pos = 0;
        dxl_read_u16(id, DXL_ADDR_MODEL, &model);
        dxl_read_u16(id, DXL_ADDR_PRESENT_POS, &pos);
        s_model[id] = model;
        found++;
        printf("  id %3u  model %-8s (%u)  pos %4u  (%.1f deg)\n",
               id, dxl_model_name(model), model, pos, pos_to_angle(model, pos));
    }
    printf("Found %d motor(s)\n", found);
    return found;
}

static esp_err_t ensure_model(uint8_t id)
{
    if (id > DXL_MAX_ID) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_model[id] != 0) {
        return ESP_OK;
    }
    uint16_t model = 0;
    ESP_RETURN_ON_ERROR(dxl_read_u16(id, DXL_ADDR_MODEL, &model), TAG, "model");
    s_model[id] = model;
    return ESP_OK;
}

esp_err_t dxl_move_deg(uint8_t id, float angle_deg)
{
    ESP_RETURN_ON_ERROR(ensure_model(id), TAG, "ensure_model");
    uint16_t model = s_model[id];
    uint16_t pos = angle_to_pos(model, angle_deg);
    ESP_RETURN_ON_ERROR(dxl_write_u8(id, DXL_ADDR_TORQUE, 1), TAG, "torque");
    ESP_RETURN_ON_ERROR(dxl_write_u16(id, DXL_ADDR_GOAL_POS, pos), TAG, "goal");
    printf("id %u -> %.1f deg  (raw %u, %s)\n",
           id, angle_deg, pos, dxl_is_mx(model) ? "MX 0-360/4095" : "AX 0-300/1023");
    return ESP_OK;
}

esp_err_t dxl_read_pos_deg(uint8_t id, float *angle_deg, uint16_t *raw)
{
    ESP_RETURN_ON_ERROR(ensure_model(id), TAG, "ensure_model");
    uint16_t pos = 0;
    ESP_RETURN_ON_ERROR(dxl_read_u16(id, DXL_ADDR_PRESENT_POS, &pos), TAG, "pos");
    if (raw) {
        *raw = pos;
    }
    if (angle_deg) {
        *angle_deg = pos_to_angle(s_model[id], pos);
    }
    return ESP_OK;
}
