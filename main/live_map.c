#include "live_map.h"

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "imu.h"
#include "lidar.h"

static const char *TAG = "live_map";

#define L_FREE   (-1)
#define L_OCC    (3)
#define L_MAX    32
#define MIN_R    0.15f
#define MAX_R    8.0f

static int8_t s_cells[LIVE_MAP_W * LIVE_MAP_H];
static SemaphoreHandle_t s_lock;
static uint32_t s_map_version;
static uint32_t s_last_seq;
static volatile bool s_want_stream = true;

static inline int8_t clip_add(int8_t v, int d)
{
    int r = (int)v + d;
    if (r < -L_MAX) {
        return (int8_t)-L_MAX;
    }
    if (r > L_MAX) {
        return (int8_t)L_MAX;
    }
    return (int8_t)r;
}

static inline bool inside(int x, int y)
{
    return x >= 0 && y >= 0 && x < LIVE_MAP_W && y < LIVE_MAP_H;
}

static inline void add_cell(int x, int y, int d)
{
    if (!inside(x, y)) {
        return;
    }
    int i = y * LIVE_MAP_W + x;
    s_cells[i] = clip_add(s_cells[i], d);
}

static void world_to_cell(float x, float y, int *ix, int *iy)
{
    *ix = (int)floorf((x - LIVE_MAP_OX) / LIVE_MAP_RES);
    *iy = (int)floorf((y - LIVE_MAP_OY) / LIVE_MAP_RES);
}

static void ray(int x0, int y0, int x1, int y1, bool hit)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (x != x1 || y != y1) {
        int e2 = 2 * err;
        int nx = x, ny = y;
        if (e2 >= dy) {
            err += dy;
            nx += sx;
        }
        if (e2 <= dx) {
            err += dx;
            ny += sy;
        }
        if (nx == x1 && ny == y1) {
            break;
        }
        add_cell(x, y, L_FREE);
        x = nx;
        y = ny;
        if (!inside(x, y)) {
            return;
        }
    }
    if (hit) {
        add_cell(x1, y1, L_OCC);
    } else {
        add_cell(x1, y1, L_FREE);
    }
}

static void integrate_scan(const float *r, int n, float yaw)
{
    int x0, y0;
    world_to_cell(0.0f, 0.0f, &x0, &y0);
    float cy = cosf(yaw), sy = sinf(yaw);
    for (int i = 0; i < n; i++) {
        float d = r[i];
        bool hit = d > MIN_R && d < MAX_R && isfinite(d);
        float use = hit ? d : MAX_R;
        float a = (float)i * (float)M_PI / 180.0f;
        float ca = cosf(a), sa = sinf(a);
        /* robot frame (x forward, y left) rotated by yaw into world. */
        float wx = (ca * cy - sa * sy) * use;
        float wy = (ca * sy + sa * cy) * use;
        int x1, y1;
        world_to_cell(wx, wy, &x1, &y1);
        ray(x0, y0, x1, y1, hit);
    }
}

static void mapper_task(void *arg)
{
    (void)arg;
    float ranges[LIDAR_BINS];
    while (1) {
        if (s_want_stream && lidar_is_ready() && !lidar_is_streaming()) {
            if (lidar_stream_start() == ESP_OK) {
                ESP_LOGI(TAG, "live LiDAR stream on");
            }
        }
        uint32_t seq = lidar_scan_seq();
        if (seq != 0 && seq != s_last_seq) {
            s_last_seq = seq;
            int n = lidar_copy_ranges(ranges, LIDAR_BINS);
            if (n > 0) {
                float yaw = imu_is_ready() ? imu_yaw() : 0.0f;
                if (s_lock) {
                    xSemaphoreTake(s_lock, portMAX_DELAY);
                }
                integrate_scan(ranges, n, yaw);
                s_map_version++;
                if (s_lock) {
                    xSemaphoreGive(s_lock);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t live_map_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    memset(s_cells, 0, sizeof(s_cells));
    BaseType_t ok = xTaskCreate(mapper_task, "live_map", 4096, NULL, 5, NULL);
    if (ok != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "live occupancy %dx%d  %.2f m  origin (%.1f,%.1f)",
             LIVE_MAP_W, LIVE_MAP_H, LIVE_MAP_RES, LIVE_MAP_OX, LIVE_MAP_OY);
    return ESP_OK;
}

void live_map_reset(void)
{
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    memset(s_cells, 0, sizeof(s_cells));
    s_map_version++;
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
    printf("Live map cleared.\n");
}

void live_map_set_streaming(bool on)
{
    s_want_stream = on;
    if (!on) {
        lidar_stream_stop();
    }
}

bool live_map_streaming(void)
{
    return lidar_is_streaming();
}

void live_map_get_state(live_state_t *out)
{
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->yaw = imu_is_ready() ? imu_yaw() : 0.0f;
    out->scan_seq = lidar_scan_seq();
    out->streaming = lidar_is_streaming();
    out->lidar = lidar_is_ready();
    out->imu = imu_is_ready();
    out->imu_calibrated = imu_is_calibrated();
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    out->map_version = s_map_version;
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}

const int8_t *live_map_cells(void)
{
    return s_cells;
}

int live_map_copy_scan_mm(uint16_t *mm, int n)
{
    if (n > LIDAR_BINS) {
        n = LIDAR_BINS;
    }
    float r[LIDAR_BINS];
    int got = lidar_copy_ranges(r, LIDAR_BINS);
    int use = got < n ? got : n;
    for (int i = 0; i < use; i++) {
        if (isfinite(r[i]) && r[i] > MIN_R && r[i] < 40.0f) {
            float v = r[i] * 1000.0f;
            if (v > 65535.0f) {
                v = 65535.0f;
            }
            mm[i] = (uint16_t)(v + 0.5f);
        } else {
            mm[i] = 0;
        }
    }
    for (int i = use; i < n; i++) {
        mm[i] = 0;
    }
    return use;
}

void live_map_preview(uint8_t *dst, int pw, int ph)
{
    if (pw <= 0 || ph <= 0) {
        return;
    }
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    for (int py = 0; py < ph; py++) {
        int y0 = py * LIVE_MAP_H / ph;
        for (int px = 0; px < pw; px++) {
            int x0 = px * LIVE_MAP_W / pw;
            int8_t v = s_cells[y0 * LIVE_MAP_W + x0];
            uint8_t p = 80;
            if (v > 0) {
                p = 20;
            } else if (v < 0) {
                p = 220;
            }
            dst[py * pw + px] = p;
        }
    }
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}

void live_map_print(void)
{
    int occ = 0, free_n = 0, unk = 0;
    for (int i = 0; i < LIVE_MAP_W * LIVE_MAP_H; i++) {
        if (s_cells[i] > 0) {
            occ++;
        } else if (s_cells[i] < 0) {
            free_n++;
        } else {
            unk++;
        }
    }
    live_state_t st;
    live_map_get_state(&st);
    printf("Live map %dx%d  res %.3f m  origin (%.2f, %.2f)  version %lu\n",
           LIVE_MAP_W, LIVE_MAP_H, LIVE_MAP_RES, LIVE_MAP_OX, LIVE_MAP_OY,
           (unsigned long)st.map_version);
    printf("  occ %d  free %d  unk %d  yaw %.1f deg  stream %s  scans %lu\n",
           occ, free_n, unk, st.yaw * 180.0f / (float)M_PI,
           st.streaming ? "on" : "off", (unsigned long)st.scan_seq);

    const int step_x = LIVE_MAP_W / 40;
    const int step_y = LIVE_MAP_H / 20;
    printf("ASCII (#=wall .=free, arrow is front):\n");
    int rx, ry;
    world_to_cell(0.0f, 0.0f, &rx, &ry);
    for (int gy = LIVE_MAP_H - step_y; gy >= 0; gy -= step_y) {
        printf("  ");
        for (int gx = 0; gx < LIVE_MAP_W; gx += step_x) {
            if (abs(gx - rx) < step_x && abs(gy - ry) < step_y) {
                putchar('^');
                continue;
            }
            int8_t v = s_cells[gy * LIVE_MAP_W + gx];
            putchar(v > 0 ? '#' : (v < 0 ? '.' : ' '));
        }
        putchar('\n');
    }
}
