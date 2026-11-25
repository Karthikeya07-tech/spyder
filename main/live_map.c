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
#include "actions.h"
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
static volatile bool s_want_stream;
static float s_x;
static float s_y;
static int s_match_score;
static bool s_localized;

/* A one-cell step must beat "stay here" by this many beam hits. */
#define MATCH_GAIN   4
/* Below this, the grid does not yet confirm the pose. */
#define MATCH_LOCK   12

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

static void beam_world(float x, float y, float yaw, int deg, float range, float *wx, float *wy)
{
    float a = (float)deg * (float)M_PI / 180.0f;
    float ca = cosf(a), sa = sinf(a);
    float cy = cosf(yaw), sy = sinf(yaw);
    /* robot frame (x forward, y left) rotated by yaw into world. */
    *wx = x + (ca * cy - sa * sy) * range;
    *wy = y + (ca * sy + sa * cy) * range;
}

static int score_at(const float *r, int n, float x, float y, float yaw)
{
    int hits = 0;
    for (int i = 0; i < n; i += 2) {
        float d = r[i];
        if (!(d > MIN_R && d < MAX_R) || !isfinite(d)) {
            continue;
        }
        float wx, wy;
        beam_world(x, y, yaw, i, d, &wx, &wy);
        int ix, iy;
        world_to_cell(wx, wy, &ix, &iy);
        if (!inside(ix, iy)) {
            continue;
        }
        if (s_cells[iy * LIVE_MAP_W + ix] > 0) {
            hits++;
        }
    }
    return hits;
}

/* Heading is always the calibrated IMU yaw. Translation moves at most one
 * cell, and only while a walk clip is playing and the current pose already
 * matches the map. A still robot does not slide. */
static void update_pose(const float *r, int n, float yaw)
{
    if (!imu_is_calibrated()) {
        s_localized = false;
        s_match_score = 0;
        return;
    }

    int stay = score_at(r, n, s_x, s_y, yaw);
    s_match_score = stay;
    s_localized = stay >= MATCH_LOCK;
    if (!s_localized || !action_is_walking()) {
        return;
    }

    int best = stay;
    int step_x = 0;
    int step_y = 0;
    for (int cy = -1; cy <= 1; cy++) {
        for (int cx = -1; cx <= 1; cx++) {
            if (cx == 0 && cy == 0) {
                continue;
            }
            float x = s_x + (float)cx * LIVE_MAP_RES;
            float y = s_y + (float)cy * LIVE_MAP_RES;
            int s = score_at(r, n, x, y, yaw);
            if (s > best) {
                best = s;
                step_x = cx;
                step_y = cy;
            }
        }
    }

    if (best < stay + MATCH_GAIN) {
        return;
    }
    s_x += (float)step_x * LIVE_MAP_RES;
    s_y += (float)step_y * LIVE_MAP_RES;
    s_match_score = best;
    ESP_LOGI(TAG, "pose %.2f %.2f  score %d (stay %d)", s_x, s_y, best, stay);
}

static void integrate_scan(const float *r, int n, float x, float y, float yaw)
{
    int x0, y0;
    world_to_cell(x, y, &x0, &y0);
    for (int i = 0; i < n; i++) {
        float d = r[i];
        bool hit = d > MIN_R && d < MAX_R && isfinite(d);
        float use = hit ? d : MAX_R;
        float wx, wy;
        beam_world(x, y, yaw, i, use, &wx, &wy);
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
                update_pose(ranges, n, yaw);
                integrate_scan(ranges, n, s_x, s_y, yaw);
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
    BaseType_t ok = xTaskCreate(mapper_task, "live_map", 8192, NULL, 5, NULL);
    if (ok != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "live occupancy %dx%d  %.2f m  origin (%.1f,%.1f)  (scan waits for app)",
             LIVE_MAP_W, LIVE_MAP_H, LIVE_MAP_RES, LIVE_MAP_OX, LIVE_MAP_OY);
    return ESP_OK;
}

void live_map_reset(void)
{
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    memset(s_cells, 0, sizeof(s_cells));
    s_x = 0.0f;
    s_y = 0.0f;
    s_match_score = 0;
    s_localized = false;
    s_map_version++;
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
    printf("Live map cleared. Pose reset to the origin.\n");
}

void live_map_set_streaming(bool on)
{
    s_want_stream = on;
    if (on) {
        if (lidar_is_ready()) {
            (void)lidar_stream_start();
            ESP_LOGI(TAG, "SCAN requested: motor on, live frames to the app");
        } else {
            ESP_LOGW(TAG, "SCAN requested but LiDAR is not open yet");
        }
    } else {
        lidar_stream_stop();
        ESP_LOGI(TAG, "SCAN stop: motor off");
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
    out->x = s_x;
    out->y = s_y;
    out->localized = s_localized;
    out->match_score = s_match_score;
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
    printf("  occ %d  free %d  unk %d  pose (%.2f, %.2f) yaw %.1f deg  %s  score %d  stream %s  scans %lu\n",
           occ, free_n, unk, st.x, st.y, st.yaw * 180.0f / (float)M_PI,
           st.localized ? "locked" : "not locked", st.match_score,
           st.streaming ? "on" : "off", (unsigned long)st.scan_seq);

    const int step_x = LIVE_MAP_W / 40;
    const int step_y = LIVE_MAP_H / 20;
    printf("ASCII (#=wall .=free, arrow is front):\n");
    int rx, ry;
    world_to_cell(st.x, st.y, &rx, &ry);
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
