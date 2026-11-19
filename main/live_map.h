#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define LIVE_MAP_W      200
#define LIVE_MAP_H      200
#define LIVE_MAP_RES    0.05f
#define LIVE_MAP_OX     (-5.0f)
#define LIVE_MAP_OY     (-5.0f)
#define LIVE_SCAN_BINS  360

typedef struct {
    float x;
    float y;
    float yaw;          /* rad, 0 = front after IMU calibrate */
    uint32_t scan_seq;
    uint32_t map_version;
    bool streaming;
    bool lidar;
    bool imu;
    bool imu_calibrated;
} live_state_t;

esp_err_t live_map_start(void);
void live_map_reset(void);
void live_map_set_streaming(bool on);
bool live_map_streaming(void);

void live_map_get_state(live_state_t *out);
/* int8 log-odds, row-major, y * W + x. Length W*H. */
const int8_t *live_map_cells(void);
int live_map_copy_scan_mm(uint16_t *mm, int n);   /* 0 = no return */
void live_map_preview(uint8_t *dst, int pw, int ph);  /* 0 unk, 128 free, 255 occ */

void live_map_print(void);
