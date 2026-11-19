#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define LIDAR_BINS           360
/* A1 0° faces the robot's back on this machine (same as the Pi). */
#define LIDAR_YAW_DEG        180.0f

esp_err_t lidar_start(void);
esp_err_t lidar_wait_ready(uint32_t timeout_ms);
bool lidar_is_ready(void);
esp_err_t lidar_ping(void);
esp_err_t lidar_scan(void);

int lidar_last_n(const float **ranges_m);
int lidar_copy_ranges(float *dst, int n);
uint32_t lidar_scan_seq(void);

esp_err_t lidar_stream_start(void);
void lidar_stream_stop(void);
bool lidar_is_streaming(void);

void lidar_set_yaw_deg(float deg);
float lidar_yaw_deg(void);
