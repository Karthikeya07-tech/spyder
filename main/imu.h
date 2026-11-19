#pragma once

#include <stdbool.h>
#include "esp_err.h"

#define IMU_SDA_GPIO         7
#define IMU_SCL_GPIO         8
/* Board X arrow faces the robot's back on this machine (same as the Pi). */
#define IMU_MOUNT_YAW_DEG    180.0f

esp_err_t imu_start(void);
bool imu_is_ready(void);
bool imu_is_calibrated(void);
void imu_bus_scan(void);
esp_err_t imu_read(void);

/* Keep the robot still. Gyro bias + level, then current heading becomes yaw 0 (front). */
esp_err_t imu_calibrate(uint32_t timeout_ms);
/* Current heading becomes the map front / arrow without re-measuring bias. */
void imu_zero_yaw(void);

/* Robot frame: x forward, y left, z up. Angles in radians, yaw CCW, 0 = front. */
void imu_get_rpy(float *roll, float *pitch, float *yaw);
float imu_yaw(void);
float imu_yaw_deg(void);
float imu_gz(void);          /* yaw rate, rad/s */
bool imu_is_still(void);
const char *imu_status(void);
void imu_set_mount_yaw_deg(float deg);
float imu_mount_yaw_deg(void);
