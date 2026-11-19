#include "imu.h"

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "imu";

#define MPU_ADDR_A      0x68
#define MPU_ADDR_B      0x69
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_XOUT  0x3B
#define REG_PWR_MGMT_1  0x6B
#define REG_WHO_AM_I    0x75

#define ACCEL_LSB       8192.0f   /* ±4 g */
#define GYRO_LSB        65.5f     /* ±500 dps */
#define IMU_RATE_HZ     100.0f
#define STILL_DPS       3.0f

enum {
    IMU_CMD_NONE = 0,
    IMU_CMD_CALIB,
    IMU_CMD_ZERO,
};

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static uint8_t s_addr;
static uint8_t s_who;
static bool s_ready;
static bool s_calibrated;
static volatile int s_cmd;
static SemaphoreHandle_t s_done;
static SemaphoreHandle_t s_lock;

static float s_mount = IMU_MOUNT_YAW_DEG * (float)M_PI / 180.0f;
static float s_mount_deg = IMU_MOUNT_YAW_DEG;
static float s_R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
static float s_bias[3];
static float s_roll, s_pitch, s_yaw, s_gz;
static bool s_fusion_on;
static bool s_still;
static char s_status[96] = "starting";

static const char *chip_name(uint8_t who)
{
    switch (who) {
    case 0x68: return "MPU-6050";
    case 0x70: return "MPU-6500";
    case 0x71: return "MPU-9250";
    case 0x73: return "MPU-9255";
    default:   return "MPU-compatible";
    }
}

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, 100);
}

static esp_err_t rd(uint8_t reg, uint8_t *data, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, n, 100);
}

static float wrap_pi(float a)
{
    while (a > (float)M_PI) {
        a -= 2.0f * (float)M_PI;
    }
    while (a < -(float)M_PI) {
        a += 2.0f * (float)M_PI;
    }
    return a;
}

static void rot(const float R[3][3], const float v[3], float o[3])
{
    for (int i = 0; i < 3; i++) {
        o[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
    }
}

static void level_from_acc(float ax, float ay, float az)
{
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-6f) {
        n = 1.0f;
    }
    float ux = ax / n, uy = ay / n, uz = az / n;
    float vx = uy, vy = -ux;
    float s2 = vx * vx + vy * vy;
    float c = uz;
    if (s2 < 1e-12f) {
        if (c > 0.0f) {
            s_R[0][0] = 1; s_R[0][1] = 0; s_R[0][2] = 0;
            s_R[1][0] = 0; s_R[1][1] = 1; s_R[1][2] = 0;
            s_R[2][0] = 0; s_R[2][1] = 0; s_R[2][2] = 1;
        } else {
            s_R[0][0] = 1; s_R[0][1] = 0; s_R[0][2] = 0;
            s_R[1][0] = 0; s_R[1][1] = -1; s_R[1][2] = 0;
            s_R[2][0] = 0; s_R[2][1] = 0; s_R[2][2] = -1;
        }
        return;
    }
    float k = (1.0f - c) / s2;
    s_R[0][0] = 1.0f - k * vy * vy;
    s_R[0][1] = k * vx * vy;
    s_R[0][2] = vy;
    s_R[1][0] = k * vx * vy;
    s_R[1][1] = 1.0f - k * vx * vx;
    s_R[1][2] = -vx;
    s_R[2][0] = -vy;
    s_R[2][1] = vx;
    s_R[2][2] = 1.0f - k * s2;
}

static void to_robot(const float acc[3], const float gyr[3], float aout[3], float gout[3])
{
    float al[3], gl[3];
    rot(s_R, acc, al);
    rot(s_R, gyr, gl);
    float c = cosf(s_mount), s = sinf(s_mount);
    aout[0] = c * al[0] - s * al[1];
    aout[1] = s * al[0] + c * al[1];
    aout[2] = al[2];
    gout[0] = c * gl[0] - s * gl[1];
    gout[1] = s * gl[0] + c * gl[1];
    gout[2] = gl[2];
}

static esp_err_t read_board(float acc[3], float gyr[3])
{
    uint8_t raw[14];
    ESP_RETURN_ON_ERROR(rd(REG_ACCEL_XOUT, raw, 14), TAG, "read");
    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);
    acc[0] = ax / ACCEL_LSB;
    acc[1] = ay / ACCEL_LSB;
    acc[2] = az / ACCEL_LSB;
    gyr[0] = (gx / GYRO_LSB) * (float)M_PI / 180.0f;
    gyr[1] = (gy / GYRO_LSB) * (float)M_PI / 180.0f;
    gyr[2] = (gz / GYRO_LSB) * (float)M_PI / 180.0f;
    return ESP_OK;
}

static void fusion_update(float dt, const float a[3], const float g[3])
{
    float acc_roll = atan2f(a[1], a[2]);
    float acc_pitch = atan2f(-a[0], hypotf(a[1], a[2]));
    if (!s_fusion_on) {
        s_roll = acc_roll;
        s_pitch = acc_pitch;
        s_fusion_on = true;
        return;
    }
    s_roll += g[0] * dt;
    s_pitch += g[1] * dt;
    s_yaw = wrap_pi(s_yaw + g[2] * dt);
    float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (n > 0.8f && n < 1.2f) {
        float k = dt / (0.5f + dt);
        s_roll += k * (acc_roll - s_roll);
        s_pitch += k * (acc_pitch - s_pitch);
    }
}

static void do_calibrate(void)
{
    snprintf(s_status, sizeof(s_status), "calibrating: hold STILL");
    printf("IMU: hold the robot STILL for 2 s (gyro bias + front = arrow)...\n");
    vTaskDelay(pdMS_TO_TICKS(800));

    float acc_sum[3] = {0}, gyr_sum[3] = {0};
    int n = 0;
    for (int i = 0; i < 200; i++) {
        float acc[3], gyr[3];
        if (read_board(acc, gyr) == ESP_OK) {
            for (int k = 0; k < 3; k++) {
                acc_sum[k] += acc[k];
                gyr_sum[k] += gyr[k];
            }
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (n < 50) {
        snprintf(s_status, sizeof(s_status), "calib failed: no samples");
        printf("IMU calibrate failed (no samples)\n");
        return;
    }
    float acc[3];
    for (int k = 0; k < 3; k++) {
        acc[k] = acc_sum[k] / (float)n;
        s_bias[k] = gyr_sum[k] / (float)n;
    }
    level_from_acc(acc[0], acc[1], acc[2]);
    s_fusion_on = false;
    s_yaw = 0.0f;
    s_roll = 0.0f;
    s_pitch = 0.0f;
    s_calibrated = true;
    snprintf(s_status, sizeof(s_status), "ok: front is yaw 0");
    printf("IMU calibrated. Front of the robot is now the map arrow (yaw 0).\n");
    printf("  gyro bias %.2f %.2f %.2f dps  mount %.0f deg\n",
           s_bias[0] * 180.0f / (float)M_PI,
           s_bias[1] * 180.0f / (float)M_PI,
           s_bias[2] * 180.0f / (float)M_PI,
           s_mount_deg);
}

static void imu_task(void *arg)
{
    (void)arg;
    s_cmd = IMU_CMD_CALIB;
    int64_t prev = esp_timer_get_time();
    while (1) {
        int cmd = s_cmd;
        if (cmd == IMU_CMD_CALIB) {
            s_cmd = IMU_CMD_NONE;
            do_calibrate();
            if (s_done) {
                xSemaphoreGive(s_done);
            }
            prev = esp_timer_get_time();
            continue;
        }
        if (cmd == IMU_CMD_ZERO) {
            s_cmd = IMU_CMD_NONE;
            if (s_lock) {
                xSemaphoreTake(s_lock, portMAX_DELAY);
            }
            s_yaw = 0.0f;
            if (s_lock) {
                xSemaphoreGive(s_lock);
            }
            snprintf(s_status, sizeof(s_status), "ok: front is yaw 0");
            printf("IMU: current heading is now the front arrow (yaw 0).\n");
            if (s_done) {
                xSemaphoreGive(s_done);
            }
        }

        float acc[3], gyr[3];
        if (read_board(acc, gyr) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        int64_t now = esp_timer_get_time();
        float dt = (now - prev) / 1e6f;
        prev = now;
        if (dt <= 0.0f || dt > 0.1f) {
            dt = 0.01f;
        }

        float ub[3] = {gyr[0] - s_bias[0], gyr[1] - s_bias[1], gyr[2] - s_bias[2]};
        float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        float still_lim = STILL_DPS * (float)M_PI / 180.0f;
        bool still = fabsf(ub[0]) < still_lim && fabsf(ub[1]) < still_lim &&
                     fabsf(ub[2]) < still_lim && an > 0.95f && an < 1.05f;
        if (still) {
            float k = dt / (5.0f + dt);
            for (int i = 0; i < 3; i++) {
                s_bias[i] += k * (gyr[i] - s_bias[i]);
                ub[i] = gyr[i] - s_bias[i];
            }
        }

        float ar[3], gr[3];
        to_robot(acc, ub, ar, gr);

        if (s_lock) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
        }
        fusion_update(dt, ar, gr);
        s_gz = gr[2];
        s_still = still;
        if (s_lock) {
            xSemaphoreGive(s_lock);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t imu_start(void)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = IMU_SDA_GPIO,
        .scl_io_num = IMU_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "i2c bus");

    const uint8_t addrs[] = {MPU_ADDR_A, MPU_ADDR_B};
    esp_err_t found = ESP_ERR_NOT_FOUND;
    for (size_t i = 0; i < sizeof(addrs); i++) {
        if (i2c_master_probe(s_bus, addrs[i], 50) == ESP_OK) {
            s_addr = addrs[i];
            found = ESP_OK;
            break;
        }
    }
    if (found != ESP_OK) {
        ESP_LOGW(TAG, "no IMU on GPIO%d/GPIO%d (I2C 0x68/0x69)", IMU_SDA_GPIO, IMU_SCL_GPIO);
        return ESP_ERR_NOT_FOUND;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_addr,
        .scl_speed_hz = 100000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev), TAG, "i2c dev");

    ESP_RETURN_ON_ERROR(rd(REG_WHO_AM_I, &s_who, 1), TAG, "WHO_AM_I");
    (void)wr(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(wr(REG_PWR_MGMT_1, 0x01), TAG, "wake");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(wr(REG_SMPLRT_DIV, 9), TAG, "rate");
    ESP_RETURN_ON_ERROR(wr(REG_CONFIG, 0x03), TAG, "dlpf");
    ESP_RETURN_ON_ERROR(wr(REG_GYRO_CFG, 0x08), TAG, "gyro");
    ESP_RETURN_ON_ERROR(wr(REG_ACCEL_CFG, 0x08), TAG, "accel");

    s_lock = xSemaphoreCreateMutex();
    s_done = xSemaphoreCreateBinary();
    s_ready = true;
    ESP_LOGI(TAG, "%s WHO_AM_I=0x%02X at 0x%02X  SDA=GPIO%d SCL=GPIO%d  mount %.0f deg",
             chip_name(s_who), s_who, s_addr, IMU_SDA_GPIO, IMU_SCL_GPIO, s_mount_deg);
    printf("IMU %s  WHO_AM_I=0x%02X  addr 0x%02X  SDA=GPIO%d SCL=GPIO%d  mount %.0f deg (X=back)\n",
           chip_name(s_who), s_who, s_addr, IMU_SDA_GPIO, IMU_SCL_GPIO, s_mount_deg);

    BaseType_t ok = xTaskCreate(imu_task, "imu", 4096, NULL, 6, NULL);
    ESP_RETURN_ON_FALSE(ok == pdTRUE, ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

bool imu_is_ready(void)
{
    return s_ready;
}

bool imu_is_calibrated(void)
{
    return s_calibrated;
}

void imu_bus_scan(void)
{
    if (s_bus == NULL) {
        printf("I2C not started\n");
        return;
    }
    printf("I2C scan GPIO%d/GPIO%d:", IMU_SDA_GPIO, IMU_SCL_GPIO);
    int n = 0;
    for (uint8_t a = 0x03; a <= 0x77; a++) {
        if (i2c_master_probe(s_bus, a, 20) == ESP_OK) {
            printf("  0x%02X", a);
            n++;
        }
    }
    printf(n ? "\n" : "  (none)\n");
}

esp_err_t imu_read(void)
{
    if (!s_ready) {
        printf("IMU not ready. SDA=GPIO%d SCL=GPIO%d\n", IMU_SDA_GPIO, IMU_SCL_GPIO);
        return ESP_ERR_INVALID_STATE;
    }
    float r, p, y;
    imu_get_rpy(&r, &p, &y);
    printf("IMU %s  roll %+6.1f  pitch %+6.1f  yaw %+6.1f deg  (0=front arrow)  %s  %s\n",
           chip_name(s_who),
           r * 180.0f / (float)M_PI, p * 180.0f / (float)M_PI, y * 180.0f / (float)M_PI,
           s_still ? "still" : "moving",
           s_status);
    return ESP_OK;
}

esp_err_t imu_calibrate(uint32_t timeout_ms)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_done) {
        xSemaphoreTake(s_done, 0);
    }
    s_cmd = IMU_CMD_CALIB;
    if (s_done && xSemaphoreTake(s_done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return s_calibrated ? ESP_OK : ESP_FAIL;
}

void imu_zero_yaw(void)
{
    if (!s_ready) {
        return;
    }
    if (s_done) {
        xSemaphoreTake(s_done, 0);
    }
    s_cmd = IMU_CMD_ZERO;
    if (s_done) {
        xSemaphoreTake(s_done, pdMS_TO_TICKS(500));
    }
}

void imu_get_rpy(float *roll, float *pitch, float *yaw)
{
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    if (roll) {
        *roll = s_roll;
    }
    if (pitch) {
        *pitch = s_pitch;
    }
    if (yaw) {
        *yaw = s_yaw;
    }
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}

float imu_yaw(void)
{
    float y;
    imu_get_rpy(NULL, NULL, &y);
    return y;
}

float imu_yaw_deg(void)
{
    return imu_yaw() * 180.0f / (float)M_PI;
}

float imu_gz(void)
{
    float z;
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    z = s_gz;
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
    return z;
}

bool imu_is_still(void)
{
    return s_still;
}

const char *imu_status(void)
{
    return s_status;
}

void imu_set_mount_yaw_deg(float deg)
{
    s_mount_deg = deg;
    s_mount = deg * (float)M_PI / 180.0f;
}

float imu_mount_yaw_deg(void)
{
    return s_mount_deg;
}
