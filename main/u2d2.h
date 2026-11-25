#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define U2D2_BAUDRATE          222222
#define U2D2_VID               0x0403
#define U2D2_PID_FT232H        0x6014  /* ROBOTIS U2D2 (FT232HL) */
#define U2D2_PID_FT232R        0x6001  /* older USB2Dynamixel */
#define USB2AX_VID             0x16D0  /* Xevelabs USB2AX (CDC ACM) */
#define USB2AX_PID             0x06A7

esp_err_t u2d2_start(void);
esp_err_t u2d2_wait_ready(uint32_t timeout_ms);
bool u2d2_is_ready(void);
void u2d2_usb_scan(void);
void u2d2_swap_dp_dm(bool swap);
/* Stop USB2AX VID probing so LiDAR SCAN can use the host. */
void u2d2_pause_probe(bool pause);

void u2d2_rx_flush(void);
esp_err_t u2d2_write(const uint8_t *data, size_t len);
/* Appends received bytes into buf. Returns bytes copied, 0 on timeout. */
size_t u2d2_read(uint8_t *buf, size_t max_len, uint32_t timeout_ms);
