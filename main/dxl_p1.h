#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define DXL_BROADCAST_ID     0xFE
#define DXL_MAX_ID           253

#define DXL_INSTR_PING        0x01
#define DXL_INSTR_READ        0x02
#define DXL_INSTR_WRITE       0x03
#define DXL_INSTR_SYNC_WRITE  0x83

#define DXL_ADDR_MODEL       0x00
#define DXL_ADDR_TORQUE      0x18
#define DXL_ADDR_GOAL_POS    0x1E
#define DXL_ADDR_PRESENT_POS 0x24

esp_err_t dxl_ping(uint8_t id);
esp_err_t dxl_read_u16(uint8_t id, uint8_t addr, uint16_t *value);
esp_err_t dxl_write_u8(uint8_t id, uint8_t addr, uint8_t value);
esp_err_t dxl_write_u16(uint8_t id, uint8_t addr, uint16_t value);
esp_err_t dxl_sync_write_goal(const uint8_t *ids, const uint16_t *pos, size_t n);

int dxl_scan(uint8_t max_id);
esp_err_t dxl_move_deg(uint8_t id, float angle_deg);
esp_err_t dxl_read_pos_deg(uint8_t id, float *angle_deg, uint16_t *raw);

bool dxl_is_mx(uint16_t model);
const char *dxl_model_name(uint16_t model);
uint16_t dxl_cached_model(uint8_t id);
