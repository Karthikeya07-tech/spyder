#pragma once
#include <stddef.h>
#include <stdint.h>

#define ACTION_SERVOS  12
#define ACTION_DT_MS   30

typedef struct {
    const char *name;
    const uint16_t *ticks; /* frames * 12, little-endian in flash */
    size_t frames;
} action_t;

const action_t *action_by_name(const char *name);
size_t action_count(void);
const action_t *action_at(size_t i);

#define TWIST_MAP_W    400
#define TWIST_MAP_H    400
const int8_t *twist_map_cells(void);
extern const float twist_map_res;
extern const float twist_map_ox;
extern const float twist_map_oy;
