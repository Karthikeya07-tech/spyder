#include "embed.h"
#include <string.h>

extern const uint8_t neutral_bin_start[] asm("_binary_neutral_bin_start");
extern const uint8_t walk_forward_bin_start[] asm("_binary_walk_forward_bin_start");
extern const uint8_t walk_backward_bin_start[] asm("_binary_walk_backward_bin_start");
extern const uint8_t turn_left_bin_start[] asm("_binary_turn_left_bin_start");
extern const uint8_t turn_right_bin_start[] asm("_binary_turn_right_bin_start");

static action_t s_actions[] = {
    { "neutral", (const uint16_t *)neutral_bin_start, 90 },
    { "walk_forward", (const uint16_t *)walk_forward_bin_start, 1026 },
    { "walk_backward", (const uint16_t *)walk_backward_bin_start, 1026 },
    { "turn_left", (const uint16_t *)turn_left_bin_start, 324 },
    { "turn_right", (const uint16_t *)turn_right_bin_start, 324 },
};

size_t action_count(void) { return sizeof(s_actions) / sizeof(s_actions[0]); }
const action_t *action_at(size_t i) {
    return i < action_count() ? &s_actions[i] : NULL;
}
const action_t *action_by_name(const char *name) {
    if (!name) return NULL;
    for (size_t i = 0; i < action_count(); i++) {
        if (strcmp(s_actions[i].name, name) == 0) return &s_actions[i];
    }
    return NULL;
}

extern const uint8_t map_cells_bin_start[] asm("_binary_map_cells_bin_start");
const int8_t *twist_map_cells(void)
{
    return (const int8_t *)map_cells_bin_start;
}
const float twist_map_res = 0.05f;
const float twist_map_ox = -10.0f;
const float twist_map_oy = -10.0f;
