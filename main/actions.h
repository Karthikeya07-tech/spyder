#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* Clips repeat every 54 frames (1.62 s at 30 ms). Walk bins are 19 cycles, turns are 6.
 * Metres per walk cycle, degrees per turn cycle, and the body size are measured on the robot.
 * Leave them unused until those three numbers are written down. */
#define GAIT_CYCLE_FRAMES  54

void actions_list(void);
/* Blocking. cycles 0 plays the whole clip. cycles N plays N gait cycles (not used for neutral). */
esp_err_t action_play(const char *name);
esp_err_t action_play_cycles(const char *name, unsigned cycles);

/* Player task. HTTP/WebSocket enqueue here so a long walk does not block the server. */
esp_err_t actions_start(void);
/* Non-blocking. ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_STATE (link down), ESP_FAIL (queue full). */
esp_err_t action_request(const char *name);
esp_err_t action_request_cycles(const char *name, unsigned cycles);
/* End the current clip and queue neutral. */
void action_stop(void);
/* True only while walk_forward or walk_backward is the clip actually running. */
bool action_is_walking(void);
void action_status(bool *busy, char *name, size_t name_len, unsigned *queued);
