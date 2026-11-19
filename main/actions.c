#include "actions.h"

#include <stdio.h>
#include "embed.h"
#include "dxl_p1.h"
#include "u2d2.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "action";
static const uint8_t s_ids[ACTION_SERVOS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

void actions_list(void)
{
    printf("Actions (quadruped, 12x AX, %d ms/frame):\n", ACTION_DT_MS);
    for (size_t i = 0; i < action_count(); i++) {
        const action_t *a = action_at(i);
        printf("  %-14s  %4u frames  (%.2f s)\n",
               a->name, (unsigned)a->frames, a->frames * ACTION_DT_MS / 1000.0f);
    }
    printf("Play: action <name>\n");
}

esp_err_t action_play(const char *name)
{
    const action_t *a = action_by_name(name);
    if (a == NULL) {
        printf("unknown action '%s'\n", name ? name : "");
        actions_list();
        return ESP_ERR_NOT_FOUND;
    }
    if (!u2d2_is_ready()) {
        printf("USB2AX not ready\n");
        return ESP_ERR_INVALID_STATE;
    }

    printf("Playing %s  %u frames  %.2f s\n",
           a->name, (unsigned)a->frames, a->frames * ACTION_DT_MS / 1000.0f);
    for (size_t i = 0; i < ACTION_SERVOS; i++) {
        dxl_write_u8(s_ids[i], DXL_ADDR_TORQUE, 1);
    }

    for (size_t f = 0; f < a->frames; f++) {
        uint16_t pos[ACTION_SERVOS];
        for (size_t s = 0; s < ACTION_SERVOS; s++) {
            pos[s] = a->ticks[f * ACTION_SERVOS + s];
        }
        esp_err_t err = dxl_sync_write_goal(s_ids, pos, ACTION_SERVOS);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "frame %u: %s", (unsigned)f, esp_err_to_name(err));
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(ACTION_DT_MS));
    }
    printf("action %s done\n", a->name);
    return ESP_OK;
}
