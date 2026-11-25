#include "actions.h"

#include <stdio.h>
#include <string.h>
#include "embed.h"
#include "dxl_p1.h"
#include "u2d2.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "action";
static const uint8_t s_ids[ACTION_SERVOS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

typedef struct {
    char name[24];
    uint16_t cycles; /* 0 = the whole clip */
} action_job_t;

static SemaphoreHandle_t s_play;
static SemaphoreHandle_t s_state;
static QueueHandle_t s_q;
static bool s_busy;
static char s_current[32];
static volatile int s_cancel_gen;

static void set_playing(const char *name)
{
    if (s_state) {
        xSemaphoreTake(s_state, portMAX_DELAY);
    }
    if (name && name[0]) {
        s_busy = true;
        strncpy(s_current, name, sizeof(s_current) - 1);
        s_current[sizeof(s_current) - 1] = '\0';
    } else {
        s_busy = false;
        s_current[0] = '\0';
    }
    if (s_state) {
        xSemaphoreGive(s_state);
    }
}

static void copy_action_name(char *dst, size_t cap, const char *src)
{
    size_t n = 0;
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }
    while (src[n] == ' ') {
        src++;
    }
    while (src[n] && src[n] != ' ' && src[n] != '\r' && src[n] != '\n' &&
           src[n] != '"' && src[n] != '}' && n + 1 < cap) {
        dst[n] = src[n];
        n++;
    }
    dst[n] = '\0';
}

void actions_list(void)
{
    printf("Actions (quadruped, 12x AX, %d ms/frame):\n", ACTION_DT_MS);
    for (size_t i = 0; i < action_count(); i++) {
        const action_t *a = action_at(i);
        printf("  %-14s  %4u frames  (%.2f s)\n",
               a->name, (unsigned)a->frames, a->frames * ACTION_DT_MS / 1000.0f);
    }
    printf("  One cycle is %d frames (%.2f s). Walk clips are %u cycles, turns are %u.\n",
           GAIT_CYCLE_FRAMES, GAIT_CYCLE_FRAMES * ACTION_DT_MS / 1000.0f,
           1026 / GAIT_CYCLE_FRAMES, 324 / GAIT_CYCLE_FRAMES);
    printf("Play: action <name> [cycles]    Stop: action stop\n");
    printf("Step length, turn angle, and body size are not measured yet.\n");
}

static esp_err_t action_play_job(const action_job_t *job)
{
    const action_t *a = action_by_name(job->name);
    if (a == NULL) {
        printf("unknown action '%s'\n", job->name);
        actions_list();
        return ESP_ERR_NOT_FOUND;
    }
    if (!u2d2_is_ready()) {
        printf("USB2AX not ready\n");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_play && xSemaphoreTake(s_play, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    size_t limit = a->frames;
    if (job->cycles > 0 && strcmp(a->name, "neutral") != 0) {
        size_t want = (size_t)job->cycles * GAIT_CYCLE_FRAMES;
        if (want < limit) {
            limit = want;
        }
    }

    set_playing(a->name);
    int gen = s_cancel_gen;

    printf("Playing %s  %u/%u frames  %.2f s\n",
           a->name, (unsigned)limit, (unsigned)a->frames,
           limit * ACTION_DT_MS / 1000.0f);
    for (size_t i = 0; i < ACTION_SERVOS; i++) {
        dxl_write_u8(s_ids[i], DXL_ADDR_TORQUE, 1);
    }

    esp_err_t err = ESP_OK;
    for (size_t f = 0; f < limit; f++) {
        if (s_cancel_gen != gen) {
            printf("action %s stopped at frame %u\n", a->name, (unsigned)f);
            err = ESP_ERR_INVALID_STATE;
            break;
        }
        uint16_t pos[ACTION_SERVOS];
        for (size_t s = 0; s < ACTION_SERVOS; s++) {
            pos[s] = a->ticks[f * ACTION_SERVOS + s];
        }
        err = dxl_sync_write_goal(s_ids, pos, ACTION_SERVOS);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "frame %u: %s", (unsigned)f, esp_err_to_name(err));
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(ACTION_DT_MS));
    }
    if (err == ESP_OK) {
        printf("action %s done\n", a->name);
    }
    set_playing(NULL);
    if (s_play) {
        xSemaphoreGive(s_play);
    }
    return err;
}

esp_err_t action_play_cycles(const char *name, unsigned cycles)
{
    action_job_t job = {0};
    copy_action_name(job.name, sizeof(job.name), name);
    job.cycles = cycles > 65535u ? 65535u : (uint16_t)cycles;
    return action_play_job(&job);
}

esp_err_t action_play(const char *name)
{
    return action_play_cycles(name, 0);
}

static void action_task(void *arg)
{
    (void)arg;
    for (;;) {
        action_job_t job;
        if (xQueueReceive(s_q, &job, portMAX_DELAY) == pdTRUE) {
            (void)action_play_job(&job);
        }
    }
}

esp_err_t actions_start(void)
{
    if (s_q) {
        return ESP_OK;
    }
    s_play = xSemaphoreCreateMutex();
    s_state = xSemaphoreCreateMutex();
    s_q = xQueueCreate(2, sizeof(action_job_t));
    if (s_play == NULL || s_state == NULL || s_q == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(action_task, "action", 6144, NULL, 5, NULL) != pdTRUE) {
        vQueueDelete(s_q);
        s_q = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t enqueue(const char *name, unsigned cycles)
{
    action_job_t job = {0};
    copy_action_name(job.name, sizeof(job.name), name);
    if (action_by_name(job.name) == NULL) {
        printf("unknown action '%s'\n", job.name);
        return ESP_ERR_NOT_FOUND;
    }
    if (!u2d2_is_ready() || s_q == NULL) {
        printf("USB2AX not ready\n");
        return ESP_ERR_INVALID_STATE;
    }
    job.cycles = cycles > 65535u ? 65535u : (uint16_t)cycles;
    if (xQueueSend(s_q, &job, 0) != pdTRUE) {
        printf("action queue full\n");
        return ESP_FAIL;
    }
    if (job.cycles) {
        printf("Queued action %s  %u cycle(s)\n", job.name, job.cycles);
    } else {
        printf("Queued action %s\n", job.name);
    }
    return ESP_OK;
}

esp_err_t action_request(const char *name)
{
    return enqueue(name, 0);
}

esp_err_t action_request_cycles(const char *name, unsigned cycles)
{
    return enqueue(name, cycles);
}

void action_stop(void)
{
    s_cancel_gen++;
    if (s_q) {
        xQueueReset(s_q);
    }
    if (!u2d2_is_ready() || s_q == NULL) {
        printf("Action stop. USB2AX not ready, neutral not queued.\n");
        return;
    }
    action_job_t job = {0};
    strncpy(job.name, "neutral", sizeof(job.name) - 1);
    if (xQueueSend(s_q, &job, 0) != pdTRUE) {
        printf("Action stop. Neutral did not queue.\n");
        return;
    }
    printf("Action stop. Current clip will end, then neutral.\n");
}

bool action_is_walking(void)
{
    bool busy = false;
    char name[32];
    action_status(&busy, name, sizeof(name), NULL);
    return busy && (strcmp(name, "walk_forward") == 0 || strcmp(name, "walk_backward") == 0);
}

void action_status(bool *busy, char *name, size_t name_len, unsigned *queued)
{
    bool got = s_state && xSemaphoreTake(s_state, pdMS_TO_TICKS(20)) == pdTRUE;
    if (busy) {
        *busy = got ? s_busy : true;
    }
    if (name && name_len) {
        if (got) {
            strncpy(name, s_current, name_len - 1);
            name[name_len - 1] = '\0';
        } else {
            name[0] = '\0';
        }
    }
    if (queued) {
        *queued = s_q ? (unsigned)uxQueueMessagesWaiting(s_q) : 0;
    }
    if (got) {
        xSemaphoreGive(s_state);
    }
}
