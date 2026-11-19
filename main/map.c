#include "map.h"

#include <math.h>
#include <stdio.h>
#include "embed.h"
#include "lidar.h"

#define OCC_THRESH  0

void map_print(void)
{
    const int8_t *c = twist_map_cells();
    int occ = 0, free_n = 0, unk = 0;
    for (int i = 0; i < TWIST_MAP_W * TWIST_MAP_H; i++) {
        if (c[i] > OCC_THRESH) {
            occ++;
        } else if (c[i] < -OCC_THRESH) {
            free_n++;
        } else {
            unk++;
        }
    }
    printf("Map loaded: %dx%d  res %.3f m  origin (%.2f, %.2f) m\n",
           TWIST_MAP_W, TWIST_MAP_H, twist_map_res, twist_map_ox, twist_map_oy);
    printf("  occupied %d  free %d  unknown %d  (%.1f x %.1f m)\n",
           occ, free_n, unk, TWIST_MAP_W * twist_map_res, TWIST_MAP_H * twist_map_res);

    const int step_x = TWIST_MAP_W / 40;
    const int step_y = TWIST_MAP_H / 20;
    printf("ASCII (downsampled, #=wall .=free):\n");
    for (int gy = TWIST_MAP_H - step_y; gy >= 0; gy -= step_y) {
        printf("  ");
        for (int gx = 0; gx < TWIST_MAP_W; gx += step_x) {
            int8_t v = c[gy * TWIST_MAP_W + gx];
            putchar(v > OCC_THRESH ? '#' : (v < -OCC_THRESH ? '.' : ' '));
        }
        putchar('\n');
    }
}

esp_err_t map_compare_last_scan(void)
{
    const float *r = NULL;
    int n = lidar_last_n(&r);
    if (n <= 0 || r == NULL) {
        printf("No LiDAR scan yet. Type: lscan\n");
        return ESP_ERR_INVALID_STATE;
    }
    const int8_t *c = twist_map_cells();
    int hit_occ = 0, hit_free = 0, hit_unk = 0, used = 0;
    const float ainc = 6.28318531f / 360.0f;
    /* Assume robot at map origin + 10,10 m (npz origin is -10,-10). Pose (0,0) in map frame.
     * Scan bins are robot-frame: 0 = front, CCW. */
    const float rx = 0.0f;
    const float ry = 0.0f;
    for (int i = 0; i < n; i++) {
        if (!(r[i] > 0.15f) || r[i] > 8.0f || !isfinite(r[i])) {
            continue;
        }
        float ang = i * ainc;
        float x = rx + r[i] * cosf(ang);
        float y = ry + r[i] * sinf(ang);
        int ix = (int)floorf((x - twist_map_ox) / twist_map_res);
        int iy = (int)floorf((y - twist_map_oy) / twist_map_res);
        if (ix < 0 || iy < 0 || ix >= TWIST_MAP_W || iy >= TWIST_MAP_H) {
            continue;
        }
        used++;
        int8_t v = c[iy * TWIST_MAP_W + ix];
        if (v > OCC_THRESH) {
            hit_occ++;
        } else if (v < -OCC_THRESH) {
            hit_free++;
        } else {
            hit_unk++;
        }
    }
    printf("Last scan vs map (pose assumed 0,0): %d beams on map, %d occupied, %d free, %d unknown\n",
           used, hit_occ, hit_free, hit_unk);
    printf("Full scan-match / localisation is next once you want that step.\n");
    return ESP_OK;
}
