/**
 * This file is part of RB.
 *
 * Copyright (C) 2024 XIAPROJECTS SRL
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.

 * This source is part of the project RB:
 * 01 -> Display with Synthetic vision, Autopilot and ADSB
 * 02 -> Display with SixPack
 * 03 -> Display with Autopilot, ADSB, Radio, Flight Computer
 * 04 -> Display with EMS: Engine monitoring system
 * 05 -> Display with Stratux BLE Traffic
 * 06 -> Display with Android 6.25" 7" 8" 10" 10.2"
 * 07 -> Display with Stratux BLE Traffic composed by RB-05 + RB-03 in the same box
 * 08 -> Voice Recognition Box with LLM and Natural speaking and Voice Recorder
 * Cloud -> Cloud services for RB devices, including flight data recording, flight data analysis, flight sharing and more
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
 * More about in https://www.rbavionics.com and Discord community
 * 
 */

#include "lvgl.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

static const char *TAG = "shot";

/* Fixed name: LVGL 8 lv_color_t -> BGR24 */
static inline void lv8_px_to_bgr24(lv_color_t c, uint8_t *bgr)
{
    uint32_t c32 = lv_color_to32(c);
    bgr[0] = (uint8_t)(c32 >>  0) & 0xFF;  /* B */
    bgr[1] = (uint8_t)(c32 >>  8) & 0xFF;  /* G */
    bgr[2] = (uint8_t)(c32 >> 16) & 0xFF;  /* R */
}

/* Little-endian helpers */
static void le16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static void le32(uint8_t *p, uint32_t v) { p[0]=v&0xFF; p[1]=(v>>8)&0xFF; p[2]=(v>>16)&0xFF; p[3]=(v>>24)&0xFF; }

/* Now compiles: save LVGL 8 screen to SD as BMP */
esp_err_t lvgl8_save_screen_bmp_to_sd(const char *path)
{
#if !LV_USE_SNAPSHOT
    (void)path;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if(!path) return ESP_ERR_INVALID_ARG;

    lv_obj_t *obj = lv_scr_act();
    lv_img_dsc_t *dsc = lv_snapshot_take(obj, LV_IMG_CF_TRUE_COLOR_ALPHA);
    if(!dsc) return ESP_ERR_NO_MEM;

    const uint32_t w = dsc->header.w;
    const uint32_t h = dsc->header.h;
    const uint32_t px_size = sizeof(lv_color_t) + 1; /* color + alpha */
    const uint8_t *src = (const uint8_t *)dsc->data;

    const uint32_t row_bytes = w * 3;
    const uint32_t row_padded = (row_bytes + 3) & ~3U;
    const uint32_t pixel_data_size = row_padded * h;
    const uint32_t file_size = 14 + 40 + pixel_data_size;

    FILE *f = fopen(path, "wb");
    if(!f) {
        lv_snapshot_free(dsc);
        return ESP_FAIL;
    }

    /* BMP headers */
    uint8_t file_hdr[14] = {0};
    file_hdr[0] = 'B'; file_hdr[1] = 'M';
    le32(&file_hdr[2], file_size);
    le32(&file_hdr[10], 14 + 40);

    uint8_t info_hdr[40] = {0};
    le32(&info_hdr[0], 40);
    le32(&info_hdr[4], w);
    le32(&info_hdr[8], h);
    le16(&info_hdr[12], 1);
    le16(&info_hdr[14], 24);
    le32(&info_hdr[20], pixel_data_size);

    if(fwrite(file_hdr, 1, sizeof(file_hdr), f) != sizeof(file_hdr) ||
       fwrite(info_hdr, 1, sizeof(info_hdr), f) != sizeof(info_hdr)) {
        fclose(f);
        lv_snapshot_free(dsc);
        return ESP_FAIL;
    }

    uint8_t *row = (uint8_t *)malloc(row_padded);
    if(!row) {
        fclose(f);
        lv_snapshot_free(dsc);
        return ESP_ERR_NO_MEM;
    }

    /* Bottom-up BMP */
    for(int32_t y = (int32_t)h - 1; y >= 0; y--) {
        uint8_t *p = row;
        const uint8_t *line = src + (uint32_t)y * (w * px_size);

        for(uint32_t x = 0; x < w; x++) {
            const lv_color_t *c = (const lv_color_t *)(line + x * px_size);
            lv8_px_to_bgr24(*c, p);  /* Now matches declaration */
            p += 3;
        }
        while((uint32_t)(p - row) < row_padded) *p++ = 0;

        if(fwrite(row, 1, row_padded, f) != row_padded) {
            free(row);
            fclose(f);
            lv_snapshot_free(dsc);
            return ESP_FAIL;
        }
    }

    free(row);
    fclose(f);
    lv_snapshot_free(dsc);

    ESP_LOGI(TAG, "Saved %ux%u BMP to %s", (unsigned)w, (unsigned)h, path);
    return ESP_OK;
#endif
}
