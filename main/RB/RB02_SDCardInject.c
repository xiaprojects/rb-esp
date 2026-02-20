#pragma once

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
 * 
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/



#include "lvgl.h"
#include "esp_vfs.h"
#include "esp_err.h"
#include <stdio.h>

static void *my_fs_open_cb(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode) {
    char full_path[128];
    snprintf(full_path, sizeof(full_path), "/sdcard/%s", path);

    const char *flags = (mode == LV_FS_MODE_WR) ? "wb" :
                        (mode == LV_FS_MODE_RD) ? "rb" : "rb+";

    FILE *f = fopen(full_path, flags);
    return f;
}

static lv_fs_res_t my_fs_close_cb(lv_fs_drv_t *drv, void *file_p) {
    fclose((FILE *)file_p);
    return LV_FS_RES_OK;
}

static lv_fs_res_t my_fs_read_cb(lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br) {
    *br = fread(buf, 1, btr, (FILE *)file_p);
    return LV_FS_RES_OK;
}

static lv_fs_res_t my_fs_seek_cb(lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence) {
    int w = (whence == LV_FS_SEEK_CUR) ? SEEK_CUR :
            (whence == LV_FS_SEEK_END) ? SEEK_END : SEEK_SET;
    fseek((FILE *)file_p, pos, w);
    return LV_FS_RES_OK;
}

static lv_fs_res_t my_fs_tell_cb(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p) {
    *pos_p = ftell((FILE *)file_p);
    return LV_FS_RES_OK;
}

void lvgl_register_sdcard_fs() {
    static lv_fs_drv_t fs_drv;
    lv_fs_drv_init(&fs_drv);

    fs_drv.letter = 'S';                   // S:/image.jpg
    fs_drv.open_cb = my_fs_open_cb;
    fs_drv.close_cb = my_fs_close_cb;
    fs_drv.read_cb = my_fs_read_cb;
    fs_drv.seek_cb = my_fs_seek_cb;
    fs_drv.tell_cb = my_fs_tell_cb;

    lv_fs_drv_register(&fs_drv);
}