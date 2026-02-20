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
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/
#include "RB02_Console.h"
#ifdef RB_ENABLE_CONSOLE
#include "RB02_Config.h"
#include <stdio.h>
#include <string.h>

extern RB02_Status *rb02Status;

void RB02_Console_AppendLog(uint8_t sourceId, uint8_t logLevel, const char *string)
{

    if(rb02Status==NULL || rb02Status->ui.console == NULL)
    {
        return;
    }



    const char *currentText = lv_label_get_text(singletonConfig()->ui.console);

    size_t begin = strlen(currentText);
    size_t append = strlen(string);
    size_t newLen = begin + append + 2;


    if(begin>960)
    {
        begin=960;
    }

    char *buf = calloc(1, newLen);
    if (buf != NULL)
    {
        memset(buf, 0, newLen);
        memcpy(buf, string, append);
        buf[append] = '\n';
        memcpy(buf + append + 1, currentText, begin);
        lv_label_set_text(singletonConfig()->ui.console, buf);
        free(buf);
    }
}

lv_obj_t *RB02_Console_CreateScreen(lv_obj_t *parent)
{

    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, -228);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_label_set_text(label, RB_PRODUCT_TITLE);
    }

    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, -194);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_label_set_text(label, "CONSOLE");
    }
    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_set_width(label, 440);
        lv_obj_set_height(label, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 56);
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);

        lv_label_set_text(label, RB_PRODUCT_TITLE " Startup");

        singletonConfig()->ui.console = label;
    }
    return NULL;
}
#endif