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
#include "RB02_Checklist.h"
#ifdef RB_ENABLE_CHECKLIST
#include <stdio.h>
#include <string.h>

extern lv_style_t style_title;

lv_obj_t *RB02_Checklist_Addline(lv_obj_t *parent, char *line)
{

    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    lv_obj_set_width(label, 440);
    lv_obj_set_height(label, LV_SIZE_CONTENT);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

    lv_label_set_text(label, line);
    // lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    return label;
}

lv_obj_t *RB02_Checklist_CreateScreen(lv_obj_t *parent, char *path)
{

    FILE *f = fopen(path, "r");
    
    if (f == NULL)
    {
        RB02_Checklist_Addline(parent, "CHECKS\nload your check.txt\nin sdcard");
    }
    else
    {

        char *buf = calloc(1, 4096);
        if (buf != NULL)
        {
            memset(buf, 0, 4096);
            char line[128];
            int16_t index = 0;
            while (fgets(line, sizeof(line), f))
            {
                // Single line load, works very well if you checklist is "tailored" <-- the best
                if (false)
                {
                    char *pos = strchr(line, '\n');
                    if (pos)
                    {
                        *pos = '\0';
                    }
                    RB02_Checklist_Addline(parent, line);
                }
                // multiline with also support \n
                if (true)
                {
                    memcpy(buf + index, line, strlen(line));
                    index = index + strlen(line);
                }
            }
            RB02_Checklist_Addline(parent, buf);
            free(buf);
        }
        fclose(f);
    }
    return NULL;
}
#endif