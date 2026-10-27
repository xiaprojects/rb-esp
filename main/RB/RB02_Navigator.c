/*
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

#include "RB02_Navigator.h"
#ifdef DISABLED
lv_obj_t *RB02_Navigator_CreateScreen(RB02_Status *status,lv_obj_t *parent)
{

    if (parent != NULL)
    {
        lv_coord_t w = SCREEN_WIDTH/5;
        lv_obj_t *bullet = lv_obj_create(parent);
        lv_obj_clear_flag(bullet, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(bullet, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(bullet,w,w);

        lv_obj_align(bullet, LV_ALIGN_CENTER, (SCREEN_WIDTH/4),0);
        lv_obj_set_style_bg_color(bullet, lv_color_white(), 0);
        lv_obj_set_style_border_color(bullet, lv_color_black(), 0);
        lv_obj_set_style_radius(bullet, LV_RADIUS_CIRCLE, 0);
    }


    if (parent != NULL)
    {
        lv_coord_t w = SCREEN_WIDTH/5;
        lv_obj_t *bullet = lv_obj_create(parent);
        lv_obj_clear_flag(bullet, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(bullet, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(bullet,w,w);

        lv_obj_align(bullet, LV_ALIGN_CENTER, (SCREEN_WIDTH/4),0);
        lv_obj_set_style_bg_color(bullet, lv_color_white(), 0);
        lv_obj_set_style_border_color(bullet, lv_color_black(), 0);
        lv_obj_set_style_radius(bullet, LV_RADIUS_CIRCLE, 0);
    }

        if (parent != NULL)
    {
        lv_coord_t w = SCREEN_WIDTH/5;
        lv_obj_t *bullet = lv_obj_create(parent);
        lv_obj_clear_flag(bullet, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(bullet, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(bullet,w,w);

        lv_obj_align(bullet, LV_ALIGN_CENTER, 0,(SCREEN_WIDTH/4));
        lv_obj_set_style_bg_color(bullet, lv_color_white(), 0);
        lv_obj_set_style_border_color(bullet, lv_color_black(), 0);
        lv_obj_set_style_radius(bullet, LV_RADIUS_CIRCLE, 0);
    }

        if (parent != NULL)
    {
        lv_coord_t w = SCREEN_WIDTH/5;
        lv_obj_t *bullet = lv_obj_create(parent);
        lv_obj_clear_flag(bullet, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(bullet, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(bullet,w,w);

        lv_obj_align(bullet, LV_ALIGN_CENTER, 0,(SCREEN_WIDTH/4));
        lv_obj_set_style_bg_color(bullet, lv_color_white(), 0);
        lv_obj_set_style_border_color(bullet, lv_color_black(), 0);
        lv_obj_set_style_radius(bullet, LV_RADIUS_CIRCLE, 0);
    }

        if (parent != NULL)
    {
        lv_coord_t w = SCREEN_WIDTH/5;
        lv_obj_t *bullet = lv_obj_create(parent);
        lv_obj_clear_flag(bullet, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(bullet, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(bullet,w,w);

        lv_obj_align(bullet, LV_ALIGN_CENTER, 0,0);
        lv_obj_set_style_bg_color(bullet, lv_color_white(), 0);
        lv_obj_set_style_border_color(bullet, lv_color_black(), 0);
        lv_obj_set_style_radius(bullet, LV_RADIUS_CIRCLE, 0);
    }

    return NULL;
}
#endif