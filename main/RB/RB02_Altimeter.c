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
#include "RB02_Altimeter.h"
#ifdef RB_ENABLE_ALT
#include "RB02_GUIHelpers.h"
#include "RB02_Config.h"
#include <stdio.h>

extern lv_style_t style_title;
extern const lv_img_dsc_t RoundAltimeter;
extern const lv_img_dsc_t fi_needle_small;
extern const lv_img_dsc_t fi_needle;
lv_obj_t *Screen_Altitude_QNH = NULL;
lv_obj_t *Screen_Altitude_Miles = NULL;
lv_obj_t *Screen_Altitude_Cents = NULL;


lv_obj_t *RB02_Altimeter_CreateScreen(lv_obj_t *parent)
{

    RB02_GUIHelpers_CreateBase(parent, &RoundAltimeter);

    // 1.1.1 Branding RB-02 on every screen
    if (parent != NULL)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, 96, 40);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, -130);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, RB_PRODUCT_TITLE);
        lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    }

    if (parent != NULL)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT,LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, -120,-30);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(label,lv_color_white(),0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, "GPS");
    }

    if (parent != NULL && singletonConfig()->ui.altimeterAnalog.labelGPSAltitude == NULL)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT,LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, -120,0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_color(label,lv_color_white(),0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, "-----");

        singletonConfig()->ui.altimeterAnalog.labelGPSAltitude = label;
    }

    if (parent != NULL && Screen_Altitude_QNH == NULL)
    {
        uint16_t QNH = 1013;
        Screen_Altitude_QNH = lv_label_create(parent);
        lv_obj_set_size(Screen_Altitude_QNH, 150, 48);
        lv_obj_align(Screen_Altitude_QNH, LV_ALIGN_CENTER, 155, 0);
        lv_obj_set_style_text_font(Screen_Altitude_QNH, &lv_font_montserrat_48, 0);
        char buf[6];
        if(singletonConfig()->qnhIsInInches){
            float QNHInInches = QNH / 33.8639;
            snprintf(buf, sizeof(buf), "%.2f", QNHInInches);
        } else {
        snprintf(buf, sizeof(buf), "%03u", QNH);
        }
        lv_label_set_text(Screen_Altitude_QNH, buf);
    }

    if (parent != NULL && Screen_Altitude_Miles == NULL)
    {
        Screen_Altitude_Miles = lv_img_create(parent);
        lv_img_set_src(Screen_Altitude_Miles, &fi_needle_small);
        lv_obj_set_size(Screen_Altitude_Miles, fi_needle_small.header.w, fi_needle_small.header.h);
        lv_obj_align(Screen_Altitude_Miles, LV_ALIGN_CENTER, 0, 0);
    }
    if (parent != NULL && Screen_Altitude_Cents == NULL)
    {
        Screen_Altitude_Cents = lv_img_create(parent);
        lv_img_set_src(Screen_Altitude_Cents, &fi_needle);
        lv_obj_set_size(Screen_Altitude_Cents, fi_needle.header.w, fi_needle.header.h);
        lv_obj_align(Screen_Altitude_Cents, LV_ALIGN_CENTER, 0, 0);
    }
    return NULL;
}

#endif