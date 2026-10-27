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
#include "RB02_Workflow.h"
#include "lvgl.h"
#include "Buzzer.h"

void RB02_Main()
{
    // initialise the Centralised configuration
    RB02_Status *status = singletonConfig();
    RB02_MainWithConfig(status);
}

extern lv_obj_t *tv;
extern uint8_t workflow;

lv_obj_t *RB02_Main_CreateLoadingSlider(lv_obj_t *parent)
{
    //
    lv_obj_t *loadingSlider = lv_slider_create(parent);

    lv_obj_set_size(loadingSlider, 240, 15);
    lv_obj_set_style_radius(loadingSlider, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(loadingSlider, LV_OPA_TRANSP, LV_PART_KNOB);
    // lv_obj_set_style_pad_all(Backlight_slider, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_color(loadingSlider, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(loadingSlider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(loadingSlider, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(loadingSlider, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(loadingSlider, 0, 100);
    lv_slider_set_value(loadingSlider, workflow, LV_ANIM_ON);
    lv_obj_align(loadingSlider, LV_ALIGN_CENTER, 0, 10);

    return loadingSlider;
}
void rb_increase_lvgl_tick(lv_timer_t *t);
void Set_Backlight(uint8_t Light);
void RB02_MainWithConfig(RB02_Status *status)
{
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_STATE_DEFAULT);
    tv = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 0);
    lv_obj_set_style_bg_color(tv, lv_color_black(), LV_STATE_DEFAULT);
    status->ui.Loading_slider = RB02_Main_CreateLoadingSlider(lv_scr_act());
    lv_timer_t *auto_step_timer = lv_timer_create(rb_increase_lvgl_tick, 100, NULL);
    if (auto_step_timer == NULL)
    {
    }
}
