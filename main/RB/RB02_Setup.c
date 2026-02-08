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
#ifdef RB_ENABLE_SETUP
#include <stdio.h>
#include "nvs_flash.h"

#ifdef RB02_ESP_BLUETOOTH
static void RB02_Setup_Changed_EnableBluetooth(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    singletonConfig()->settingsBluetoothEnabled = 1;
  }
  else
  {
    singletonConfig()->settingsBluetoothEnabled = 0;
  }
  RB02_Config_NVS_Store_BluetoothSettings();
}
static void RB02_Setup_Changed_EnableBluetoothGps(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    singletonConfig()->settingsBluetoothGPS = 1;
  }
  else
  {
    singletonConfig()->settingsBluetoothGPS = 0;
  }
  RB02_Config_NVS_Store_BluetoothSettings();
}
#endif
void nvsStoreFilters();
extern float acosf(float);
extern IMUdata PanelAlignment;

#define RAD2DEG (180.0f / 3.14159265f)
void nvsStoreGyroCalibration();
static void PanelMountPitchChanged(lv_event_t *e)
{
  int32_t value = lv_slider_get_value(lv_event_get_target(e));
  // Stored int 16 bit 32768 -> customers have 39°Pitch on Helicopters
  float equivalentFloat = value / 500.0;

  PanelAlignment.x = equivalentFloat;

  char buf[16 + 8];
  sprintf(buf, "Panel Roll: %.2f", PanelAlignment.x);
  lv_label_set_text(singletonConfig()->ui.panelMountAlignmentLabelPitch, buf);

  nvsStoreGyroCalibration();
}

static void PanelMountRollChanged(lv_event_t *e)
{
  int32_t value = lv_slider_get_value(lv_event_get_target(e));
  // Stored int 16 bit 32768 -> customers have 39°Pitch on Helicopters
  float equivalentFloat = value / 500.0;

  PanelAlignment.z = equivalentFloat;

  char buf[16 + 8];
  sprintf(buf, "Panel Yaw: %.2f", PanelAlignment.z);
  lv_label_set_text(singletonConfig()->ui.panelMountAlignmentLabelRoll, buf);

  nvsStoreGyroCalibration();
}

static void PanelMountYawChanged(lv_event_t *e)
{
  int32_t value = lv_slider_get_value(lv_event_get_target(e));
  // Stored int 16 bit 32768 -> customers have 39°Pitch on Helicopters
  float equivalentFloat = value / 500.0;

  PanelAlignment.y = equivalentFloat;

  char buf[16 + 8];
  sprintf(buf, "Panel Pitch: %.2f", PanelAlignment.y);
  lv_label_set_text(singletonConfig()->ui.panelMountAlignmentLabelYaw, buf);

  nvsStoreGyroCalibration();
}


void nvsStoreQNHInches(uint8_t isInches)
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    err = nvs_set_u8(my_handle, "qnhInches", isInches);
    nvs_close(my_handle);
  }
}


static void QNHIsInInches(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    singletonConfig()->qnhIsInInches = 1;
  }
  else
  {
    singletonConfig()->qnhIsInInches = 0;
  }
    nvsStoreQNHInches(singletonConfig()->qnhIsInInches);
}

void nvsStoreBMPAddress(uint8_t address)
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    err = nvs_set_u8(my_handle, "bmp280Address", address);
    nvs_close(my_handle);
  }
}



static void event_handler_set_bmp_280_76(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    nvsStoreBMPAddress(0x76);
  }
}

static void event_handler_set_bmp_280_77(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    nvsStoreBMPAddress(0x77);
  }
}


static void event_handler_set_panel_alignment(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    if (AccelFiltered.x > 0.99999 || AccelFiltered.x < -0.99999)
    {
      PanelAlignment.x = 0;
    }
    else
    {
      PanelAlignment.x = acosf(AccelFiltered.x) * RAD2DEG;
      if (AccelFiltered.z < 0)
      {
        PanelAlignment.x = -PanelAlignment.x;
      }
    }


    if (AccelFiltered.y > 0.99999 || AccelFiltered.y < -0.99999)
    {
      PanelAlignment.y = 0;
    }
    else
    {
      PanelAlignment.y = acosf(AccelFiltered.y) * RAD2DEG;
    }

    char buf[16 + 8];
    sprintf(buf, "Panel: %.2f %.2f", PanelAlignment.x, PanelAlignment.y);
    lv_label_set_text(singletonConfig()->ui.panelMountAlignmentLabelPitch, buf);

    nvsStoreGyroCalibration();
  }
}

lv_obj_t *RB02_Setup_CreateScreen(RB02_Status *status, lv_obj_t *parent, int *lineY)
{
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, *lineY);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    *lineY += 40;

    status->ui.SettingsOperativeSummary = label;
  }
#ifdef RB02_ESP_BLUETOOTH
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 150, 16);
    lv_obj_align(label, LV_ALIGN_CENTER, -75, *lineY);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_label_set_text(label, "Enable Bluetooth");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_t *sw = lv_switch_create(parent);
    if (status->settingsBluetoothEnabled != 0)
    {
      lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_set_size(sw, 65, 40);
    lv_obj_align(sw, LV_ALIGN_CENTER, 40, *lineY);
    lv_obj_add_event_cb(sw, RB02_Setup_Changed_EnableBluetooth, LV_EVENT_VALUE_CHANGED, sw);
    *lineY += 40;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 150, 16);
    lv_obj_align(label, LV_ALIGN_CENTER, -75, *lineY);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_label_set_text(label, "Use BT GPS");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_t *sw = lv_switch_create(parent);
    if (status->settingsBluetoothGPS != 0)
    {
      lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_set_size(sw, 65, 40);
    lv_obj_align(sw, LV_ALIGN_CENTER, 40, *lineY);
    lv_obj_add_event_cb(sw, RB02_Setup_Changed_EnableBluetoothGps, LV_EVENT_VALUE_CHANGED, sw);
    *lineY += 40;
  }
#endif

  // Attitude Panel Alignment Roll and Pitch #47
  if (false) // Temporary disable auto alignment
  {
    lv_obj_t *btn1 = lv_btn_create(parent);
    lv_obj_add_event_cb(btn1, event_handler_set_panel_alignment, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, *lineY);

    lv_obj_t *label = lv_label_create(btn1);
    lv_label_set_text(label, "SET PANEL ALIGNMENT");
    lv_obj_center(label);
    *lineY += 40;
  }

  
  if (true)
  {
    lv_obj_t *SliderLabel = lv_label_create(parent);
    lv_obj_set_size(SliderLabel, 300, 20);
    lv_obj_align(SliderLabel, LV_ALIGN_CENTER, 0, *lineY);
    lv_obj_set_style_text_align(SliderLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(SliderLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(SliderLabel, lv_color_white(), 0);
    char buf[16 + 8];
    sprintf(buf, "Result");
    lv_label_set_text(SliderLabel, buf);

    singletonConfig()->ui.panelMountAlignmentLabelHelper = SliderLabel;

    *lineY += 32;
  }


  if (true)
  {
    lv_obj_t *Slider = lv_slider_create(parent);
    lv_obj_add_flag(Slider, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(Slider, 300, 35);
    lv_obj_set_style_radius(Slider, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(Slider, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(Slider, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(Slider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(Slider, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(Slider, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(Slider, -32000, 32000);

    int32_t value = PanelAlignment.x * 500.0;

    lv_slider_set_value(Slider, value, LV_ANIM_OFF);
    lv_obj_add_event_cb(Slider, PanelMountPitchChanged, LV_EVENT_VALUE_CHANGED, Slider);
    lv_obj_align(Slider, LV_ALIGN_CENTER, 0, *lineY + 30);

    lv_obj_t *SliderLabel = lv_label_create(parent);
    lv_obj_set_size(SliderLabel, 300, 20);
    lv_obj_align(SliderLabel, LV_ALIGN_CENTER, 0, *lineY);
    lv_obj_set_style_text_align(SliderLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(SliderLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(SliderLabel, lv_color_white(), 0);
    char buf[16 + 8];
    sprintf(buf, "Panel Roll: %.2f", PanelAlignment.x);
    lv_label_set_text(SliderLabel, buf);

    singletonConfig()->ui.panelMountAlignmentLabelPitch = SliderLabel;

    *lineY += 70;
  }


  
  if (true)
  {
    lv_obj_t *Slider = lv_slider_create(parent);
    lv_obj_add_flag(Slider, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(Slider, 300, 35);
    lv_obj_set_style_radius(Slider, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(Slider, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(Slider, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(Slider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(Slider, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(Slider, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(Slider, -32000, 32000);

    int32_t value = PanelAlignment.z * 500.0;

    lv_slider_set_value(Slider, value, LV_ANIM_OFF);
    lv_obj_add_event_cb(Slider, PanelMountRollChanged, LV_EVENT_VALUE_CHANGED, Slider);
    lv_obj_align(Slider, LV_ALIGN_CENTER, 0, *lineY + 30);

    lv_obj_t *SliderLabel = lv_label_create(parent);
    lv_obj_set_size(SliderLabel, 300, 20);
    lv_obj_align(SliderLabel, LV_ALIGN_CENTER, 0, *lineY);
    lv_obj_set_style_text_align(SliderLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(SliderLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(SliderLabel, lv_color_white(), 0);
    char buf[16 + 8];
    sprintf(buf, "Panel Yaw: %.2f", PanelAlignment.z);
    lv_label_set_text(SliderLabel, buf);

    singletonConfig()->ui.panelMountAlignmentLabelRoll = SliderLabel;

    *lineY += 70;
  }



  if (true)
  {
    lv_obj_t *Slider = lv_slider_create(parent);
    lv_obj_add_flag(Slider, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(Slider, 300, 35);
    lv_obj_set_style_radius(Slider, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(Slider, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(Slider, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(Slider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(Slider, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(Slider, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(Slider, -32000, 32000);

    int32_t value = PanelAlignment.y * 500.0;

    lv_slider_set_value(Slider, value, LV_ANIM_OFF);
    lv_obj_add_event_cb(Slider, PanelMountYawChanged, LV_EVENT_VALUE_CHANGED, Slider);
    lv_obj_align(Slider, LV_ALIGN_CENTER, 0, *lineY + 30);

    lv_obj_t *SliderLabel = lv_label_create(parent);
    lv_obj_set_size(SliderLabel, 300, 20);
    lv_obj_align(SliderLabel, LV_ALIGN_CENTER, 0, *lineY);
    lv_obj_set_style_text_align(SliderLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(SliderLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(SliderLabel, lv_color_white(), 0);
    char buf[16 + 8];
    sprintf(buf, "Panel Pitch: %.2f", PanelAlignment.y);
    lv_label_set_text(SliderLabel, buf);

    singletonConfig()->ui.panelMountAlignmentLabelYaw = SliderLabel;

    *lineY += 70;
  }


  if (true)
  {
    lv_obj_t *btn1 = lv_btn_create(parent);
    lv_obj_add_event_cb(btn1, event_handler_set_bmp_280_76, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, *lineY);

    lv_obj_t *label = lv_label_create(btn1);
    lv_label_set_text(label, "SET BMP280 ADDRESS 0x76");
    lv_obj_center(label);
    *lineY += 60;
  }

  if (true)
  {
    lv_obj_t *btn1 = lv_btn_create(parent);
    lv_obj_add_event_cb(btn1, event_handler_set_bmp_280_77, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, *lineY);

    lv_obj_t *label = lv_label_create(btn1);
    lv_label_set_text(label, "SET BMP280 ADDRESS 0x77");
    lv_obj_center(label);
    *lineY += 60;
  }

  if (true)
  {
    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 150, 16);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, -75, *lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(DisableFilteringLabel, "QNH InHg");
    lv_obj_t *sw = lv_switch_create(parent);
    if (singletonConfig()->qnhIsInInches != 0)
    {
      lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_set_size(sw, 65, 40);
    lv_obj_align(sw, LV_ALIGN_CENTER, 40, *lineY);
    lv_obj_add_event_cb(sw, QNHIsInInches, LV_EVENT_VALUE_CHANGED, sw);
    *lineY += 40;
  }

  return NULL;
}
#endif