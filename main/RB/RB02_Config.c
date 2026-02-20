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

#include "RB02_Config.h"
#include "nvs_flash.h"
#include <lvgl.h>

RB02_Status *rb02Status = NULL;

#if LVGL_VERSION_MAJOR >=9
typedef lv_mem_alloc lv_malloc
#endif


RB02_Status *singletonConfig()
{
    if (rb02Status == NULL)
    {
        rb02Status = lv_mem_alloc(sizeof(RB02_Status));
        if (rb02Status != NULL)
        {
            lv_memset(rb02Status, 0, sizeof(RB02_Status));
        }
    }
    return rb02Status;
}

#ifdef RB02_ESP_BLUETOOTH

void RB02_Config_Set_OperativeBluetooth(uint8_t operative)
{
    singletonConfig()->Operative_Bluetooth = operative;
}

uint8_t RB02_Config_NVS_Get_BluetoothSettings()
{
    uint8_t ret = 1;
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_STORAGE, NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        ret = 0;
    }
    else
    {
        err = nvs_get_u8(my_handle, NVS_KEY_BT_ENABLE, &ret);
        switch (err)
        {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            break;
        default:
        }
        nvs_close(my_handle);
    }
    return ret;
}

uint8_t RB02_Config_NVS_Store_BluetoothSettings()
{

    uint8_t ret = 0;
    //
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_STORAGE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        return ret;
    }
    else
    {
        // Read
        err = nvs_set_u8(my_handle, NVS_KEY_BT_ENABLE, singletonConfig()->settingsBluetoothEnabled);
        if (err != ESP_OK)
        {
            ret = 0;
        }
        else
        {
            ret = 1;
        }
        nvs_set_u8(my_handle, NVS_KEY_BT_GPS, singletonConfig()->settingsBluetoothGPS);
        nvs_close(my_handle);
    }

    return ret;
}
#endif