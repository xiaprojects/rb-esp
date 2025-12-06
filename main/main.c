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

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "TCA9554PWR.h"
#include "PCF85063.h"
#include "QMI8658.h"
#include "ST7701S.h"
#include "RB02.h"
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_28
// ESP32-S3-2.8C
#ifdef RB_02_DISPLAY_TOUCH
// TODO: we shall remove the compilation of the C file
#include "GT911.h"
#endif
#endif
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_21
// ESP32-S3-2.1C
#include "CST820.h"
#endif
#include "SD_MMC.h"
#include "LVGL_Driver.h"
#include "lvgl.h"
#include "demos/lv_demos.h"
#include "Buzzer.h"
#include "BAT_Driver.h"
#include "nvs_flash.h"
#include "BAT_Driver.h"
#include "driver/uart.h"
#ifdef RB02_ESP_BLUETOOTH
#include "RB05_ESP_Bluetooth.h"
#endif

// 1.1.9
uint8_t DriverLoopMilliseconds = 40;

// 1.0.9 Install in the loop BMP280
void Get_BMP280(void);   // Declaration
extern uint8_t workflow; // When sensor is ready (After Calibration)
void Driver_Loop(void *parameter)
{
    int loopThreshold = 10; // Delay the polling of certain sensors
    while (1)
    {
        QMI8658_Loop();
        // Delay the polling of certain sensors
        if (loopThreshold == 0)
        {
            RTC_Loop();
            // 1.1.17 Improving BMP Read, by default the loop is 20Hz and read at 1Hz
            loopThreshold = 1000 / (10 + DriverLoopMilliseconds);
            // When sensor is ready (After Calibration)
            if (workflow > 100)
            {
                BAT_Get_Volts();
                Get_BMP280();
            }
        }
        loopThreshold--;
        vTaskDelay(pdMS_TO_TICKS(10 + DriverLoopMilliseconds));
    }
    vTaskDelete(NULL);
}

uint8_t RB02_Config_NVS_Get_BluetoothSettings();
#ifdef RB_DISPLAY_DEBUG
// Due to Black Screen problem we are going to investigate the I2C: the PWM may not be ready yet. No issues found, we keep this code as debugging procedure.
void i2c_scan(void) {
    static const char *TAG = "i2c";
    ESP_LOGI(TAG, "I2C scan start");
    i2c_cmd_handle_t cmd;
    esp_err_t ret;
    for (uint8_t addr = 1; addr < 127; addr++) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "I2C scan done");
}
#endif

void Driver_Init(void)
{
    Flash_Searching();
    BAT_Init();
    I2C_Init();
#ifdef RB_DISPLAY_DEBUG
    i2c_scan();
    vTaskDelay(pdMS_TO_TICKS(100));
#endif
    PCF85063_Init();
    QMI8658_Init();
    EXIO_Init(); // Example Initialize EXIO
/*
    // Keep the LCD under reset after Expander has being init
    vTaskDelay(pdMS_TO_TICKS(100));
    // Black screen issue keep under reset the panel
    Set_EXIO(TCA9554_EXIO1, false);
    Set_EXIO(TCA9554_EXIO2, false);
    vTaskDelay(pdMS_TO_TICKS(1000));
*/
#ifdef RB02_ESP_BLUETOOTH
    //if (RB02_Config_NVS_Get_BluetoothSettings() != 0)
    {
        RB02_BluetoothLowEnergy_Init();
    }
#endif

    xTaskCreatePinnedToCore(
        Driver_Loop,
        "Other Driver task",
        4096,
        NULL,
        3,
        NULL,
        0);
}
void RB02_Main();
void Set_Backlight(uint8_t Light);
void app_main(void)
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    Driver_Init();
    
#ifdef RB_ENABLE_UART
    // 1.0.9
    // Install the UART Driver as soon as possible
    uart_driver_install(UART_N, 256, 0, 0, NULL, 0);
    // Set PIN for Waveshare 2.8" Round based on Wiki Schematics
    uart_set_pin(1, UART_PIN_NO_CHANGE, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#endif

    LCD_Init();
// 1.1.23 Add non touch unified official support
#ifdef RB_02_DISPLAY_TOUCH
    Touch_Init();
#endif
    SD_Init();
    LVGL_Init(panel_handle, tp);
    /********************* Demo *********************/
    if (LVGL_lock(-1)) {

#ifdef RB_DISPLAY_DEBUG
    const int DisplayDebugTrue = true;
#else
    const int DisplayDebugTrue = false;
#endif
    if (DisplayDebugTrue) // Display test minimal routine
    {

        Set_Backlight(100);
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(255, 0, 0), 0);
    }
    else
    {
        RB02_Main();
    }

#ifdef RB_DISPLAY_DEBUG
    i2c_scan();
    vTaskDelay(pdMS_TO_TICKS(100));
#endif
        LVGL_unlock();
    }
}
