
/**
 * This file is part of RB.
 *
 * Copyright (C) 2025 XIAPROJECTS SRL
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
 * 06 -> Display with Android 6.25" 7" 8" 10" 10.2"
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/
#pragma once
#include "RB02_Config.h"

#ifdef RB_ENABLE_NavCore
#include "sc16is750.h"
#include "driver/gpio.h"
#include "freertos/stream_buffer.h"
#endif

#ifdef RB_ENABLE_NavCore
// ---- NavCore: SC16IS762/752 via existing I2C bus (GPIO07/15) + IRQ on GPIO19 ----
#define RB_NAVCORE_SC16IS_ADDR7          0x4D
#define RB_NAVCORE_GPS_CH                SC16IS750_CHANNEL_A
#define RB_NAVCORE_RS485_CH              SC16IS750_CHANNEL_B
#define RB_NAVCORE_SC16_IRQ_GPIO         GPIO_NUM_19

// EC11 wiring on SC16 GPIOs
#define RB_NAVCORE_EC11_GPIO_CLK         0
#define RB_NAVCORE_EC11_GPIO_DT          1
#define RB_NAVCORE_EC11_GPIO_SW          2


extern SC16IS750_t g_navcore_sc;
extern bool g_navcore_sc_inited;

extern TaskHandle_t g_navcore_task;
extern StreamBufferHandle_t g_navcore_gps_sb;
extern StreamBufferHandle_t g_navcore_rs485_sb;

extern volatile int32_t g_navcore_ec11_delta;
extern volatile bool    g_navcore_ec11_sw_pressed;

// last sampled EC11 pins (2-bit quad state)
extern uint8_t g_navcore_ec11_last_ab;
extern uint8_t g_navcore_ec11_last_sw;


#endif
#ifdef RB_ENABLE_NavCore
static void NavCore_SC16_Task(void *arg);
int32_t NavCore_GetEncoderDelta(void);
bool NavCore_GetAndClearSwitchPressed(void);
static void IRAM_ATTR navcore_sc16_irq_isr(void *arg);
void NavCore_SC16_Init();
#endif