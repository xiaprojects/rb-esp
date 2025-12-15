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

#include "RB02_NavCore.h"

#ifdef RB_ENABLE_NavCore
int32_t NavCore_GetEncoderDelta(void);
bool NavCore_GetAndClearSwitchPressed(void);
#endif






#ifdef RB_ENABLE_NavCore
// ---- NavCore: Enable the Air Speed Sensor.
#include "ms4525do_rb.h"



// TODO Move into the Singleton
SC16IS750_t g_navcore_sc;
bool g_navcore_sc_inited = false;

TaskHandle_t g_navcore_task = NULL;
StreamBufferHandle_t g_navcore_gps_sb = NULL;
StreamBufferHandle_t g_navcore_rs485_sb = NULL;

volatile int32_t g_navcore_ec11_delta = 0;
volatile bool    g_navcore_ec11_sw_pressed = false;

// last sampled EC11 pins (2-bit quad state)
uint8_t g_navcore_ec11_last_ab = 0;
uint8_t g_navcore_ec11_last_sw = 1;

static void IRAM_ATTR navcore_sc16_irq_isr(void *arg)
{
  (void)arg;
  if (g_navcore_task) {
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(g_navcore_task, &hp);
    if (hp) portYIELD_FROM_ISR();
  }
}
#endif


#ifdef RB_ENABLE_GPS
#ifdef RB_ENABLE_NavCore

// Decode EC11 quadrature. Table based on 4-bit transition (prev<<2 | curr)
static inline int8_t navcore_ec11_step(uint8_t prev_ab, uint8_t curr_ab)
{
  static const int8_t table[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
  };
  return table[((prev_ab & 0x3) << 2) | (curr_ab & 0x3)];
}

static void navcore_update_ec11(void)
{
  uint8_t clk = SC16IS750_GPIOGetPinState(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_CLK) ? 1 : 0;
  uint8_t dt  = SC16IS750_GPIOGetPinState(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_DT) ? 1 : 0;
  uint8_t sw  = SC16IS750_GPIOGetPinState(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_SW) ? 1 : 0;

  uint8_t ab = (clk<<1) | dt;
  int8_t step = navcore_ec11_step(g_navcore_ec11_last_ab, ab);
  if (step != 0) {
    g_navcore_ec11_delta += step;
    g_navcore_ec11_last_ab = ab;
  } else {
    g_navcore_ec11_last_ab = ab;
  }

  // Switch: detect falling edge (assuming pull-up => pressed = 0)
  if (g_navcore_ec11_last_sw == 1 && sw == 0) {
    g_navcore_ec11_sw_pressed = true;
  }
  g_navcore_ec11_last_sw = sw;
}

int32_t NavCore_GetEncoderDelta(void)
{
  int32_t v = g_navcore_ec11_delta;
  g_navcore_ec11_delta = 0;
  return v;
}

bool NavCore_GetAndClearSwitchPressed(void)
{
  bool v = g_navcore_ec11_sw_pressed;
  g_navcore_ec11_sw_pressed = false;
  return v;
}

// Dedicated IRQ-driven task: drains SC16IS interrupts and fills stream buffers.
// No LVGL / UI calls here.
static void NavCore_SC16_Task(void *arg)
{
  (void)arg;
  uint8_t buf[128];

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Drain pending interrupts on both channels
    while (SC16IS750_InterruptPendingTest(&g_navcore_sc, RB_NAVCORE_GPS_CH) == 0 ||
           SC16IS750_InterruptPendingTest(&g_navcore_sc, RB_NAVCORE_RS485_CH) == 0)
    {
      int16_t evA = SC16IS750_InterruptEventTest(&g_navcore_sc, RB_NAVCORE_GPS_CH);
      int16_t evB = SC16IS750_InterruptEventTest(&g_navcore_sc, RB_NAVCORE_RS485_CH);

      // UART A (GPS): RX or timeout => drain FIFO
      if (evA == SC16IS750_RHR_INTERRUPT || evA == SC16IS750_RECEIVE_TIMEOUT_INTERRUPT)
      {
        size_t n = 0;
        while (n < sizeof(buf))
        {
          uint8_t avail = SC16IS750_FIFOAvailableData(&g_navcore_sc, RB_NAVCORE_GPS_CH);
          if (avail == 0) break;
          int c = SC16IS750_ReadByte(&g_navcore_sc, RB_NAVCORE_GPS_CH);
          if (c < 0) break;
          buf[n++] = (uint8_t)c;
        }
        if (n && g_navcore_gps_sb) {
          xStreamBufferSend(g_navcore_gps_sb, buf, n, 0);
        }
      }

      // UART B (RS485): RX or timeout => drain FIFO into RS485 stream buffer
      if (evB == SC16IS750_RHR_INTERRUPT || evB == SC16IS750_RECEIVE_TIMEOUT_INTERRUPT)
      {
        size_t n = 0;
        while (n < sizeof(buf))
        {
          uint8_t avail = SC16IS750_FIFOAvailableData(&g_navcore_sc, RB_NAVCORE_RS485_CH);
          if (avail == 0) break;
          int c = SC16IS750_ReadByte(&g_navcore_sc, RB_NAVCORE_RS485_CH);
          if (c < 0) break;
          buf[n++] = (uint8_t)c;
        }
        if (n && g_navcore_rs485_sb) {
          xStreamBufferSend(g_navcore_rs485_sb, buf, n, 0);
        }
      }

      // GPIO change on A or B: update EC11
      if (evA == SC16IS750_INPUT_PIN_CHANGE_STATE || evB == SC16IS750_INPUT_PIN_CHANGE_STATE)
      {
        navcore_update_ec11();
      }
    }
  }
}


void NavCore_SC16_Init() {
   // Create stream buffers once (GPS NMEA bursts + RS485 frames)
    g_navcore_gps_sb = xStreamBufferCreate(UART_RX_BUF_SIZE * 4, 1);
    g_navcore_rs485_sb = xStreamBufferCreate(256, 1);

    // Init SC16IS: pass 7-bit address (0x4D). The driver uses I2C_Driver.c for transfers.
    SC16IS750_init(&g_navcore_sc, SC16IS750_PROTOCOL_I2C, RB_NAVCORE_SC16IS_ADDR7, 2);

    // Configure UART lines (8N1)
    SC16IS750_SetLine(&g_navcore_sc, RB_NAVCORE_GPS_CH, 8, 0, 1);
    SC16IS750_SetLine(&g_navcore_sc, RB_NAVCORE_RS485_CH, 8, 0, 1);

    // Baudrates: GPS=9600, RS485=9600 with crystal 1.8432MHz
    SC16IS750_begin(&g_navcore_sc, 9600, 9600, 1843200L);

    // FIFO enable + reset
    SC16IS750_FIFOEnable(&g_navcore_sc, RB_NAVCORE_GPS_CH, 1);
    SC16IS750_FIFOReset(&g_navcore_sc, RB_NAVCORE_GPS_CH, 1);

    SC16IS750_FIFOEnable(&g_navcore_sc, RB_NAVCORE_RS485_CH, 1);
    SC16IS750_FIFOReset(&g_navcore_sc, RB_NAVCORE_RS485_CH, 1);

    // FIFO trigger level (TLR-based): 8 bytes for both channels
    SC16IS750_FIFOSetTriggerLevel(&g_navcore_sc, RB_NAVCORE_GPS_CH, 1, 8);
    SC16IS750_FIFOSetTriggerLevel(&g_navcore_sc, RB_NAVCORE_RS485_CH, 1, 8);

    // Enable RX interrupt (Receive timeout events will be reported in IIR when FIFO is enabled)
    SC16IS750_InterruptControl(&g_navcore_sc, RB_NAVCORE_GPS_CH, SC16IS750_INT_RHR);
    SC16IS750_InterruptControl(&g_navcore_sc, RB_NAVCORE_RS485_CH, SC16IS750_INT_RHR);

    // RS485 on channel B using RTSB to drive DE/RE (connect RTSB -> DE/RE)
    SC16IS750_EnableRs485(&g_navcore_sc, RB_NAVCORE_RS485_CH, NO_INVERT_RTS_SIGNAL);

    // EC11 on GPIO0/1/2 of SC16
    SC16IS750_GPIOSetPinMode(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_CLK, 0);
    SC16IS750_GPIOSetPinMode(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_DT, 0);
    SC16IS750_GPIOSetPinMode(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_SW, 0);
    // Enable GPIO change interrupt for GPIO0..2
    SC16IS750_SetPinInterrupt(&g_navcore_sc, 0x07);

    // Create dedicated task and IRQ ISR
    xTaskCreate(NavCore_SC16_Task, "navcore_sc16", 4096, NULL, 10, &g_navcore_task);

    gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << RB_NAVCORE_SC16_IRQ_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
      ESP_ERROR_CHECK(isr_err);
    }
    gpio_isr_handler_add(RB_NAVCORE_SC16_IRQ_GPIO, navcore_sc16_irq_isr, NULL);

    // Prime EC11 state
    uint8_t clk = SC16IS750_GPIOGetPinState(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_CLK) ? 1 : 0;
    uint8_t dt  = SC16IS750_GPIOGetPinState(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_DT) ? 1 : 0;
    uint8_t sw  = SC16IS750_GPIOGetPinState(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_SW) ? 1 : 0;
    g_navcore_ec11_last_ab = (clk<<1) | dt;
    g_navcore_ec11_last_sw = sw;

    g_navcore_sc_inited = true;
}

#endif // RB_ENABLE_NavCore
#endif