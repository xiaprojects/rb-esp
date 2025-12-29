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


/*
 * @file RB02_NavCore.c
 * Author: The_MiNuS (2025)
*/

#include "RB02_NavCore.h"
#ifdef RB_ENABLE_NavCore
#include "RB02.h"


#include <string.h>


navcore_ec11_t g_navcore_ec11;

// ---- Ordered event queue (ring) ----
static inline void ec11_q_push(navcore_ec11_t *e, uint32_t evt)
{
  uint16_t next = (uint16_t)(e->q_wr + 1u);
  if ((uint16_t)(next - e->q_rd) > (uint16_t)NAVCORE_EC11_QUEUE_LEN) {
    // drop oldest to keep most recent
    e->q_rd++;
    e->q_overflow++;
  }
  e->q[e->q_wr % NAVCORE_EC11_QUEUE_LEN] = (uint16_t)evt;
  e->q_wr = next;
}

bool navcore_ec11_pop_event(navcore_ec11_t *e, uint32_t *evt)
{
  if (e->q_rd == e->q_wr) return false;
  uint16_t v = e->q[e->q_rd % NAVCORE_EC11_QUEUE_LEN];
  e->q_rd++;
  if (evt) *evt = (uint32_t)v;
  return true;
}

// ---- EC11 state (legacy compatibility) ----
int32_t  g_navcore_ec11_delta = 0;

bool     g_navcore_ec11_sw_short_pressed = false;
bool     g_navcore_ec11_sw_long_pressed  = false;

// ---- SC16 / streams ----
static SC16IS750_t g_navcore_sc;
static TaskHandle_t g_navcore_task = NULL;

static StreamBufferHandle_t g_navcore_gps_sb   = NULL;
static StreamBufferHandle_t g_navcore_rs485_sb = NULL;

static uint8_t g_sc16_last_gpio_bits = 0xFF;

// --------------------
// Helpers
// --------------------
static inline uint32_t navcore_now_ms(void)
{
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

StreamBufferHandle_t rb_navcore_gps_stream(void)   { return g_navcore_gps_sb; }
StreamBufferHandle_t rb_navcore_rs485_stream(void) { return g_navcore_rs485_sb; }

// Read entire SC16 GPIO port in ONE I2C operation
static inline uint8_t navcore_sc16_read_gpio_bits(void)
{
  return SC16IS750_GPIOGetPortState(&g_navcore_sc);
}

// --------------------
// Legacy public getters
// --------------------
int32_t NavCore_GetEncoderDelta(void)
{
  int32_t d = g_navcore_ec11_delta;
  g_navcore_ec11_delta = 0;
  return d;
}

bool NavCore_GetAndClearSwitchShortPressed(void)
{
  bool v = g_navcore_ec11_sw_short_pressed;
  g_navcore_ec11_sw_short_pressed = false;
  return v;
}

bool NavCore_GetAndClearSwitchLongPressed(void)
{
  bool v = g_navcore_ec11_sw_long_pressed;
  g_navcore_ec11_sw_long_pressed = false;
  return v;
}

// --------------------
// Decoder init
// --------------------
void navcore_ec11_init(navcore_ec11_t *e)
{
    memset(e, 0, sizeof(*e));
    e->last_ab = 0;
    e->last_sw = 1; /* pull-up idle */
    e->last_clk = 0;
    e->accum = 0;
    e->last_step_dir = 0;
    e->last_detent_ms = 0;
    e->fast_mode = false;
    e->press_start_ms = 0;
    e->sw_is_down = false;
    e->rotation_while_pressed = false;
    e->last_clk_edge_ms = 0;
    e->last_sw_edge_ms = 0;

    e->q_wr = 0;
    e->q_rd = 0;
    e->q_overflow = 0;

    // Multi short-press aggregation (delayed single)
    e->short_press_seq = 0;          // 0=no pending, 1=pending single, 2=pending double
    e->last_short_release_ms = 0;    // deadline (now + window)
}



// --------------------
// Legacy bitmask get/clear
// --------------------
uint32_t navcore_ec11_get_and_clear(navcore_ec11_t *e)
{
  uint32_t ev = e->events;
  e->events = 0;
  return ev;
}


// --------------------
// Multi Press helper.
// --------------------
static inline void navcore_ec11_flush_multi_short_press(navcore_ec11_t *e, uint32_t now_ms)
{
    if (e->short_press_seq == 0) return;

    // Deadline reached -> emit final aggregated event
    if (e->last_short_release_ms != 0u && (uint32_t)(now_ms - e->last_short_release_ms) < 0x80000000u) {
        // now_ms >= deadline (unsigned safe check via wrap)
        if (now_ms < e->last_short_release_ms) return;
    } else {
        // no deadline => nothing
        return;
    }

    if (e->short_press_seq == 1)
    {
        e->events |= NAVCORE_EC11_EVT_PRESS;
        ec11_q_push(e, NAVCORE_EC11_EVT_PRESS);
        g_navcore_ec11_sw_short_pressed = true;
    }
    else if (e->short_press_seq == 2)
    {
        e->events |= NAVCORE_EC11_EVT_DOUBLE_PRESS;
        ec11_q_push(e, NAVCORE_EC11_EVT_DOUBLE_PRESS);
    }

    // Clear pending state
    e->short_press_seq = 0;
    e->last_short_release_ms = 0;
}

// --------------------
// CLK-edge-only update
// - called when SC16 GPIO input change IRQ triggers and CLK or SW changed.
// - only processes rotation on CLK edge; DT is sampled at that moment.
// --------------------
void navcore_ec11_update(navcore_ec11_t *e, uint8_t a, uint8_t b, uint8_t sw, uint32_t now_ms)
{
    const uint8_t new_sw = (uint8_t)(sw & 1u);
    const uint8_t old_sw = (uint8_t)(e->last_sw & 1u);

    const uint8_t new_clk = (uint8_t)(a & 1u);
    const uint8_t old_clk = (uint8_t)(e->last_clk & 1u);

    // ----------------
    // Switch handling (press/release with debounce)
    // ----------------
    if (new_sw != old_sw)
    {
        if (e->last_sw_edge_ms == 0u || (uint32_t)(now_ms - e->last_sw_edge_ms) >= (uint32_t)NAVCORE_EC11_SW_DEBOUNCE_MS)
        {
            e->last_sw_edge_ms = now_ms;

            // press: 1 -> 0
            if (old_sw == 1 && new_sw == 0)
            {
                e->press_start_ms = now_ms;
                e->sw_is_down = true;
                e->rotation_while_pressed = false;
                e->long_press_fired = false;
            }
            // release: 0 -> 1
            else if (old_sw == 0 && new_sw == 1)
            {
                e->sw_is_down = false;

                uint32_t held = 0;
                if (e->press_start_ms != 0u) {
                    held = (uint32_t)(now_ms - e->press_start_ms);
                }
                e->press_start_ms = 0;

                // ---------------------------------------------------------
                // IMPORTANT: If rotation occurred during this press,
                // DO NOT emit any short/long press (and cancel any pending multi-press).
                // ---------------------------------------------------------
                if (e->rotation_while_pressed)
                {
                    // Cancel any pending aggregated short press sequence
                    e->short_press_seq = 0;
                    e->last_short_release_ms = 0;

                    // End of press cycle
                    e->rotation_while_pressed = false;
                    goto save_and_out;
                }

                // No rotation: normal press logic
                if (held >= (uint32_t)EC11_LONG_PRESS_MS)
                {
                    // Long press cancels any pending short aggregation
                    e->short_press_seq = 0;
                    e->last_short_release_ms = 0;

                    e->events |= NAVCORE_EC11_EVT_LONG_PRESS;
                    ec11_q_push(e, NAVCORE_EC11_EVT_LONG_PRESS);
                    g_navcore_ec11_sw_long_pressed = true;
                }
                else
                {
                    // SHORT press: aggregate single/double/triple
                    // If pending expired, flush it now before starting new sequence
                    if (e->short_press_seq != 0 && e->last_short_release_ms != 0u && now_ms >= e->last_short_release_ms)
                    {
                        navcore_ec11_flush_multi_short_press(e, now_ms);
                    }

                    if (e->short_press_seq == 0)
                    {
                        // Start new sequence (pending single)
                        e->short_press_seq = 1;
                        e->last_short_release_ms = (uint32_t)(now_ms + NAVCORE_EC11_MULTI_PRESS_WINDOW_MS);
                    }
                    else
                    {
                        // Within window: extend sequence
                        e->short_press_seq++;
                        e->last_short_release_ms = (uint32_t)(now_ms + NAVCORE_EC11_MULTI_PRESS_WINDOW_MS);

                        if (e->short_press_seq >= 3)
                        {
                            // Emit triple immediately and clear
                            e->events |= NAVCORE_EC11_EVT_TRIPLE_PRESS;
                            ec11_q_push(e, NAVCORE_EC11_EVT_TRIPLE_PRESS);

                            e->short_press_seq = 0;
                            e->last_short_release_ms = 0;
                        }
                    }
                }

                // End of press cycle
                e->rotation_while_pressed = false;
            }
        }
    }

    // ----------------
    // CLK-edge-only rotation decode (unchanged)
    // ----------------
    if (new_clk != old_clk)
    {
        if (e->last_clk_edge_ms == 0u || (uint32_t)(now_ms - e->last_clk_edge_ms) >= (uint32_t)NAVCORE_EC11_CLK_DEBOUNCE_MS)
        {
            e->last_clk_edge_ms = now_ms;

            // Count only the selected CLK edge
            if (new_clk != (uint8_t)EC11_COUNT_ON_CLK_LEVEL)
            {
                goto save_and_out;
            }

            // step = +1 if (CLK XOR DT) else -1
            int8_t step = ((new_clk ^ (uint8_t)(b & 1u)) ? +1 : -1);

#if (NAVCORE_EC11_DIR_INVERT)
            step = (int8_t)(-step);
#endif

            e->accum = (int8_t)(e->accum + step);
            e->last_step_dir = step;

            if (e->accum >= (int8_t)EC11_DETENT_TRANSITIONS || e->accum <= (int8_t)(-EC11_DETENT_TRANSITIONS))
            {
                int8_t dir = (e->accum > 0) ? +1 : -1;
                e->accum = 0;

                // FAST hysteresis
                if (e->last_detent_ms != 0u)
                {
                    uint32_t dt_ms = (uint32_t)(now_ms - e->last_detent_ms);
                    if (dt_ms <= (uint32_t)NAVCORE_EC11_FAST_ENTER_MS) e->fast_mode = true;
                    else if (dt_ms >= (uint32_t)NAVCORE_EC11_FAST_EXIT_MS) e->fast_mode = false;
                }
                e->last_detent_ms = now_ms;

                // Legacy delta
                g_navcore_ec11_delta += (int32_t)dir;

                // If rotation while pressed, prevent short/long on release
                if (e->sw_is_down) e->rotation_while_pressed = true;

                uint32_t evt = 0;
                if (!e->sw_is_down)
                {
                    if (dir > 0) evt = (e->fast_mode ? NAVCORE_EC11_EVT_FAST_CW : NAVCORE_EC11_EVT_CW);
                    else         evt = (e->fast_mode ? NAVCORE_EC11_EVT_FAST_CCW : NAVCORE_EC11_EVT_CCW);
                }
                else
                {
                    if (dir > 0) evt = (e->fast_mode ? NAVCORE_EC11_EVT_PRESS_FAST_CW : NAVCORE_EC11_EVT_PRESS_CW);
                    else         evt = (e->fast_mode ? NAVCORE_EC11_EVT_PRESS_FAST_CCW : NAVCORE_EC11_EVT_PRESS_CCW);
                }

                if (evt)
                {
                    e->events |= evt;
                    ec11_q_push(e, evt);
                }
            }
        }
    }

save_and_out:
    // Save last levels for edge detection
    e->last_sw  = new_sw;
    e->last_clk = new_clk;
    e->last_ab  = (uint8_t)(((new_clk & 1u) << 1) | ((b & 1u)));
}


#ifdef RB02_BUTTON_KNOB
// --------------------
// Console/UI polling task
// (already exists in your file; now drains the queue and prints the 10 events)
// --------------------
TaskHandle_t g_navcore_ui_task = NULL;

void NavCore_UI_Task(void *arg)
{
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(20);
  for (;;) {
    vTaskDelay(period);
    navcore_ui_poll_ec11();
  }
}

// --------------------
// Existing UI/poll handler
// - Now: drains the queue (ordered) and prints events
// - Then: keeps your existing "bitmask based" handling (compat)
// --------------------
void navcore_ui_poll_ec11(void)
{
  // Flush pending single/double when the window expires
  uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
  navcore_ec11_flush_multi_short_press(&g_navcore_ec11, now_ms);

  uint32_t ev = 0;
  uint32_t one = 0;

  while (navcore_ec11_pop_event(&g_navcore_ec11, &one)) {
    ev |= one;

  #if NAVCORE_EC11_DEBUG
    if (one & NAVCORE_EC11_EVT_CW)                printf("[EC11] CW\n");
    if (one & NAVCORE_EC11_EVT_CCW)               printf("[EC11] CCW\n");
    if (one & NAVCORE_EC11_EVT_FAST_CW)           printf("[EC11] Fast CW\n");
    if (one & NAVCORE_EC11_EVT_FAST_CCW)          printf("[EC11] Fast CCW\n");

    if (one & NAVCORE_EC11_EVT_PRESS_CW)          printf("[EC11] CW while Press\n");
    if (one & NAVCORE_EC11_EVT_PRESS_CCW)         printf("[EC11] CCW while Press\n");
    if (one & NAVCORE_EC11_EVT_PRESS_FAST_CW)     printf("[EC11] Fast CW while Press\n");
    if (one & NAVCORE_EC11_EVT_PRESS_FAST_CCW)    printf("[EC11] Fast CCW while Press\n");

    if (one & NAVCORE_EC11_EVT_PRESS)             printf("[EC11] Short press\n");
    if (one & NAVCORE_EC11_EVT_LONG_PRESS)        printf("[EC11] Long press (>= %ums)\n", (unsigned)EC11_LONG_PRESS_MS);

    if (one & NAVCORE_EC11_EVT_DOUBLE_PRESS)      printf("[EC11] Double short press\n");
    if (one & NAVCORE_EC11_EVT_TRIPLE_PRESS)      printf("[EC11] Triple short press\n");
  #endif

    // ----------------------------
    // EC11 -> RB02 logical touch mapping
    // Nord  = CW
    // Sud   = CCW
    // Est   = Double press
    // Ouest = Triple press
    // ----------------------------
    if (one & (NAVCORE_EC11_EVT_CW | NAVCORE_EC11_EVT_FAST_CW))
    {
      RB02_NavCore_InjectTouch(RB02_TOUCHLOC_N);
    }

    if (one & (NAVCORE_EC11_EVT_CCW | NAVCORE_EC11_EVT_FAST_CCW))
    {
      RB02_NavCore_InjectTouch(RB02_TOUCHLOC_S);
    }

    if (one & NAVCORE_EC11_EVT_DOUBLE_PRESS)
    {
      RB02_NavCore_InjectTouch(RB02_TOUCHLOC_E);
    }

    if (one & NAVCORE_EC11_EVT_TRIPLE_PRESS)
    {
      RB02_NavCore_InjectTouch(RB02_TOUCHLOC_W);
    }
  
  }

  // Clear legacy bitmask (queue already drained)
  (void)navcore_ec11_get_and_clear(&g_navcore_ec11);

  if (ev == 0) return;
}
#endif
// --------------------
// SC16 IRQ ISR + SC16 worker task
// - Inchangé sur le fond: UART A + UART B drainés comme avant
// - EC11: on ne traite que CLK/SW, et on sample DT sur edge CLK
// --------------------
static void IRAM_ATTR navcore_sc16_irq_isr(void *arg)
{
  (void)arg;
  BaseType_t hp_task_woken = pdFALSE;
  if (g_navcore_task) {
    vTaskNotifyGiveFromISR(g_navcore_task, &hp_task_woken);
  }
  if (hp_task_woken) {
    portYIELD_FROM_ISR();
  }
}

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

      // UART A (GPS): RX or timeout => drain FIFO into GPS stream buffer
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

      // GPIO input change (EC11): on ne traite que CLK & SW (DT sampled on CLK edge)
      if (evA == SC16IS750_INPUT_PIN_CHANGE_STATE || evB == SC16IS750_INPUT_PIN_CHANGE_STATE)
      {
        uint8_t gpio_bits = navcore_sc16_read_gpio_bits();
        uint8_t changed   = (uint8_t)(gpio_bits ^ g_sc16_last_gpio_bits);
        g_sc16_last_gpio_bits = gpio_bits;

        if (changed & (uint8_t)EC11_IRQ_MASK) {
          uint8_t clk = (gpio_bits & EC11_MASK_CLK) ? 1 : 0;
          uint8_t dt  = (gpio_bits & EC11_MASK_DT)  ? 1 : 0;
          uint8_t sw  = (gpio_bits & EC11_MASK_SW)  ? 1 : 0;

          navcore_ec11_update(&g_navcore_ec11, clk, dt, sw, navcore_now_ms());
        }
      }
    }
  }
}

// --------------------
// Init NavCore (SC16 + tasks + ISR)
// --------------------
void rb_navcore_init(void)
{
    // Create stream buffers if needed (sizes you had before)
    if (!g_navcore_gps_sb)   g_navcore_gps_sb   = xStreamBufferCreate(2048, 1);
    if (!g_navcore_rs485_sb) g_navcore_rs485_sb = xStreamBufferCreate(2048, 1);

    // Init SC16 on your existing I2C bus (your existing code should be here)
    // NOTE: I keep your existing init calls as they were in your file, with the key lines below.
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
    SC16IS750_GPIOSetPinMode(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_CLK, INPUT);
    SC16IS750_GPIOSetPinMode(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_DT, INPUT);
    SC16IS750_GPIOSetPinMode(&g_navcore_sc, RB_NAVCORE_EC11_GPIO_SW, INPUT);

    // Enable GPIO change interrupt for GPIO0 & GPIO2 (CLK + SW only)
    SC16IS750_SetPinInterrupt(&g_navcore_sc, 0b00000101);

    // Create dedicated task and IRQ ISR
    xTaskCreate(NavCore_SC16_Task, "navcore_sc16", 4096, NULL, 10, &g_navcore_task);

    // Debug/console task (prints)
#ifdef RB02_BUTTON_KNOB
    xTaskCreate(NavCore_UI_Task, "ec11_dbg", 3072, NULL, 5, &g_navcore_ui_task);
#endif
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RB_NAVCORE_SC16_IRQ_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE, // try LOW_LEVEL if you suspect level IRQ
    };
    gpio_config(&io_conf);

    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE("NAVCORE", "gpio_install_isr_service failed: %d", (int)isr_err);
    }

    gpio_isr_handler_add(RB_NAVCORE_SC16_IRQ_GPIO, navcore_sc16_irq_isr, NULL);

    // Prime EC11 state
    navcore_ec11_init(&g_navcore_ec11);
    uint8_t gpio_bits = navcore_sc16_read_gpio_bits();
    g_sc16_last_gpio_bits = gpio_bits;

    g_navcore_ec11.last_clk = (gpio_bits & EC11_MASK_CLK) ? 1 : 0;
    g_navcore_ec11.last_sw  = (gpio_bits & EC11_MASK_SW)  ? 1 : 0;
    g_navcore_ec11.last_ab  = (uint8_t)(((g_navcore_ec11.last_clk & 1u) << 1) | (((gpio_bits & EC11_MASK_DT) ? 1 : 0) & 1u));
}

// Optional debug dump kept (your existing implementation can remain)
void navcore_ec11_debug_dump(const navcore_ec11_t *e)
{
#if NAVCORE_EC11_DEBUG
    printf("[EC11] q=%u/%u overflow=%u last_clk=%u last_sw=%u fast=%d accum=%d\n",
           (unsigned)e->q_rd, (unsigned)e->q_wr, (unsigned)e->q_overflow,
           (unsigned)e->last_clk, (unsigned)e->last_sw,
           (int)e->fast_mode, (int)e->accum);
#else
    (void)e;
#endif
}

#endif