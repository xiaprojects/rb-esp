
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
#include "RB02_Defines.h"
#ifdef RB_ENABLE_NavCore

#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "sc16is750.h"
#include "driver/gpio.h"
#include "freertos/stream_buffer.h"
#include "driver/uart.h"
#include "RB02_GUIHelpers.h"
#include "ms4525do_rb.h"

/*
 * @file navcore.h
 * Author: The_MiNuS (2025)
 *
 * @brief RB02-NavCore board support helpers.
 *
 * This module groups together the integration code for the RB02-NavCore board:
 *  - SC16IS76x (UART/GPIO expander over I2C) handling GPS (port A), RS485 (port B) and EC11 GPIOs
 *  - MS4525DO differential pressure sensor helper (airspeed)
 *
 * Notes:
 *  - The EC11 signals are pulled-up (4.7 kOhm) and read through SC16 GPIO pins.
 *  - Interrupt from SC16 is routed to ESP32-S3 GPIO19 on the RB02-NavCore board.
 *
 * @details
 * ## Overall Architecture (RB02-NavCore)
 *
 * This module provides the integration “glue” for the RB02-NavCore board,
 * built around the SC16IS76x I2C UART/GPIO expander and its associated helpers.
 *
 * - **u-blox M8 GPS** connected to **UART Port A** of the SC16IS76x.
 * - **RS485 transceiver** connected to **UART Port B** of the SC16IS76x.
 * - **EC11 rotary encoder** connected to **SC16IS76x GPIOs**:
 *   - GPIO0 = CLK, GPIO1 = DT, GPIO2 = SW
 *   - 4.7kΩ pull-ups (idle = logic '1', pressed = logic '0' for SW)
 * - **SC16IS76x interrupt line** connected to **ESP32-S3 GPIO19**
 *   on the RB02-NavCore board.
 *
 * ### Execution model (IRQ + periodic polling)
 * The SC16IS76x can raise an interrupt on GPIO state changes (rotation or button).
 * To correctly handle **LONG_PRESS** timing even when GPIO levels remain stable,
 * the design relies on:
 *
 * 1) wake-up on SC16IS76x IRQ (GPIO19 edge),
 * 2) plus a periodic wake-up (e.g. every 20 ms) to maintain button press timing.
 *
 * On each wake-up:
 * - GPIO states are read (preferably in a **single port read**),
 * - `navcore_ec11_update(...)` is called with (CLK/DT/SW, now_ms),
 * - the EC11 state machine generates events into its internal queue,
 * - the UI layer can then consume these events.
 *
 * ## EC11 event model
 * - `navcore_ec11_init()` initializes the EC11 state machine.
 * - `navcore_ec11_update()` must be called periodically (or on IRQ + tick)
 *   with the current A/B/SW logic levels.
 * - `navcore_ec11_pop_event()` retrieves the next EC11 event
 *   (CW / CCW, FAST, SHORT_PRESS, LONG_PRESS, etc.).
 *
 * ### UI integration (RB02 glue)
 * - `navcore_ui_poll_ec11()` acts as the UI bridge: it consumes EC11 events
 *   and translates them into UI actions via
 *   `RB02_NavCore_InjectTouch(...)` (touch-equivalent injection).
 *
 * ## GPS / RS485 stream buffers
 * Bytes received from the SC16IS76x UARTs can be pushed into **FreeRTOS StreamBuffers**
 * for simple and decoupled consumption by the application:
 *
 * - `rb_navcore_gps_stream()`   → GPS StreamBuffer handle
 * - `rb_navcore_rs485_stream()` → RS485 StreamBuffer handle
 *
 * Typical consumption is done using:
 * `xStreamBufferReceive()` or `xStreamBufferBytesAvailable()`.
 *
 * ## Configuration and tuning
 * Timing parameters (debounce, long press duration, fast rotation threshold, etc.)
 * are defined using `#define` macros.
 * Default values are provided in this header and may be overridden before inclusion.
 */


#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief EC11 rotary encoder state machine.
 *
 * This state machine expects A/B/SW sampled at a regular cadence.
 * It outputs events such as CW/CCW, fast turns and button presses.
*/

// ===============================
// Streams (GPS / RS485) for RB02
// ===============================
StreamBufferHandle_t rb_navcore_gps_stream(void);
StreamBufferHandle_t rb_navcore_rs485_stream(void);

// ---- NavCore: SC16IS762/752 via existing I2C bus (GPIO07/15) + IRQ on GPIO19 --
#define RB_NAVCORE_SC16IS_ADDR7          0x4D
#define RB_NAVCORE_GPS_CH                SC16IS750_CHANNEL_A
#define RB_NAVCORE_RS485_CH              SC16IS750_CHANNEL_B
#define RB_NAVCORE_SC16_IRQ_GPIO         GPIO_NUM_19

// EC11 wiring on SC16 GPIOs
#define RB_NAVCORE_EC11_GPIO_CLK         0
#define RB_NAVCORE_EC11_GPIO_DT          1
#define RB_NAVCORE_EC11_GPIO_SW          2

// ===============================
// Events
// ===============================
typedef enum {
    NAVCORE_EC11_EVT_NONE            = 0,

    NAVCORE_EC11_EVT_CW              = (1u << 0),
    NAVCORE_EC11_EVT_CCW             = (1u << 1),
    NAVCORE_EC11_EVT_FAST_CW         = (1u << 2),
    NAVCORE_EC11_EVT_FAST_CCW        = (1u << 3),

    NAVCORE_EC11_EVT_PRESS           = (1u << 4),  /* short press (on release) */
    NAVCORE_EC11_EVT_LONG_PRESS      = (1u << 5),  /* long press (on release)  */

    NAVCORE_EC11_EVT_PRESS_CW        = (1u << 6),
    NAVCORE_EC11_EVT_PRESS_CCW       = (1u << 7),
    NAVCORE_EC11_EVT_PRESS_FAST_CW   = (1u << 8),
    NAVCORE_EC11_EVT_PRESS_FAST_CCW  = (1u << 9),

    NAVCORE_EC11_EVT_DOUBLE_PRESS   = (1u << 10),
    NAVCORE_EC11_EVT_TRIPLE_PRESS   = (1u << 11),

} navcore_ec11_evt_t;


#define RB02_TOUCHLOC_N      1u
#define RB02_TOUCHLOC_W      3u
#define RB02_TOUCHLOC_CENTER 4u
#define RB02_TOUCHLOC_E      5u
#define RB02_TOUCHLOC_S      7u

void RB02_NavCore_InjectTouch(uint8_t touch_loc);

// ===============================
// EC11 tuning (CLK-edge-only decode)
// ===============================

#ifndef NAVCORE_EC11_DIR_INVERT
#define NAVCORE_EC11_DIR_INVERT 1
#endif

// With CLK-edge-only decode, a detent typically produces 2 CLK edges
#define EC11_DETENT_TRANSITIONS  1

// Choix de l'edge utilisé pour compter : 0 = falling, 1 = rising
#define EC11_COUNT_ON_CLK_LEVEL  0

// Long press threshold (evaluated on release)
#define EC11_LONG_PRESS_MS       2000u

// Masks on SC16 GPIO port bitmap
#define EC11_MASK_CLK   (1u << RB_NAVCORE_EC11_GPIO_CLK)
#define EC11_MASK_DT    (1u << RB_NAVCORE_EC11_GPIO_DT)
#define EC11_MASK_SW    (1u << RB_NAVCORE_EC11_GPIO_SW)
#define EC11_IRQ_MASK   (EC11_MASK_CLK | EC11_MASK_SW)

// Fast hysteresis already used in your code
#ifndef NAVCORE_EC11_FAST_ENTER_MS
#define NAVCORE_EC11_FAST_ENTER_MS 90u
#endif

#ifndef NAVCORE_EC11_FAST_EXIT_MS
#define NAVCORE_EC11_FAST_EXIT_MS 150u
#endif

#define NAVCORE_EC11_TRACE_LEN 128

#ifndef NAVCORE_EC11_QUEUE_LEN
#define NAVCORE_EC11_QUEUE_LEN 32
#endif

#ifndef NAVCORE_EC11_CLK_DEBOUNCE_MS
#define NAVCORE_EC11_CLK_DEBOUNCE_MS 1u
#endif

#ifndef NAVCORE_EC11_SW_DEBOUNCE_MS
#define NAVCORE_EC11_SW_DEBOUNCE_MS 10u
#endif

#ifndef NAVCORE_EC11_MULTI_PRESS_WINDOW_MS
#define NAVCORE_EC11_MULTI_PRESS_WINDOW_MS 400u
#endif

// ===============================
// State
// ===============================
typedef struct navcore_ec11_s {

    /* Last sampled A/B as a 2-bit value: (a<<1)|b */
    uint8_t  last_ab;

    /* Last sampled switch state: 0 pressed, 1 released */
    uint8_t  last_sw;

    /* CLK-edge-only decode support (DT sampled on CLK edge) */
    uint8_t  last_clk;
    uint32_t last_clk_edge_ms;
    uint32_t last_sw_edge_ms;

    /* Ordered event queue to avoid losing events when UI polls slowly */
    uint16_t q_wr;
    uint16_t q_rd;
    uint16_t q_overflow;
    uint16_t q[NAVCORE_EC11_QUEUE_LEN];

    /* Accumulator of valid transitions; emits a detent when abs(accum) reaches
     * EC11_DETENT_TRANSITIONS.
     */
    int8_t   accum;

    /* Last non-zero step direction (+1 / -1). */
    int8_t   last_step_dir;

    /* Timestamp tracking for speed / press handling */
    uint32_t last_detent_ms;
    uint32_t press_start_ms;

    /* Speed classification */
    uint32_t fast_enter_ms;
    uint32_t fast_exit_ms;
    bool     fast_mode;
    bool     sw_is_down;

    /* Button behaviour */
    bool     long_press_fired;
    bool     rotation_while_pressed;

    uint32_t last_short_release_ms;
    uint8_t  short_press_seq;

    uint32_t events; /* legacy bitmask compatibility */

    struct navcore_ec11_trace_s {
        uint32_t t_ms;
        uint8_t old_ab;
        uint8_t new_ab;
        int8_t  step;
        int8_t  accum_before;
        uint8_t sw;
        uint8_t flags; /* bit0 invalid, bit1 detent, bit2 fast */
        int8_t  detent_dir;
    } trace[NAVCORE_EC11_TRACE_LEN];
    uint16_t trace_wr;

} navcore_ec11_t;

// ===============================
// API
// ===============================
/** @brief Initialize an EC11 state machine instance. */
void     navcore_ec11_init(navcore_ec11_t *e);

/*
 * @brief Initialize RB02-NavCore peripherals (SC16IS76x, EC11 GPIOs, etc.).
 *
 * This function is application-glue: it expects RB02 project macros (GPIO mapping, etc.).
*/
void     rb_navcore_init(void);

/*
 * @brief Feed new sampled A/B/SW states into the EC11 state machine.
 * @param now_ms Current time in milliseconds (monotonic).
*/
void     navcore_ec11_update(navcore_ec11_t *e, uint8_t a, uint8_t b, uint8_t sw, uint32_t now_ms);

// Legacy bitmask (compat)
uint32_t navcore_ec11_get_and_clear(navcore_ec11_t *e);

// Ordered queue pop (new)
/*
 * @brief Pop the next pending EC11 event from the internal queue.
 * @return true if an event was returned, false if the queue was empty.
*/
bool     navcore_ec11_pop_event(navcore_ec11_t *e, uint32_t *evt);

// Existing UI poll (now drains queue and prints)
/* @brief RB02 UI glue: poll/dispatch EC11 events and inject them into the UI navigation. */
void navcore_ui_poll_ec11(void);

/** @brief Dump the EC11 internal state for debugging (ESP_LOG). */
void navcore_ec11_debug_dump(const navcore_ec11_t *e);

#ifdef __cplusplus
} // extern "C"
#endif
#endif
