/*
 * Test-facing control API for the host stubs.
 *
 * The firmware modules under test call ESP-IDF / FreeRTOS / LVGL / driver
 * functions. On the host those are replaced by the fakes in this directory.
 * Tests use the functions below to drive the fakes deterministically.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

/* ---- esp_timer ---------------------------------------------------------- */
/* Set the value returned by esp_timer_get_time() (microseconds). */
void host_stub_set_time_us(int64_t us);
/* Advance the fake clock by the given number of milliseconds. */
void host_stub_advance_ms(uint32_t ms);
/* Convenience: current fake time in milliseconds. */
uint32_t host_stub_now_ms(void);

/* ---- LVGL --------------------------------------------------------------- */
/* Text passed to the most recent lv_label_set_text() call ("" if none). */
const char *host_stub_last_label_text(void);
/* Reset the object pool and all recorded LVGL interactions. */
void host_stub_lvgl_reset(void);
/* Display size reported by lv_disp_get_hor_res()/ver_res() (default 480x480). */
void host_stub_lvgl_set_resolution(int16_t w, int16_t h);
/* Number of objects created since the last reset. */
size_t host_stub_lvgl_object_count(void);
