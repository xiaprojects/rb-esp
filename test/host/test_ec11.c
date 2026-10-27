/*
 * Unit tests for the EC11 rotary-encoder state machine in
 * main/RB/RB02_NavCore.c and its EC11 -> touch mapping.
 *
 * Wiring/decoding facts the tests rely on (see RB02_NavCore.h):
 *  - inputs are raw GPIO levels; SW is active-low (pull-up idle = 1)
 *  - rotation is counted on the CLK falling edge only, DT sampled at that edge,
 *    NAVCORE_EC11_DIR_INVERT=1  ->  DT=0 is CW, DT=1 is CCW
 *  - one counted edge == one detent (EC11_DETENT_TRANSITIONS == 1)
 *  - short presses are aggregated for NAVCORE_EC11_MULTI_PRESS_WINDOW_MS and
 *    flushed by navcore_ui_poll_ec11(); triple fires immediately
 */
#include <string.h>
#include "unity.h"
#include "host_stubs.h"
#include "RB02_NavCore.h"

/* Instance used by the firmware poll/dispatch layer. */
extern navcore_ec11_t g_navcore_ec11;

/* Legacy getters (defined in RB02_NavCore.c, not exported by the header). */
int32_t NavCore_GetEncoderDelta(void);
bool NavCore_GetAndClearSwitchShortPressed(void);
bool NavCore_GetAndClearSwitchLongPressed(void);

/* ---- Recorder for the UI injection point (normally RB02_Knob.c) --------- */
#define MAX_TOUCHES 16
static uint8_t touches[MAX_TOUCHES];
static int touch_count;

void RB02_NavCore_InjectTouch(uint8_t touch_loc)
{
    if (touch_count < MAX_TOUCHES)
        touches[touch_count] = touch_loc;
    touch_count++;
}

/* ---- Signal simulator ----------------------------------------------------- */
static navcore_ec11_t e;
static uint32_t t;      /* simulated time in ms */
static uint8_t clk, dt, sw;

static void feed(void) { navcore_ec11_update(&e, clk, dt, sw, t); }

/* Gap between the (ignored) CLK rising edge and the counted falling edge. */
#define EDGE_GAP_MS 5u

/* One CW detent whose counted (falling) edge lands at at_ms: DT low. */
static void detent_cw(uint32_t at_ms)
{
    t = at_ms - EDGE_GAP_MS; clk = 1; dt = 0; feed();
    t = at_ms;               clk = 0;         feed();   /* counted edge */
}

/* One CCW detent whose counted (falling) edge lands at at_ms: DT high. */
static void detent_ccw(uint32_t at_ms)
{
    t = at_ms - EDGE_GAP_MS; clk = 1; dt = 1; feed();
    t = at_ms;               clk = 0;         feed();   /* counted edge */
}

static void press(uint32_t at_ms)   { t = at_ms; sw = 0; feed(); }
static void release(uint32_t at_ms) { t = at_ms; sw = 1; feed(); }

static int drain(uint32_t *out, int max)
{
    int n = 0;
    uint32_t ev;
    while (navcore_ec11_pop_event(&e, &ev)) {
        if (n < max) out[n] = ev;
        n++;
    }
    return n;
}

static uint32_t pop1(void)
{
    uint32_t ev = 0;
    TEST_ASSERT_TRUE_MESSAGE(navcore_ec11_pop_event(&e, &ev), "expected an event");
    return ev;
}

static void assert_queue_empty(void)
{
    uint32_t ev;
    TEST_ASSERT_FALSE_MESSAGE(navcore_ec11_pop_event(&e, &ev), "queue should be empty");
}

void setUp(void)
{
    navcore_ec11_init(&e);
    navcore_ec11_init(&g_navcore_ec11);
    t = 1000;                /* keep clear of the 0 == "never" sentinel */
    clk = 1; dt = 0; sw = 1; /* idle levels */
    feed();
    (void)NavCore_GetEncoderDelta();
    (void)NavCore_GetAndClearSwitchShortPressed();
    (void)NavCore_GetAndClearSwitchLongPressed();
    touch_count = 0;
    memset(touches, 0, sizeof(touches));
    host_stub_set_time_us(0);
}

void tearDown(void) {}

/* ---- Init ------------------------------------------------------------------ */

void test_init_idle_state(void)
{
    navcore_ec11_t x;
    memset(&x, 0xAA, sizeof(x));
    navcore_ec11_init(&x);
    TEST_ASSERT_EQUAL_UINT8(1, x.last_sw);
    TEST_ASSERT_FALSE(x.sw_is_down);
    TEST_ASSERT_FALSE(x.fast_mode);
    TEST_ASSERT_EQUAL_UINT16(0, x.q_wr);
    TEST_ASSERT_EQUAL_UINT16(0, x.q_rd);
    TEST_ASSERT_EQUAL_UINT8(0, x.short_press_seq);
    uint32_t ev;
    TEST_ASSERT_FALSE(navcore_ec11_pop_event(&x, &ev));
}

/* ---- Rotation -------------------------------------------------------------- */

void test_cw_detent(void)
{
    detent_cw(1200);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW, pop1());
    assert_queue_empty();
}

void test_ccw_detent(void)
{
    detent_ccw(1200);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CCW, pop1());
    assert_queue_empty();
}

void test_clk_rising_edge_is_not_counted(void)
{
    t = 1200; clk = 0; dt = 0; feed();   /* falling: counted */
    t = 1400; clk = 1;         feed();   /* rising: ignored */
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW, pop1());
    assert_queue_empty();
}

void test_dt_change_alone_is_not_rotation(void)
{
    t = 1200; dt = 1; feed();
    t = 1400; dt = 0; feed();
    assert_queue_empty();
}

void test_slow_detents_are_not_fast(void)
{
    detent_cw(1200);
    detent_cw(1400);   /* 200 ms >= FAST_EXIT */
    detent_cw(1600);
    uint32_t ev[4];
    TEST_ASSERT_EQUAL_INT(3, drain(ev, 4));
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW, ev[0]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW, ev[1]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW, ev[2]);
}

void test_quick_detents_enter_fast_mode(void)
{
    detent_cw(1200);
    detent_cw(1250);   /* 50 ms <= FAST_ENTER -> fast */
    uint32_t ev[2];
    TEST_ASSERT_EQUAL_INT(2, drain(ev, 2));
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW, ev[0]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_FAST_CW, ev[1]);
}

void test_fast_mode_hysteresis(void)
{
    detent_cw(1200);
    detent_cw(1250);   /* enter fast (50 ms) */
    detent_cw(1370);   /* 120 ms: between enter/exit thresholds -> stays fast */
    detent_cw(1600);   /* 230 ms >= FAST_EXIT -> back to normal */
    uint32_t ev[4];
    TEST_ASSERT_EQUAL_INT(4, drain(ev, 4));
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW,      ev[0]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_FAST_CW, ev[1]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_FAST_CW, ev[2]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW,      ev[3]);
}

void test_fast_ccw(void)
{
    detent_ccw(1200);
    detent_ccw(1240);
    uint32_t ev[2];
    TEST_ASSERT_EQUAL_INT(2, drain(ev, 2));
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CCW,      ev[0]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_FAST_CCW, ev[1]);
}

void test_clk_bounce_within_debounce_window_is_ignored(void)
{
    /* Genuine falling edge, then a contact bounce 0 ms later */
    t = 1200; clk = 0; dt = 0; feed();
    t = 1200; clk = 1;         feed();
    t = 1200; clk = 0;         feed();   /* < CLK_DEBOUNCE_MS: dropped */
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW, pop1());
    assert_queue_empty();
}

void test_legacy_delta_accumulates_signed_detents(void)
{
    detent_cw(1200);
    detent_cw(1400);
    detent_ccw(1600);
    TEST_ASSERT_EQUAL_INT32(1, NavCore_GetEncoderDelta());
    TEST_ASSERT_EQUAL_INT32(0, NavCore_GetEncoderDelta());   /* cleared on read */
}

/* ---- Button: long press ------------------------------------------------------ */

void test_long_press_fires_on_release(void)
{
    press(1200);
    release(1200 + EC11_LONG_PRESS_MS);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_LONG_PRESS, pop1());
    assert_queue_empty();
    TEST_ASSERT_TRUE(NavCore_GetAndClearSwitchLongPressed());
    TEST_ASSERT_FALSE(NavCore_GetAndClearSwitchLongPressed());
}

void test_nothing_fires_while_still_held(void)
{
    press(1200);
    t = 1200 + EC11_LONG_PRESS_MS + 500; feed();   /* still down */
    assert_queue_empty();
}

void test_just_under_long_press_threshold_is_short(void)
{
    press(1200);
    release(1200 + EC11_LONG_PRESS_MS - 1);
    assert_queue_empty();                        /* short press is deferred */
    TEST_ASSERT_EQUAL_UINT8(1, e.short_press_seq);
}

/* ---- Button: short / multi press --------------------------------------------- */

void test_single_short_press_is_deferred_until_window_expires(void)
{
    press(1200);
    release(1300);
    assert_queue_empty();
    TEST_ASSERT_EQUAL_UINT8(1, e.short_press_seq);
    TEST_ASSERT_EQUAL_UINT32(1300 + NAVCORE_EC11_MULTI_PRESS_WINDOW_MS, e.last_short_release_ms);
}

void test_double_press_within_window(void)
{
    press(1200);  release(1300);
    press(1400);  release(1500);
    assert_queue_empty();
    TEST_ASSERT_EQUAL_UINT8(2, e.short_press_seq);
}

void test_triple_press_fires_immediately(void)
{
    press(1200);  release(1300);
    press(1400);  release(1500);
    press(1600);  release(1700);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_TRIPLE_PRESS, pop1());
    assert_queue_empty();
    TEST_ASSERT_EQUAL_UINT8(0, e.short_press_seq);
}

void test_expired_pending_press_is_flushed_by_next_release(void)
{
    press(1200);  release(1300);            /* pending single, deadline 1700 */
    press(2000);  release(2100);            /* after the window */
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_PRESS, pop1());   /* the first one */
    assert_queue_empty();
    TEST_ASSERT_EQUAL_UINT8(1, e.short_press_seq);           /* second is pending */
}

void test_long_press_cancels_pending_short(void)
{
    press(1200);  release(1300);            /* pending single */
    press(1400);  release(1400 + EC11_LONG_PRESS_MS);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_LONG_PRESS, pop1());
    assert_queue_empty();
    TEST_ASSERT_EQUAL_UINT8(0, e.short_press_seq);
}

/* ---- Button: debounce --------------------------------------------------------- */

void test_switch_bounce_on_press_is_ignored(void)
{
    press(1200);
    release(1203);   /* 3 ms: bounce */
    press(1206);     /* 3 ms: bounce */
    release(1500);
    /* Exactly one clean press/release cycle should be seen */
    assert_queue_empty();
    TEST_ASSERT_EQUAL_UINT8(1, e.short_press_seq);
}

/* ---- Rotation while pressed ----------------------------------------------------- */

void test_rotation_while_pressed_emits_press_rotation(void)
{
    press(1200);
    detent_cw(1400);
    detent_ccw(1700);
    uint32_t ev[2];
    TEST_ASSERT_EQUAL_INT(2, drain(ev, 2));
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_PRESS_CW,  ev[0]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_PRESS_CCW, ev[1]);
}

void test_fast_rotation_while_pressed(void)
{
    press(1200);
    detent_cw(1400);
    detent_cw(1440);
    uint32_t ev[2];
    TEST_ASSERT_EQUAL_INT(2, drain(ev, 2));
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_PRESS_CW,      ev[0]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_PRESS_FAST_CW, ev[1]);
}

void test_release_after_rotation_emits_no_press(void)
{
    press(1200);
    detent_cw(1400);
    (void)pop1();                 /* PRESS_CW */
    release(1500);
    assert_queue_empty();
    TEST_ASSERT_EQUAL_UINT8(0, e.short_press_seq);
}

void test_release_after_long_hold_with_rotation_emits_no_long_press(void)
{
    press(1200);
    detent_cw(1400);
    (void)pop1();
    release(1200 + EC11_LONG_PRESS_MS + 100);
    assert_queue_empty();
    TEST_ASSERT_FALSE(NavCore_GetAndClearSwitchLongPressed());
}

void test_rotation_while_pressed_cancels_pending_short(void)
{
    press(1200);  release(1300);   /* pending single */
    press(1400);
    detent_cw(1450);
    (void)pop1();                  /* PRESS_CW */
    release(1600);
    assert_queue_empty();
    TEST_ASSERT_EQUAL_UINT8(0, e.short_press_seq);
}

/* ---- Event queue ---------------------------------------------------------------- */

void test_queue_preserves_order(void)
{
    detent_cw(1200);
    detent_ccw(1400);
    detent_cw(1600);
    uint32_t ev[3];
    TEST_ASSERT_EQUAL_INT(3, drain(ev, 3));
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW,  ev[0]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CCW, ev[1]);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW,  ev[2]);
}

void test_queue_overflow_keeps_most_recent(void)
{
    /* Slow detents so every event is a plain CW/CCW; the last one is CCW. */
    uint32_t at = 1200;
    for (int i = 0; i < NAVCORE_EC11_QUEUE_LEN + 5; i++, at += 200)
        detent_cw(at);
    detent_ccw(at);

    uint32_t ev[NAVCORE_EC11_QUEUE_LEN + 8];
    int n = drain(ev, NAVCORE_EC11_QUEUE_LEN + 8);
    TEST_ASSERT_EQUAL_INT(NAVCORE_EC11_QUEUE_LEN, n);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CCW, ev[n - 1]);
    TEST_ASSERT_GREATER_THAN_UINT16(0, e.q_overflow);
}

void test_pop_with_null_out_still_consumes(void)
{
    detent_cw(1200);
    TEST_ASSERT_TRUE(navcore_ec11_pop_event(&e, NULL));
    assert_queue_empty();
}

void test_legacy_bitmask_accumulates_and_clears(void)
{
    detent_cw(1200);
    detent_ccw(1400);
    uint32_t mask = navcore_ec11_get_and_clear(&e);
    TEST_ASSERT_EQUAL_HEX32(NAVCORE_EC11_EVT_CW | NAVCORE_EC11_EVT_CCW, mask);
    TEST_ASSERT_EQUAL_HEX32(0, navcore_ec11_get_and_clear(&e));
}

/* ---- Poll layer: EC11 -> RB02 touch injection ------------------------------------ */

static void g_detent_cw(uint32_t at_ms)
{
    navcore_ec11_update(&g_navcore_ec11, 1, 0, 1, at_ms - EDGE_GAP_MS);
    navcore_ec11_update(&g_navcore_ec11, 0, 0, 1, at_ms);
}

static void g_detent_ccw(uint32_t at_ms)
{
    navcore_ec11_update(&g_navcore_ec11, 1, 1, 1, at_ms - EDGE_GAP_MS);
    navcore_ec11_update(&g_navcore_ec11, 0, 1, 1, at_ms);
}

static void g_press(uint32_t at_ms)   { navcore_ec11_update(&g_navcore_ec11, 0, 0, 0, at_ms); }
static void g_release(uint32_t at_ms) { navcore_ec11_update(&g_navcore_ec11, 0, 0, 1, at_ms); }

void test_poll_maps_cw_to_north_and_ccw_to_south(void)
{
    g_detent_cw(1200);
    g_detent_ccw(1400);
    host_stub_set_time_us(1400 * 1000LL);
    navcore_ui_poll_ec11();
    TEST_ASSERT_EQUAL_INT(2, touch_count);
    TEST_ASSERT_EQUAL_UINT8(RB02_TOUCHLOC_N, touches[0]);
    TEST_ASSERT_EQUAL_UINT8(RB02_TOUCHLOC_S, touches[1]);
}

void test_poll_maps_fast_rotation_like_normal(void)
{
    g_detent_cw(1200);
    g_detent_cw(1240);   /* fast */
    host_stub_set_time_us(1240 * 1000LL);
    navcore_ui_poll_ec11();
    TEST_ASSERT_EQUAL_INT(2, touch_count);
    TEST_ASSERT_EQUAL_UINT8(RB02_TOUCHLOC_N, touches[1]);
}

void test_poll_flushes_single_press_only_after_window(void)
{
    g_press(1200);
    g_release(1300);

    host_stub_set_time_us(1500 * 1000LL);       /* inside the window */
    navcore_ui_poll_ec11();
    TEST_ASSERT_EQUAL_INT(0, touch_count);
    TEST_ASSERT_FALSE(NavCore_GetAndClearSwitchShortPressed());

    host_stub_set_time_us((1300 + NAVCORE_EC11_MULTI_PRESS_WINDOW_MS) * 1000LL);
    navcore_ui_poll_ec11();
    /* A single short press is not mapped to a touch location ... */
    TEST_ASSERT_EQUAL_INT(0, touch_count);
    /* ... but it is exposed through the legacy flag. */
    TEST_ASSERT_TRUE(NavCore_GetAndClearSwitchShortPressed());
}

void test_poll_maps_double_press_to_east(void)
{
    g_press(1200); g_release(1300);
    g_press(1400); g_release(1500);
    host_stub_set_time_us((1500 + NAVCORE_EC11_MULTI_PRESS_WINDOW_MS) * 1000LL);
    navcore_ui_poll_ec11();
    TEST_ASSERT_EQUAL_INT(1, touch_count);
    TEST_ASSERT_EQUAL_UINT8(RB02_TOUCHLOC_E, touches[0]);
}

void test_poll_maps_triple_press_to_west(void)
{
    g_press(1200); g_release(1300);
    g_press(1400); g_release(1500);
    g_press(1600); g_release(1700);
    host_stub_set_time_us(1700 * 1000LL);
    navcore_ui_poll_ec11();
    TEST_ASSERT_EQUAL_INT(1, touch_count);
    TEST_ASSERT_EQUAL_UINT8(RB02_TOUCHLOC_W, touches[0]);
}

void test_poll_long_press_is_not_mapped_to_touch(void)
{
    g_press(1200);
    g_release(1200 + EC11_LONG_PRESS_MS);
    host_stub_set_time_us((1200 + EC11_LONG_PRESS_MS) * 1000LL);
    navcore_ui_poll_ec11();
    TEST_ASSERT_EQUAL_INT(0, touch_count);
    TEST_ASSERT_TRUE(NavCore_GetAndClearSwitchLongPressed());
}

void test_poll_drains_queue(void)
{
    g_detent_cw(1200);
    host_stub_set_time_us(1200 * 1000LL);
    navcore_ui_poll_ec11();
    navcore_ui_poll_ec11();
    TEST_ASSERT_EQUAL_INT(1, touch_count);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_idle_state);

    RUN_TEST(test_cw_detent);
    RUN_TEST(test_ccw_detent);
    RUN_TEST(test_clk_rising_edge_is_not_counted);
    RUN_TEST(test_dt_change_alone_is_not_rotation);
    RUN_TEST(test_slow_detents_are_not_fast);
    RUN_TEST(test_quick_detents_enter_fast_mode);
    RUN_TEST(test_fast_mode_hysteresis);
    RUN_TEST(test_fast_ccw);
    RUN_TEST(test_clk_bounce_within_debounce_window_is_ignored);
    RUN_TEST(test_legacy_delta_accumulates_signed_detents);

    RUN_TEST(test_long_press_fires_on_release);
    RUN_TEST(test_nothing_fires_while_still_held);
    RUN_TEST(test_just_under_long_press_threshold_is_short);

    RUN_TEST(test_single_short_press_is_deferred_until_window_expires);
    RUN_TEST(test_double_press_within_window);
    RUN_TEST(test_triple_press_fires_immediately);
    RUN_TEST(test_expired_pending_press_is_flushed_by_next_release);
    RUN_TEST(test_long_press_cancels_pending_short);

    RUN_TEST(test_switch_bounce_on_press_is_ignored);

    RUN_TEST(test_rotation_while_pressed_emits_press_rotation);
    RUN_TEST(test_fast_rotation_while_pressed);
    RUN_TEST(test_release_after_rotation_emits_no_press);
    RUN_TEST(test_release_after_long_hold_with_rotation_emits_no_long_press);
    RUN_TEST(test_rotation_while_pressed_cancels_pending_short);

    RUN_TEST(test_queue_preserves_order);
    RUN_TEST(test_queue_overflow_keeps_most_recent);
    RUN_TEST(test_pop_with_null_out_still_consumes);
    RUN_TEST(test_legacy_bitmask_accumulates_and_clears);

    RUN_TEST(test_poll_maps_cw_to_north_and_ccw_to_south);
    RUN_TEST(test_poll_maps_fast_rotation_like_normal);
    RUN_TEST(test_poll_flushes_single_press_only_after_window);
    RUN_TEST(test_poll_maps_double_press_to_east);
    RUN_TEST(test_poll_maps_triple_press_to_west);
    RUN_TEST(test_poll_long_press_is_not_mapped_to_touch);
    RUN_TEST(test_poll_drains_queue);
    return UNITY_END();
}
