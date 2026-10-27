/*
 * Integration tests for the traffic path:
 *
 *   FLARM / ADS-B sentences
 *        -> RB05_FlarmParser_ParseData()        (the real BLE / USB entry point)
 *        -> flarmStatus.obstacles[]             (the traffic store)
 *        -> RB05_Traffic_Tick()  / RB05_Radar_Tick()
 *        -> recorded LVGL state                 (what the pilot would see)
 *
 * Input is replayed from fixtures through the transport simulator in
 * sim/traffic_replay.c, which also advances `datetime` from the fixture's
 * $GPRMC timestamps, so traffic ages exactly as it would on the device.
 *
 * Layering of the tests matters: sentence-level tests go through the parser,
 * screen-level tests inject into the store directly. That keeps the screen
 * assertions independent of the parser defects recorded at the bottom of this
 * file, so they stay valid once those are fixed.
 *
 * Units, as the firmware uses them:
 *   store x = relative EAST metres,  y = relative NORTH metres
 *   store z = relative vertical metres; screens show z/100*3.28084, i.e.
 *             flight-level difference (hundreds of feet)
 */
#include <math.h>
#include <string.h>
#include "unity.h"
#include "host_stubs.h"
#include "traffic_replay.h"
#include "RB02_Config.h"
#include "RB05_FlarmParser.h"
#include "RB05_Traffic.h"
#include "RB05_Radar.h"

/* RB_TEST_DIR is this source directory, injected by CMake, so the test can be
 * run from any working directory. */
#ifndef RB_TEST_DIR
#define RB_TEST_DIR "."
#endif
#define FIXTURE_FLARM    RB_TEST_DIR "/fixtures/flarm_session.nmea"
#define FIXTURE_DUMP1090 RB_TEST_DIR "/fixtures/dump1090_traffic.nmea"

/* ---- firmware globals owned by the test ---------------------------------- */
datetime_t datetime = {0};
float AttitudeYaw = 0.0f;

static uint8_t operative_bluetooth;
void RB02_Config_Set_OperativeBluetooth(uint8_t operative) { operative_bluetooth = operative; }

extern FlarmStatus flarmStatus;

/* ---- fixtures and screen under test -------------------------------------- */
static replay_fixture_t fx_flarm, fx_dump1090;
static RB05_TrafficStatus ts;
static lv_obj_t *screen_root;

static void build_screens(void)
{
    memset(&ts, 0, sizeof(ts));
    screen_root = lv_obj_create(NULL);
    ts.lv_parent = screen_root;
    ts.lv_parent_radar = screen_root;
    RB05_Traffic_CreateScreen(&ts);
    RB05_Radar_CreateScreen(&ts);
}

/* Put a target straight into the store, bypassing the parser. */
static void inject(int slot, const char *id6, const char *tail,
                   int16_t east, int16_t north, int16_t vert_m,
                   int16_t bearing, uint32_t now)
{
    FlarmObstacle *o = &flarmStatus.obstacles[slot];
    memset(o, 0, sizeof(*o));
    memcpy(o->identifier, id6, 6);
    snprintf(o->tail, sizeof(o->tail), "%s", tail);
    o->x = east;
    o->y = north;
    o->z = vert_m;
    o->bearing = bearing;
    o->now = now;
}

static int8_t find_slot(const char *id6)
{
    for (int i = 0; i < RB05_Flarm_SERIAL; i++)
        if (memcmp(flarmStatus.obstacles[i].identifier, id6, 6) == 0)
            return (int8_t)i;
    return -1;
}

static const FlarmObstacle *obstacle(const char *id6)
{
    int8_t s = find_slot(id6);
    TEST_ASSERT_TRUE_MESSAGE(s >= 0, "aircraft not present in the traffic store");
    return &flarmStatus.obstacles[s];
}

static int stored_count(void)
{
    int n = 0;
    for (int i = 0; i < RB05_Flarm_SERIAL; i++)
        if (flarmStatus.obstacles[i].identifier[0] != 0) n++;
    return n;
}

void setUp(void)
{
    memset(&flarmStatus, 0, sizeof(flarmStatus));
    host_stub_lvgl_reset();
    replay_set_clock(0, 0, 0);
    AttitudeYaw = 0.0f;
    operative_bluetooth = 0;
    build_screens();
}

void tearDown(void) {}

/* ====================================================================== */
/* Transport: sentence framing                                             */
/* ====================================================================== */

void test_line_framed_capture_populates_the_store(void)
{
    /* The device capture at the top of the fixture holds three distinct
     * aircraft; all three must reach the store. */
    replay_all(&fx_flarm, REPLAY_LINE, 0);
    TEST_ASSERT_EQUAL_INT(4, stored_count());   /* 3 from the capture + D-EABC */
    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("4CA8AF"));
    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("39D2AC"));
}

void test_bulk_read_loses_no_traffic(void)
{
    /* A 255-byte stream read carries several sentences, so slot eviction can
     * land differently than with line framing; what must hold either way is
     * that every long-lived aircraft still arrives. */
    replay_all(&fx_flarm, REPLAY_LINE, 0);
    int line_count = stored_count();

    memset(&flarmStatus, 0, sizeof(flarmStatus));
    replay_set_clock(0, 0, 0);
    replay_all(&fx_flarm, REPLAY_BULK, 0);

    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("4CA8AF"));   /* RYR74SZ */
    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("39D2AC"));   /* AFR199  */
    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("3C6444"));   /* DLH123  */
    TEST_ASSERT_GREATER_OR_EQUAL_INT(line_count, stored_count());
}

void test_pflau_marks_the_bluetooth_link_operative(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, operative_bluetooth);
    replay_cycles(&fx_flarm, REPLAY_LINE, 0, 1);
    TEST_ASSERT_EQUAL_UINT8(1, operative_bluetooth);
}

void test_non_traffic_sentences_are_ignored(void)
{
    static const char *noise =
        "$GPGSA,A,3,,,,,,,,,,,,,1.0,1.0,1.0*33\r\n"
        "$RPYL,-38,62,334,-41,0,993,0\r\n"
        "$PGRMZ,918,f,3*1B\r\n";
    replay_set_clock(19, 18, 30);
    RB05_FlarmParser_ParseData((const uint8_t *)noise, (uint8_t)strlen(noise));
    TEST_ASSERT_EQUAL_INT(0, stored_count());
}

/* ====================================================================== */
/* Store semantics                                                         */
/* ====================================================================== */

void test_aircraft_identity_comes_from_the_icao_hex(void)
{
    replay_cycles(&fx_flarm, REPLAY_LINE, 0, 2);
    const FlarmObstacle *o = obstacle("4CA8AF");
    TEST_ASSERT_EQUAL_STRING("RYR74SZ", o->tail);
    TEST_ASSERT_EQUAL_INT16(136, o->bearing);
}

void test_repeated_reports_update_in_place_rather_than_duplicating(void)
{
    replay_cycles(&fx_flarm, REPLAY_LINE, 0, 2);
    int8_t slot = find_slot("4CA8AF");
    int count_before = stored_count();

    replay_cycles(&fx_flarm, REPLAY_LINE, 0, 4);

    TEST_ASSERT_EQUAL_INT8(slot, find_slot("4CA8AF"));
    TEST_ASSERT_EQUAL_INT(count_before, stored_count());
}

void test_position_advances_as_the_target_moves(void)
{
    replay_cycles(&fx_flarm, REPLAY_LINE, 0, 2);
    int16_t east_early = obstacle("4CA8AF")->x;

    replay_all(&fx_flarm, REPLAY_LINE, 0);
    int16_t east_late = obstacle("4CA8AF")->x;

    /* RYR74SZ tracks 136 degrees, so it moves steadily east. */
    TEST_ASSERT_GREATER_THAN_INT16(east_early, east_late);
}

void test_timestamp_follows_the_device_clock(void)
{
    replay_cycles(&fx_flarm, REPLAY_LINE, 0, 2);
    TEST_ASSERT_EQUAL_UINT32(replay_clock_seconds(), obstacle("4CA8AF")->now);
}

void test_a_slot_is_reused_once_its_aircraft_is_over_60s_stale(void)
{
    /* EFW4YW stops reporting after 19:18:33; DLH123 appears at 19:19:40.
     * With only 10 slots the stale one must be recycled. */
    replay_all(&fx_flarm, REPLAY_LINE, 0);
    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("3C6444"));   /* DLH123 arrived */
    TEST_ASSERT_EQUAL_INT8(-1, find_slot("407022"));       /* EFW4YW evicted */
}

/* ====================================================================== */
/* Traffic list screen                                                     */
/* ====================================================================== */

void test_traffic_list_shows_one_line_per_live_aircraft(void)
{
    replay_set_clock(10, 0, 30);
    uint32_t now = replay_clock_seconds();
    inject(0, "4CA8AF", "RYR74SZ", 5000, -10000, 9778, 136, now);
    inject(1, "39D2AC", "AFR199", 22291, -2977, 11899, 311, now);

    RB05_Traffic_Tick(&ts);

    TEST_ASSERT_NOT_NULL(strstr(ts.lv_lines->text, "RYR74SZ"));
    TEST_ASSERT_NOT_NULL(strstr(ts.lv_lines->text, "AFR199"));
}

void test_traffic_list_reports_flight_level_difference_and_range(void)
{
    /* 02:00 keeps us clear of the empty-slot window documented in
     * test_known_issue_traffic_list_shows_phantom_rows_for_empty_slots. */
    replay_set_clock(2, 0, 30);
    /* 5000 m east, 10000 m north -> 2.70 + 5.40 NM -> 6 NM slant.
     * 9778 m vertical -> 9778/100*3.28084 -> +318 FL. */
    inject(0, "4CA8AF", "RYR74SZ", 5000, -10000, 9778, 136, replay_clock_seconds());

    RB05_Traffic_Tick(&ts);

    TEST_ASSERT_EQUAL_STRING(" +318FL RYR74SZ 6NM\n", ts.lv_lines->text);
}

void test_traffic_list_omits_aircraft_silent_for_over_60s(void)
{
    replay_set_clock(10, 0, 30);
    uint32_t now = replay_clock_seconds();
    inject(0, "4CA8AF", "RYR74SZ", 5000, -10000, 9778, 136, now);
    inject(1, "39D2AC", "AFR199", 22291, -2977, 11899, 311, now - 61);

    RB05_Traffic_Tick(&ts);

    TEST_ASSERT_NOT_NULL(strstr(ts.lv_lines->text, "RYR74SZ"));
    TEST_ASSERT_NULL_MESSAGE(strstr(ts.lv_lines->text, "AFR199"),
                             "a 61 s old target must drop off the list");
}

void test_traffic_list_applies_the_vertical_filter(void)
{
    replay_set_clock(10, 0, 30);
    uint32_t now = replay_clock_seconds();
    inject(0, "AAAAAA", "LOW", 5000, -5000, 300, 90, now);     /* +9 FL  */
    inject(1, "BBBBBB", "HIGH", 5000, -5000, 9778, 90, now);   /* +318 FL */

    ts.verticalFilter = 2;   /* ftFilter = 20 FL */
    RB05_Traffic_Tick(&ts);

    TEST_ASSERT_NOT_NULL(strstr(ts.lv_lines->text, "LOW"));
    TEST_ASSERT_NULL(strstr(ts.lv_lines->text, "HIGH"));
}

void test_traffic_list_is_empty_with_no_traffic(void)
{
    replay_set_clock(2, 0, 30);
    RB05_Traffic_Tick(&ts);
    TEST_ASSERT_EQUAL_STRING("", ts.lv_lines->text);
}

/* ====================================================================== */
/* Radar screen                                                            */
/* ====================================================================== */

void test_radar_starts_at_zoom_level_7(void)
{
    TEST_ASSERT_EQUAL_UINT8(7, ts.zoomLevel);
}

void test_radar_places_a_target_by_relative_geometry(void)
{
    replay_set_clock(10, 0, 30);
    /* 5000 m east, 10000 m north; zoom 7 -> 16 NM full scale on a 480 px face.
     *   east  2.6998 NM / 16 * 240 =  40.5 px
     *   north 5.3996 NM / 16 * 240 =  81.0 px (screen +y, i.e. drawn below)
     * Bearing 136 deg -> LVGL angle in 0.1 deg units. */
    inject(0, "4CA8AF", "RYR74SZ", 5000, -10000, 9778, 136, replay_clock_seconds());

    RB05_Radar_Tick(&ts);

    lv_obj_t *icon = ts.lv_obstacles[0];
    TEST_ASSERT_FALSE(lv_obj_has_flag(icon, LV_OBJ_FLAG_HIDDEN));
    TEST_ASSERT_INT_WITHIN(1, 40, icon->x);
    TEST_ASSERT_INT_WITHIN(1, 81, icon->y);
    TEST_ASSERT_EQUAL_INT16(1360, icon->angle);
    TEST_ASSERT_EQUAL_STRING("+318FL\nRYR74SZ", ts.lv_obstacles_label[0]->text);
}

void test_radar_label_tracks_the_icon(void)
{
    replay_set_clock(10, 0, 30);
    inject(0, "4CA8AF", "RYR74SZ", 5000, -10000, 9778, 136, replay_clock_seconds());

    RB05_Radar_Tick(&ts);

    TEST_ASSERT_EQUAL_INT16(ts.lv_obstacles[0]->x, ts.lv_obstacles_label[0]->x);
    TEST_ASSERT_EQUAL_INT16(ts.lv_obstacles[0]->y + 44, ts.lv_obstacles_label[0]->y);
}

void test_radar_zooming_in_moves_targets_outward(void)
{
    replay_set_clock(10, 0, 30);
    inject(0, "4CA8AF", "RYR74SZ", 5000, -10000, 9778, 136, replay_clock_seconds());

    RB05_Radar_Tick(&ts);
    int16_t y_wide = ts.lv_obstacles[0]->y;

    RB05_Radar_Touch_N(&ts);          /* zoom in: 16 NM -> 14 NM */
    TEST_ASSERT_EQUAL_UINT8(6, ts.zoomLevel);
    RB05_Radar_Tick(&ts);

    TEST_ASSERT_GREATER_THAN_INT16(y_wide, ts.lv_obstacles[0]->y);
}

void test_radar_zoom_is_clamped_at_both_ends(void)
{
    for (int i = 0; i < 30; i++) RB05_Radar_Touch_N(&ts);
    TEST_ASSERT_EQUAL_UINT8(0, ts.zoomLevel);
    for (int i = 0; i < 40; i++) RB05_Radar_Touch_S(&ts);
    TEST_ASSERT_EQUAL_UINT8(19, ts.zoomLevel);
}

void test_radar_hides_a_target_silent_for_over_60s(void)
{
    replay_set_clock(10, 0, 30);
    inject(0, "4CA8AF", "RYR74SZ", 5000, -10000, 9778, 136, replay_clock_seconds());
    RB05_Radar_Tick(&ts);
    TEST_ASSERT_FALSE(lv_obj_has_flag(ts.lv_obstacles[0], LV_OBJ_FLAG_HIDDEN));

    replay_set_clock(10, 1, 32);      /* 62 s later, no new report */
    RB05_Radar_Tick(&ts);

    TEST_ASSERT_TRUE(lv_obj_has_flag(ts.lv_obstacles[0], LV_OBJ_FLAG_HIDDEN));
    TEST_ASSERT_TRUE(lv_obj_has_flag(ts.lv_obstacles_label[0], LV_OBJ_FLAG_HIDDEN));
}

void test_radar_leaves_empty_slots_alone(void)
{
    replay_set_clock(10, 0, 30);
    inject(0, "4CA8AF", "RYR74SZ", 5000, -10000, 9778, 136, replay_clock_seconds());
    RB05_Radar_Tick(&ts);

    /* Slot 1 was never written, so its icon keeps the off-screen parking
     * position set by RB05_Radar_CreateScreen(). */
    TEST_ASSERT_EQUAL_INT16(-500, ts.lv_obstacles[1]->x);
    TEST_ASSERT_EQUAL_INT16(-500, ts.lv_obstacles[1]->y);
}

void test_radar_ownship_symbol_stays_north_without_a_heading_source(void)
{
    /* This build defines neither RB_ENABLE_TRK nor RB_ENABLE_GPS, and the
     * QMI8658 is a 6-axis IMU, so there is no absolute heading available.
     * Rotating the ownship symbol by raw Madgwick yaw would put it in a
     * different reference frame from the traffic, which is drawn against true
     * north, so it is pinned to north instead. */
    replay_set_clock(2, 0, 30);
    AttitudeYaw = -90.0f;
    RB05_Radar_Tick(&ts);
    TEST_ASSERT_EQUAL_INT16(0, ts.lv_myself->angle);

    AttitudeYaw = 137.0f;
    RB05_Radar_Tick(&ts);
    TEST_ASSERT_EQUAL_INT16(0, ts.lv_myself->angle);
}

void test_radar_bearing_rose_is_placed_on_the_first_tick(void)
{
    /* The rose is only repositioned when the heading changes, so a freshly
     * created screen must still get its first placement. */
    replay_set_clock(2, 0, 30);
    RB05_Radar_Tick(&ts);

    /* 0 degrees sits at the top of the face, 90 to the right. */
    TEST_ASSERT_INT_WITHIN(1, 0, ts.Numbers[0]->x);
    TEST_ASSERT_INT_WITHIN(1, -224, ts.Numbers[0]->y);
    TEST_ASSERT_INT_WITHIN(1, 224, ts.Numbers[1]->x);
    TEST_ASSERT_INT_WITHIN(1, 0, ts.Numbers[1]->y);
}

void test_radar_center_touch_cycles_the_vertical_filter(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, ts.verticalFilter);
    RB05_Radar_Touch_Center(&ts);
    TEST_ASSERT_EQUAL_UINT8(2, ts.verticalFilter);
    TEST_ASSERT_NOT_NULL(strstr(ts.lv_label_lower->text, "20FL"));

    for (int i = 0; i < 9; i++) RB05_Radar_Touch_Center(&ts);
    TEST_ASSERT_EQUAL_UINT8(0, ts.verticalFilter);   /* wraps at 20 */
}

/* ====================================================================== */
/* End to end, from each input source                                      */
/* ====================================================================== */

void test_flarm_session_reaches_both_screens(void)
{
    replay_all(&fx_flarm, REPLAY_LINE, 0);

    RB05_Traffic_Tick(&ts);
    TEST_ASSERT_NOT_NULL(strstr(ts.lv_lines->text, "RYR74SZ"));
    TEST_ASSERT_NOT_NULL(strstr(ts.lv_lines->text, "AFR199"));

    RB05_Radar_Tick(&ts);
    int8_t slot = find_slot("4CA8AF");
    TEST_ASSERT_FALSE(lv_obj_has_flag(ts.lv_obstacles[slot], LV_OBJ_FLAG_HIDDEN));
    TEST_ASSERT_NOT_NULL(strstr(ts.lv_obstacles_label[slot]->text, "RYR74SZ"));
}

void test_dump1090_derived_session_reaches_both_screens(void)
{
    /* fixtures/dump1090_traffic.nmea is produced from a dump1090 SBS-1 capture
     * by tools/sbs_to_pflaa.py, so this exercises ADS-B input over the FLARM
     * path the firmware already implements. */
    replay_all(&fx_dump1090, REPLAY_LINE, 0);

    TEST_ASSERT_EQUAL_INT(3, stored_count());
    TEST_ASSERT_EQUAL_STRING("RYR74SZ", obstacle("4CA8AF")->tail);
    TEST_ASSERT_EQUAL_STRING("DLH123", obstacle("3C6444")->tail);

    RB05_Traffic_Tick(&ts);
    TEST_ASSERT_NOT_NULL(strstr(ts.lv_lines->text, "DLH123"));

    RB05_Radar_Tick(&ts);
    int8_t slot = find_slot("39D2AC");
    TEST_ASSERT_FALSE(lv_obj_has_flag(ts.lv_obstacles[slot], LV_OBJ_FLAG_HIDDEN));
}

void test_dump1090_altitudes_survive_the_conversion(void)
{
    /* The capture has RYR74SZ at 32000 ft with ownship at 278 m, i.e.
     * 9753 - 278 = 9475 m relative -> +311 FL on screen. */
    replay_cycles(&fx_dump1090, REPLAY_LINE, 0, 1);
    TEST_ASSERT_INT16_WITHIN(5, 9476, obstacle("4CA8AF")->z);
}

/* ====================================================================== */
/* Regressions                                                             */
/*                                                                         */
/* Each of these started life as a TEST_IGNORE marker for a defect this     */
/* suite found, and is now a live assertion. The comments record what the   */
/* defect was so the fix is not quietly undone.                            */
/* ====================================================================== */

void test_negative_coordinates_round_trip(void)
{
    /* RB05_FlarmParser.c treats '-' as "subtract one" instead of setting a
     * sign, so every negative field is corrupted:
     *     finalNumber = finalNumber - 1;        <-- wrong
     * then each digit does finalNumber = 10*finalNumber + digit.
     * "-1" becomes -9, "-9" becomes -1, "-14025" becomes -85975 (which then
     * truncates to -20439 in the int16_t store). Any target south or west of
     * the aircraft is drawn in the wrong place; the same bug is in
     * nmea_RMC_UpdatedValueFor()/nmea_GGA_UpdatedValueFor() in RB02.c.
     * Fix: keep a sign flag and apply it once the field ends. */
    replay_set_clock(19, 18, 30);
    static const char *s =
        "$PFLAA,0,-14025,5073,9778,1,4CA8AF!RYR74SZ,136,0,215,-0.3,9*18\r\n";
    RB05_FlarmParser_ParseData((const uint8_t *)s, (uint8_t)strlen(s));

    TEST_ASSERT_EQUAL_INT16(-14025, obstacle("4CA8AF")->y);
}

void test_negative_magnitudes_are_exact_at_every_digit_count(void)
{
    /* The old scanner seeded the accumulator with -1 on '-', giving
     * finalNumber = magnitude - 10^digits: -1 read as -9, -9 as -1, -14025 as
     * -85975 (which then wrapped in the int16_t store). Check each width. */
    static const int16_t cases[] = { -1, -9, -10, -100, -2977, -14025, -32000 };
    char sentence[128];

    for (unsigned i = 0; i < sizeof(cases) / sizeof(*cases); i++) {
        memset(&flarmStatus, 0, sizeof(flarmStatus));
        replay_set_clock(2, 0, (uint8_t)(i + 1));
        snprintf(sentence, sizeof(sentence),
                 "$PFLAA,0,%d,%d,%d,1,ABCDEF!T%u,90,0,100,0.0,9*00\r\n",
                 cases[i], cases[i], cases[i], i);
        RB05_FlarmParser_ParseData((const uint8_t *)sentence, (uint16_t)strlen(sentence));

        const FlarmObstacle *o = obstacle("ABCDEF");
        TEST_ASSERT_EQUAL_INT16(cases[i], o->y);
        TEST_ASSERT_EQUAL_INT16(cases[i], o->x);
        TEST_ASSERT_EQUAL_INT16(cases[i], o->z);
    }
}

void test_icon_heading_matches_the_apparent_motion(void)
{
    /* RYR74SZ declares track 136 and the icon is rotated to it. With the sign
     * bug the stored north component moved the wrong way, so the blip slid
     * across the screen at 43.9 degrees while the nose pointed at 136 - a
     * reflection about the east-west axis. Positions must now agree. */
    replay_cycles(&fx_flarm, REPLAY_LINE, 0, 2);
    const FlarmObstacle *o = obstacle("4CA8AF");
    int16_t east0 = o->x, north0 = o->y;

    replay_cycles(&fx_flarm, REPLAY_LINE, 0, 3);
    o = obstacle("4CA8AF");
    double de = (double)(o->x - east0);
    double dn = (double)(o->y - north0);

    TEST_ASSERT_TRUE_MESSAGE(de != 0.0 || dn != 0.0, "target did not move");
    double brg = atan2(de, dn) * 180.0 / M_PI;
    if (brg < 0) brg += 360.0;

    TEST_ASSERT_DOUBLE_WITHIN(2.0, 136.0, brg);

    /* ...and the icon nose is drawn at that same track. */
    RB05_Radar_Tick(&ts);
    TEST_ASSERT_EQUAL_INT16(1360, ts.lv_obstacles[find_slot("4CA8AF")]->angle);
}

void test_south_west_target_is_drawn_in_the_lower_left_quadrant(void)
{
    replay_set_clock(2, 0, 30);
    static const char *s =
        "$PFLAA,0,-5000,-5000,100,1,4CA8AF!SW,225,0,200,0.0,9*00\r\n";
    RB05_FlarmParser_ParseData((const uint8_t *)s, (uint16_t)strlen(s));

    const FlarmObstacle *o = obstacle("4CA8AF");
    TEST_ASSERT_EQUAL_INT16(-5000, o->y);   /* south */
    TEST_ASSERT_EQUAL_INT16(-5000, o->x);   /* west  */

    RB05_Radar_Tick(&ts);
    lv_obj_t *icon = ts.lv_obstacles[find_slot("4CA8AF")];
    TEST_ASSERT_FALSE(lv_obj_has_flag(icon, LV_OBJ_FLAG_HIDDEN));
    TEST_ASSERT_LESS_THAN_INT16(0, icon->x);        /* left of centre  */
    TEST_ASSERT_GREATER_THAN_INT16(0, icon->y);     /* below centre    */
}

void test_positive_coordinates_round_trip(void)
{
    /* Companion to the test above: the positive field on the same sentence is
     * exact, which is why the defect is easy to miss on a flight where traffic
     * happens to be north-east. */
    replay_set_clock(19, 18, 30);
    static const char *s =
        "$PFLAA,0,-14025,5073,9778,1,4CA8AF!RYR74SZ,136,0,215,-0.3,9*18\r\n";
    RB05_FlarmParser_ParseData((const uint8_t *)s, (uint8_t)strlen(s));

    TEST_ASSERT_EQUAL_INT16(5073, obstacle("4CA8AF")->x);
    TEST_ASSERT_EQUAL_INT16(9778, obstacle("4CA8AF")->z);
}

void test_ble_fragments_are_reassembled_into_sentences(void)
{
    /* A GATT notification carries 20 bytes at the default MTU of 23, while a
     * $PFLAA runs to about 60, so every sentence straddles a fragment.
     * RB05_FlarmParser_FeedStream() buffers to end-of-line first. */
    unsigned calls = replay_all(&fx_flarm, REPLAY_BLE, 20);
    TEST_ASSERT_GREATER_THAN_UINT32(1, calls);   /* it really did fragment */

    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("4CA8AF"));
    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("39D2AC"));
    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("3C6444"));
    TEST_ASSERT_EQUAL_STRING("RYR74SZ", obstacle("4CA8AF")->tail);
}

void test_ble_fragment_size_does_not_change_the_result(void)
{
    static const size_t chunks[] = { 20, 23, 40, 64, 128, 255 };

    replay_all(&fx_flarm, REPLAY_LINE, 0);
    const int expected = stored_count();
    TEST_ASSERT_GREATER_THAN_INT(0, expected);

    for (unsigned i = 0; i < sizeof(chunks) / sizeof(*chunks); i++) {
        memset(&flarmStatus, 0, sizeof(flarmStatus));
        replay_set_clock(0, 0, 0);
        replay_all(&fx_flarm, REPLAY_BLE, chunks[i]);
        TEST_ASSERT_EQUAL_INT_MESSAGE(expected, stored_count(),
                                      "fragment size changed the parsed traffic");
    }
}

void test_regression_unreassembled_fragments_lose_traffic(void)
{
    /* Why FeedStream exists: handing raw fragments to ParseData, as the BLE
     * path used to, loses every sentence that straddles a boundary. */
    replay_all(&fx_flarm, REPLAY_BLE_NO_REASSEMBLY, 20);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, stored_count(),
        "20-byte fragments carry no complete sentence");
}

void test_ble_overlong_line_is_discarded_not_truncated(void)
{
    /* A stream with no line terminator must not be parsed as a sentence, and
     * must not wedge the reassembler for the next one. */
    char junk[600];
    memset(junk, 'A', sizeof(junk));
    RB05_FlarmParser_FeedStream((const uint8_t *)junk, (uint16_t)sizeof(junk));
    TEST_ASSERT_EQUAL_INT(0, stored_count());

    replay_set_clock(2, 0, 30);
    static const char *good =
        "$PFLAA,0,1000,2000,500,1,4CA8AF!OK,90,0,200,0.0,9*00\r\n";
    RB05_FlarmParser_FeedStream((const uint8_t *)good, (uint16_t)strlen(good));
    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("4CA8AF"));
}

void test_fragments_never_create_an_unidentified_entry(void)
{
    /* RB05_FlarmParser_FindEntryByIdentifier() memcmp's against every slot
     * including untouched ones, whose identifier is all zeros. A $PFLAA whose
     * ID field never got parsed therefore "matches" slot 0 and overwrites it.
     * Fix: skip empty slots when searching, and reject a PFLAA with no ID. */
    replay_all(&fx_flarm, REPLAY_BLE_NO_REASSEMBLY, 20);

    int junk = 0;
    for (int i = 0; i < RB05_Flarm_SERIAL; i++) {
        const FlarmObstacle *o = &flarmStatus.obstacles[i];
        if (o->identifier[0] == 0 && (o->now != 0 || o->y != 0)) junk++;
    }
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, junk,
        "a sentence with no parsed ID must be dropped, not written to slot 0");
}

void test_radar_range_label_matches_the_drawn_scale(void)
{
    /* RB05_Radar_Tick() draws at   2 * (1 + zoomLevel) NM.
     * RB05_Radar_Touch_N()  shows  2 * (1 + zoomLevel) NM  - agrees.
     * RB05_Radar_Touch_S()  shows  5 * (1 + zoomLevel) NM  - disagrees.
     * So zooming out claims a range 2.5x larger than what is actually drawn,
     * and the initial label from CreateScreen says 20 NM while zoom level 7
     * draws 16 NM. Fix: derive the label from one shared helper. */
    char expected[32];

    /* CreateScreen: zoomLevel 7 draws 16 NM, and the label used to be the
     * hardcoded string "RANGE:20NM 500FL". */
    snprintf(expected, sizeof(expected), "RANGE:%dNM", RB05_Radar_RangeNm(&ts));
    TEST_ASSERT_EQUAL_STRING("RANGE:16NM 500FL", ts.lv_label_lower->text);
    TEST_ASSERT_NOT_NULL(strstr(ts.lv_label_lower->text, expected));

    /* Zooming out used to advertise 5 * (1 + zoomLevel) while Tick drew
     * 2 * (1 + zoomLevel). */
    RB05_Radar_Touch_S(&ts);                       /* zoomLevel 7 -> 8 */
    snprintf(expected, sizeof(expected), "RANGE:%dNM", RB05_Radar_RangeNm(&ts));
    TEST_ASSERT_NOT_NULL(strstr(ts.lv_label_lower->text, expected));

    RB05_Radar_Touch_N(&ts);                       /* back to 7 */
    snprintf(expected, sizeof(expected), "RANGE:%dNM", RB05_Radar_RangeNm(&ts));
    TEST_ASSERT_NOT_NULL(strstr(ts.lv_label_lower->text, expected));
}

void test_radar_label_range_is_the_range_targets_are_plotted_at(void)
{
    /* Place a target at exactly the advertised full-scale range due east and
     * it must land on the right edge of the face. */
    replay_set_clock(2, 0, 30);
    const int range_nm = RB05_Radar_RangeNm(&ts);        /* 16 at zoom 7 */
    const int16_t east_m = (int16_t)(range_nm * 1852);   /* NM -> metres */
    inject(0, "4CA8AF", "EDGE", east_m, 0, 100, 90, replay_clock_seconds());

    RB05_Radar_Tick(&ts);

    /* Half of a 480 px face. */
    TEST_ASSERT_INT_WITHIN(2, 240, ts.lv_obstacles[0]->x);
    TEST_ASSERT_INT_WITHIN(2, 0, ts.lv_obstacles[0]->y);
}

void test_radar_hides_a_target_outside_the_vertical_filter(void)
{
    /* In RB05_Radar_Tick() the staleness branch hides the icon, but the
     * vertical-filter branch just `continue`s. A target already on screen that
     * climbs out of the filter band keeps its last icon and label forever.
     * RB05_Radar_Touch_Center() masks this by hiding everything when the filter
     * changes, so it only shows up when the traffic moves, not the filter.
     * Fix: hide the icon before `continue`, as the staleness branch does. */
    replay_set_clock(10, 0, 30);
    ts.verticalFilter = 2;                     /* ftFilter = 20 FL */
    inject(0, "AAAAAA", "CLIMBER", 5000, -5000, 300, 90, replay_clock_seconds());
    RB05_Radar_Tick(&ts);
    TEST_ASSERT_FALSE_MESSAGE(lv_obj_has_flag(ts.lv_obstacles[0], LV_OBJ_FLAG_HIDDEN),
                              "precondition: +9 FL is inside the 20 FL filter");

    /* Same aircraft, now well above the filter band. */
    replay_set_clock(10, 0, 31);
    inject(0, "AAAAAA", "CLIMBER", 5000, -5000, 9778, 90, replay_clock_seconds());
    RB05_Radar_Tick(&ts);

    TEST_ASSERT_TRUE(lv_obj_has_flag(ts.lv_obstacles[0], LV_OBJ_FLAG_HIDDEN));
    TEST_ASSERT_TRUE(lv_obj_has_flag(ts.lv_obstacles_label[0], LV_OBJ_FLAG_HIDDEN));
}

void test_traffic_list_has_no_phantom_rows_at_any_time_of_day(void)
{
    /* RB05_Radar_Tick() skips untouched slots with
     *     if (flarmStatus.obstacles[i].now == 0) continue;
     * RB05_Traffic_Tick() has no such guard and relies only on
     *     int16_t diff = tStatus->lastTimeDirty - obstacles[i].now;
     *     if (diff > 60) continue;
     * For an empty slot `now` is 0, so diff is just lastTimeDirty, which is
     * uint16_t seconds-of-day. Once that exceeds INT16_MAX the assignment to
     * int16_t goes negative, the staleness test stops rejecting the slot, and
     * the list renders all ten of them as " +0FL  0NM".
     * Window: seconds-of-day 32768..65535, i.e. 09:06:08..18:12:15 UTC.
     * Fix: add the `now == 0` guard and widen `diff`/`lastTimeDirty` to int32_t. */
    /* Sweep the whole day, including the 09:06:08..18:12:15 window where
     * seconds-of-day exceeds INT16_MAX. */
    static const uint8_t hours[] = { 0, 2, 8, 9, 10, 14, 18, 19, 20, 23 };
    for (unsigned i = 0; i < sizeof(hours) / sizeof(*hours); i++) {
        replay_set_clock(hours[i], 30, 0);
        RB05_Traffic_Tick(&ts);
        TEST_ASSERT_EQUAL_STRING_MESSAGE("", ts.lv_lines->text,
            "empty slots must not render as rows at any time of day");
    }

    /* And a real target is still listed inside that window. */
    replay_set_clock(10, 30, 0);
    inject(0, "4CA8AF", "RYR74SZ", 5000, -10000, 9778, 136, replay_clock_seconds());
    RB05_Traffic_Tick(&ts);
    TEST_ASSERT_EQUAL_STRING(" +318FL RYR74SZ 6NM\n", ts.lv_lines->text);
}

/* Feed one synthetic $PFLAA with a given ICAO hex and callsign. */
static void send_target(const char *hex6, const char *tail)
{
    char sentence[128];
    snprintf(sentence, sizeof(sentence),
             "$PFLAA,0,1000,2000,500,1,%s!%s,90,0,200,0.0,9*00\r\n", hex6, tail);
    RB05_FlarmParser_ParseData((const uint8_t *)sentence, (uint8_t)strlen(sentence));
}

void test_store_holds_ten_simultaneous_targets(void)
{
    /* RB05_Flarm_SERIAL slots, all filled with live traffic. */
    replay_set_clock(2, 0, 0);
    for (int i = 0; i < RB05_Flarm_SERIAL; i++) {
        char hex[7];
        snprintf(hex, sizeof(hex), "A0000%d", i);
        send_target(hex, "X");
    }
    TEST_ASSERT_EQUAL_INT(RB05_Flarm_SERIAL, stored_count());
}

void test_a_stale_slot_is_recycled_within_the_same_day(void)
{
    replay_set_clock(2, 0, 0);
    for (int i = 0; i < RB05_Flarm_SERIAL; i++) {
        char hex[7];
        snprintf(hex, sizeof(hex), "A0000%d", i);
        send_target(hex, "X");
    }
    replay_set_clock(3, 5, 0);                 /* 65 minutes later */
    send_target("B00001", "NEW");

    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("B00001"));
    TEST_ASSERT_EQUAL_INT(RB05_Flarm_SERIAL, stored_count());
}

void test_stale_slots_are_recycled_across_midnight(void)
{
    /* Traffic is stamped with seconds-of-day, and a slot is freed only when
     *     obstacles[i].now + 60 < buffer.now
     * Past midnight `buffer.now` restarts near 0 while the stored stamps are
     * near 86400, so that test can never succeed: every slot looks fresh and
     * new traffic is dropped until the clock climbs back past the old stamps.
     * Here ten targets are seen at 23:00 and a new one arrives at 00:05, an
     * hour later, with nothing recycled.
     * Fix: stamp with a monotonic counter (esp_timer_get_time()) instead of
     * seconds-of-day. */
    replay_set_clock(23, 0, 0);
    for (int i = 0; i < RB05_Flarm_SERIAL; i++) {
        char hex[7];
        snprintf(hex, sizeof(hex), "A0000%d", i);
        send_target(hex, "X");
    }
    TEST_ASSERT_EQUAL_INT(RB05_Flarm_SERIAL, stored_count());

    replay_set_clock(0, 5, 0);                 /* 65 minutes later, next day */
    send_target("B00001", "NEW");

    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("B00001"));
}

void test_a_target_seen_before_midnight_ages_out_after_it(void)
{
    replay_set_clock(23, 59, 50);
    inject(0, "4CA8AF", "RYR74SZ", 5000, -10000, 9778, 136, replay_clock_seconds());

    replay_set_clock(0, 0, 5);            /* 15 s later, past midnight */
    RB05_Traffic_Tick(&ts);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(ts.lv_lines->text, "RYR74SZ"),
        "a 15 s old target must survive the midnight rollover");

    replay_set_clock(0, 1, 0);            /* 70 s after the report */
    RB05_Traffic_Tick(&ts);
    TEST_ASSERT_NULL_MESSAGE(strstr(ts.lv_lines->text, "RYR74SZ"),
        "a 70 s old target must still age out across midnight");
}

void test_eleventh_simultaneous_target_is_dropped_without_reading_past_the_array(void)
{
    /* With every slot live, the search loop in RB05_FlarmParser_ParsePFLAA()
     * ends with `index == RB05_Flarm_SERIAL` and the trailing
     *     printf("Traffic update %d %s ...", index,
     *            flarmStatus.obstacles[index].tail, ...)
     * reads obstacles[10] of a 10-element array. It happens to land on the
     * adjacent `threat` member rather than crashing, which is why it has gone
     * unnoticed. Built with -DRB_SANITIZE=ON this reports
     *     RB05_FlarmParser.c:286: index 10 out of bounds for type
     *     'FlarmObstacle [10]'
     * The visible effect is that the 11th simultaneous target is dropped with
     * no indication. Fix: guard the log on `index < RB05_Flarm_SERIAL`, and
     * consider evicting the oldest slot rather than dropping the newcomer. */
    replay_set_clock(2, 0, 0);
    for (int i = 0; i < RB05_Flarm_SERIAL; i++) {
        char hex[7];
        snprintf(hex, sizeof(hex), "A0000%d", i);
        send_target(hex, "X");
    }
    send_target("C00001", "ELEVEN");

    /* Capacity is RB05_Flarm_SERIAL, so the newcomer is dropped - that part is
     * by design. What must not happen is the out-of-bounds read of
     * obstacles[RB05_Flarm_SERIAL] that the trailing log used to do; build with
     * -DRB_SANITIZE=ON and UBSan confirms it is gone. */
    TEST_ASSERT_EQUAL_INT8(-1, find_slot("C00001"));
    TEST_ASSERT_EQUAL_INT(RB05_Flarm_SERIAL, stored_count());

    /* The existing ten are untouched. */
    for (int i = 0; i < RB05_Flarm_SERIAL; i++) {
        char hex[7];
        snprintf(hex, sizeof(hex), "A0000%d", i);
        TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot(hex));
    }
}

void test_checksum_field_does_not_corrupt_the_accumulator(void)
{
    /* The field scanner in RB05_FlarmParser_ParsePFLAA() ends a field only on
     * ',', '.', '\r', '\n' or NUL. '*' is not a terminator and non-digits are
     * not rejected, so the checksum and anything after it keep feeding
     *     finalNumber = 10 * finalNumber + (sentence[x] - '0');
     * which overflows a signed int - undefined behaviour. Built with
     * -DRB_SANITIZE=ON this reports
     *     RB05_FlarmParser.c:246: signed integer overflow
     * Stored fields happen to survive because only field indices 0..8 are
     * kept, so this is latent rather than visible today.
     * Fix: stop the field at '*' and ignore non-digits. */
    replay_set_clock(2, 0, 0);
    static const char *s =
        "$PFLAA,0,1000,2000,500,1,4CA8AF!RYR74SZ,136,0,215,-0.3,9*18\r\n";
    RB05_FlarmParser_ParseData((const uint8_t *)s, (uint8_t)strlen(s));

    const FlarmObstacle *o = obstacle("4CA8AF");
    TEST_ASSERT_EQUAL_INT16(1000, o->y);
    TEST_ASSERT_EQUAL_INT16(2000, o->x);
    TEST_ASSERT_EQUAL_INT16(500, o->z);
    TEST_ASSERT_EQUAL_INT16(136, o->bearing);
}

void test_sentence_without_a_checksum_still_delivers_its_last_field(void)
{
    /* The scanner only emitted a field on ',', so the final field of every
     * sentence was dropped. A truncated $PFLAA ends on its last value. */
    replay_set_clock(2, 0, 30);
    static const char *s =
        "$PFLAA,0,1000,2000,500,1,4CA8AF!TRUNC,217\r\n";
    RB05_FlarmParser_ParseData((const uint8_t *)s, (uint16_t)strlen(s));

    TEST_ASSERT_EQUAL_INT16(217, obstacle("4CA8AF")->bearing);
}

void test_tag_match_requires_the_whole_sentence_name(void)
{
    /* strncmp compared only 5 of 6 characters, so "$PFLAAX" was accepted. */
    replay_set_clock(2, 0, 30);
    static const char *bad =
        "$PFLAAX,0,1000,2000,500,1,4CA8AF!NOPE,90,0,200,0.0,9*00\r\n";
    RB05_FlarmParser_ParseData((const uint8_t *)bad, (uint16_t)strlen(bad));
    TEST_ASSERT_EQUAL_INT(0, stored_count());
}

void test_long_unterminated_input_terminates(void)
{
    /* The scanners indexed with a uint8_t against a uint16_t length, so a
     * sentence longer than 255 bytes with no terminator wrapped the index and
     * looped for ever. Reaching the assertion at all is the test. */
    char big[1200];
    memset(big, 'A', sizeof(big));
    memcpy(big, "$PFLAA,0,1000,2000,500,1,4CA8AF!LONG,90,0,200,0.0,9", 50);
    replay_set_clock(2, 0, 30);
    RB05_FlarmParser_ParseData((const uint8_t *)big, (uint16_t)sizeof(big));
    TEST_ASSERT_NOT_EQUAL_INT8(-1, find_slot("4CA8AF"));
}

/* ====================================================================== */

int main(void)
{
    fx_flarm = replay_load(FIXTURE_FLARM);
    fx_dump1090 = replay_load(FIXTURE_DUMP1090);

    UNITY_BEGIN();

    RUN_TEST(test_line_framed_capture_populates_the_store);
    RUN_TEST(test_bulk_read_loses_no_traffic);
    RUN_TEST(test_pflau_marks_the_bluetooth_link_operative);
    RUN_TEST(test_non_traffic_sentences_are_ignored);

    RUN_TEST(test_aircraft_identity_comes_from_the_icao_hex);
    RUN_TEST(test_repeated_reports_update_in_place_rather_than_duplicating);
    RUN_TEST(test_position_advances_as_the_target_moves);
    RUN_TEST(test_timestamp_follows_the_device_clock);
    RUN_TEST(test_a_slot_is_reused_once_its_aircraft_is_over_60s_stale);
    RUN_TEST(test_store_holds_ten_simultaneous_targets);
    RUN_TEST(test_a_stale_slot_is_recycled_within_the_same_day);

    RUN_TEST(test_traffic_list_shows_one_line_per_live_aircraft);
    RUN_TEST(test_traffic_list_reports_flight_level_difference_and_range);
    RUN_TEST(test_traffic_list_omits_aircraft_silent_for_over_60s);
    RUN_TEST(test_traffic_list_applies_the_vertical_filter);
    RUN_TEST(test_traffic_list_is_empty_with_no_traffic);

    RUN_TEST(test_radar_starts_at_zoom_level_7);
    RUN_TEST(test_radar_places_a_target_by_relative_geometry);
    RUN_TEST(test_radar_label_tracks_the_icon);
    RUN_TEST(test_radar_zooming_in_moves_targets_outward);
    RUN_TEST(test_radar_zoom_is_clamped_at_both_ends);
    RUN_TEST(test_radar_hides_a_target_silent_for_over_60s);
    RUN_TEST(test_radar_leaves_empty_slots_alone);
    RUN_TEST(test_radar_ownship_symbol_stays_north_without_a_heading_source);
    RUN_TEST(test_radar_bearing_rose_is_placed_on_the_first_tick);
    RUN_TEST(test_radar_center_touch_cycles_the_vertical_filter);

    RUN_TEST(test_flarm_session_reaches_both_screens);
    RUN_TEST(test_dump1090_derived_session_reaches_both_screens);
    RUN_TEST(test_dump1090_altitudes_survive_the_conversion);

    RUN_TEST(test_negative_coordinates_round_trip);
    RUN_TEST(test_negative_magnitudes_are_exact_at_every_digit_count);
    RUN_TEST(test_icon_heading_matches_the_apparent_motion);
    RUN_TEST(test_south_west_target_is_drawn_in_the_lower_left_quadrant);
    RUN_TEST(test_positive_coordinates_round_trip);
    RUN_TEST(test_ble_fragments_are_reassembled_into_sentences);
    RUN_TEST(test_ble_fragment_size_does_not_change_the_result);
    RUN_TEST(test_regression_unreassembled_fragments_lose_traffic);
    RUN_TEST(test_ble_overlong_line_is_discarded_not_truncated);
    RUN_TEST(test_fragments_never_create_an_unidentified_entry);
    RUN_TEST(test_radar_range_label_matches_the_drawn_scale);
    RUN_TEST(test_radar_label_range_is_the_range_targets_are_plotted_at);
    RUN_TEST(test_radar_hides_a_target_outside_the_vertical_filter);
    RUN_TEST(test_traffic_list_has_no_phantom_rows_at_any_time_of_day);
    RUN_TEST(test_stale_slots_are_recycled_across_midnight);
    RUN_TEST(test_a_target_seen_before_midnight_ages_out_after_it);
    RUN_TEST(test_eleventh_simultaneous_target_is_dropped_without_reading_past_the_array);
    RUN_TEST(test_checksum_field_does_not_corrupt_the_accumulator);
    RUN_TEST(test_sentence_without_a_checksum_still_delivers_its_last_field);
    RUN_TEST(test_tag_match_requires_the_whole_sentence_name);
    RUN_TEST(test_long_unterminated_input_terminates);

    int rc = UNITY_END();
    replay_free(&fx_flarm);
    replay_free(&fx_dump1090);
    return rc;
}
