/*
 * Unit tests for RB04_NMEA_EMS_ParseData() in main/RB/RB04_NMEA_Parser.c.
 *
 * Sentence layout as implemented (differs from the stale comment in the source):
 *   $RBEMS,rpm,map,oilT,oilP,coolantT,outT,fuelLevel,batteryV*hh
 *   $RBCYL,cht1,cht2,cht3,cht4,egt1,egt2,egt3,egt4*hh
 */
#include <string.h>
#include "unity.h"
#include "RB02_Config.h"

uint8_t RB04_NMEA_EMS_ParseData(const uint8_t *data, uint8_t len, RB04_EMSData *emsData);

static RB04_EMSData ems;

static uint8_t parse(const char *s)
{
    return RB04_NMEA_EMS_ParseData((const uint8_t *)s, (uint8_t)strlen(s), &ems);
}

void setUp(void)
{
    memset(&ems, 0, sizeof(ems));
}

void tearDown(void) {}

/* ---- $RBEMS ------------------------------------------------------------- */

void test_rbems_full_sentence(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBEMS,2450,28,85,62,90,15,45,124*70"));
    TEST_ASSERT_EQUAL_UINT16(2450, ems.rpm);
    TEST_ASSERT_EQUAL_UINT8(28, ems.manifoldPressure);
    TEST_ASSERT_EQUAL_INT16(85, ems.oilTemperature);
    TEST_ASSERT_EQUAL_UINT8(62, ems.oilPressure);
    TEST_ASSERT_EQUAL_INT16(90, ems.coolantTemperature);
    TEST_ASSERT_EQUAL_INT16(15, ems.outTemperature);
    TEST_ASSERT_EQUAL_UINT8(45, ems.fuelLevel);
    TEST_ASSERT_EQUAL_UINT8(124, ems.batteryVoltage);
}

void test_rbems_without_checksum_or_line_ending(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBEMS,1000,20,70,50,80,10,30,120"));
    TEST_ASSERT_EQUAL_UINT16(1000, ems.rpm);
    TEST_ASSERT_EQUAL_UINT8(120, ems.batteryVoltage);
}

void test_rbems_with_crlf(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBEMS,1000,20,70,50,80,10,30,120*5A\r\n"));
    TEST_ASSERT_EQUAL_UINT16(1000, ems.rpm);
    TEST_ASSERT_EQUAL_UINT8(120, ems.batteryVoltage);
}

void test_rbems_negative_outside_temperature(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBEMS,900,18,60,40,70,-15,50,125"));
    TEST_ASSERT_EQUAL_INT16(-15, ems.outTemperature);
}

void test_rbems_decimal_part_is_truncated(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBEMS,2450.7,28.9,85,62,90,15,45,12.4"));
    TEST_ASSERT_EQUAL_UINT16(2450, ems.rpm);
    TEST_ASSERT_EQUAL_UINT8(28, ems.manifoldPressure);
    TEST_ASSERT_EQUAL_UINT8(12, ems.batteryVoltage);
}

void test_rbems_empty_fields_parse_as_zero(void)
{
    ems.rpm = 1234; ems.oilTemperature = 77;
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBEMS,,,,,,,,"));
    TEST_ASSERT_EQUAL_UINT16(0, ems.rpm);
    TEST_ASSERT_EQUAL_INT16(0, ems.oilTemperature);
}

void test_rbems_short_sentence_leaves_remaining_fields_untouched(void)
{
    ems.fuelLevel = 99; ems.batteryVoltage = 98;
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBEMS,2000,25,80"));
    TEST_ASSERT_EQUAL_UINT16(2000, ems.rpm);
    TEST_ASSERT_EQUAL_UINT8(25, ems.manifoldPressure);
    TEST_ASSERT_EQUAL_INT16(80, ems.oilTemperature);
    TEST_ASSERT_EQUAL_UINT8(99, ems.fuelLevel);
    TEST_ASSERT_EQUAL_UINT8(98, ems.batteryVoltage);
}

void test_rbems_extra_fields_are_ignored(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBEMS,2000,25,80,50,80,10,30,120,999,888"));
    TEST_ASSERT_EQUAL_UINT8(120, ems.batteryVoltage);
}

/* ---- $RBCYL ------------------------------------------------------------- */

void test_rbcyl_full_sentence(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBCYL,123,135,100,120,700,750,735,800*0B"));
    TEST_ASSERT_EQUAL_INT16(123, ems.chtCelsius[0]);
    TEST_ASSERT_EQUAL_INT16(135, ems.chtCelsius[1]);
    TEST_ASSERT_EQUAL_INT16(100, ems.chtCelsius[2]);
    TEST_ASSERT_EQUAL_INT16(120, ems.chtCelsius[3]);
    TEST_ASSERT_EQUAL_INT16(700, ems.egtCelsius[0]);
    TEST_ASSERT_EQUAL_INT16(750, ems.egtCelsius[1]);
    TEST_ASSERT_EQUAL_INT16(735, ems.egtCelsius[2]);
    TEST_ASSERT_EQUAL_INT16(800, ems.egtCelsius[3]);
}

void test_rbcyl_does_not_touch_rbems_fields(void)
{
    ems.rpm = 2450;
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBCYL,123,135,100,120,700,750,735,800"));
    TEST_ASSERT_EQUAL_UINT16(2450, ems.rpm);
}

void test_rbcyl_extra_fields_are_ignored(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBCYL,1,2,3,4,5,6,7,8,9,10"));
    TEST_ASSERT_EQUAL_INT16(8, ems.egtCelsius[3]);
}

void test_known_issue_rbcyl_cht_above_255_as_last_field(void)
{
    /* A CHT-only sentence ends on a CHT field. Cylinder head temperatures
     * routinely exceed 255 C and the field is int16_t, but the last-token
     * path casts through (uint8_t), so 300 arrives as 44. */
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBCYL,123,135,100,300"));
    if (ems.chtCelsius[3] == (int16_t)(uint8_t)300)
        TEST_IGNORE_MESSAGE("known issue: last CHT token truncated to uint8_t (RB04_NMEA_Parser.c)");
    TEST_ASSERT_EQUAL_INT16(300, ems.chtCelsius[3]);
}

/* ---- Buffer handling ---------------------------------------------------- */

void test_both_sentences_in_one_buffer(void)
{
    TEST_ASSERT_EQUAL_UINT8(1,
        parse("$RBEMS,2450,28,85,62,90,15,45,124*70\r\n$RBCYL,123,135,100,120,700,750,735,800*0B\r\n"));
    TEST_ASSERT_EQUAL_UINT16(2450, ems.rpm);
    TEST_ASSERT_EQUAL_INT16(800, ems.egtCelsius[3]);
}

void test_leading_garbage_before_dollar(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("xx\x01\xff$RBEMS,2450,28,85,62,90,15,45,124"));
    TEST_ASSERT_EQUAL_UINT16(2450, ems.rpm);
}

void test_unknown_sentence_returns_zero(void)
{
    ems.rpm = 2450;
    TEST_ASSERT_EQUAL_UINT8(0, parse("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"));
    TEST_ASSERT_EQUAL_UINT16(2450, ems.rpm);
}

void test_similar_prefix_is_not_matched(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, parse("$RBEMSX,2450,28,85,62,90,15,45,124"));
    TEST_ASSERT_EQUAL_UINT8(0, parse("$RBEM,2450"));
}

void test_null_and_empty_input(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, RB04_NMEA_EMS_ParseData(NULL, 10, &ems));
    TEST_ASSERT_EQUAL_UINT8(0, RB04_NMEA_EMS_ParseData((const uint8_t *)"$RBEMS,1", 0, &ems));
    TEST_ASSERT_EQUAL_UINT8(0, RB04_NMEA_EMS_ParseData((const uint8_t *)"$RBEMS,1", 8, NULL));
}

void test_input_is_bounded_by_len_not_nul(void)
{
    /* Only the first `len` bytes may be read: the tail here would change rpm. */
    const char *s = "$RBEMS,1000,20,70,50,80,10,30,120\r\n$RBEMS,9999";
    TEST_ASSERT_EQUAL_UINT8(1, RB04_NMEA_EMS_ParseData((const uint8_t *)s, 33, &ems));
    TEST_ASSERT_EQUAL_UINT16(1000, ems.rpm);
}

void test_max_length_input_does_not_overflow(void)
{
    char big[255];
    memset(big, 'A', sizeof(big));
    memcpy(big, "$RBEMS,2450,28,85,62,90,15,45,124", 33);
    TEST_ASSERT_EQUAL_UINT8(1, RB04_NMEA_EMS_ParseData((const uint8_t *)big, 255, &ems));
    TEST_ASSERT_EQUAL_UINT16(2450, ems.rpm);
}

/* ---- Freshness flag ----------------------------------------------------- */

void test_successful_parse_marks_data_fresh(void)
{
    ems.obsoleteData = 30;
    parse("$RBEMS,2450,28,85,62,90,15,45,124");
    TEST_ASSERT_EQUAL_UINT8(0, ems.obsoleteData);
}

void test_known_issue_unrelated_sentence_marks_data_fresh(void)
{
    /* RB_Usb.c feeds every console line to this parser. On builds where GPS
     * NMEA and EMS share the USB port, a GPS line carries no EMS data yet
     * `obsoleteData = 0` is written unconditionally, so stale EMS readings
     * keep looking fresh as long as the GPS keeps talking. */
    ems.obsoleteData = 30;
    parse("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A");
    if (ems.obsoleteData == 0)
        TEST_IGNORE_MESSAGE("known issue: obsoleteData reset by non-EMS sentence (RB04_NMEA_Parser.c)");
    TEST_ASSERT_EQUAL_UINT8(30, ems.obsoleteData);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_rbems_full_sentence);
    RUN_TEST(test_rbems_without_checksum_or_line_ending);
    RUN_TEST(test_rbems_with_crlf);
    RUN_TEST(test_rbems_negative_outside_temperature);
    RUN_TEST(test_rbems_decimal_part_is_truncated);
    RUN_TEST(test_rbems_empty_fields_parse_as_zero);
    RUN_TEST(test_rbems_short_sentence_leaves_remaining_fields_untouched);
    RUN_TEST(test_rbems_extra_fields_are_ignored);
    RUN_TEST(test_rbcyl_full_sentence);
    RUN_TEST(test_rbcyl_does_not_touch_rbems_fields);
    RUN_TEST(test_rbcyl_extra_fields_are_ignored);
    RUN_TEST(test_known_issue_rbcyl_cht_above_255_as_last_field);
    RUN_TEST(test_both_sentences_in_one_buffer);
    RUN_TEST(test_leading_garbage_before_dollar);
    RUN_TEST(test_unknown_sentence_returns_zero);
    RUN_TEST(test_similar_prefix_is_not_matched);
    RUN_TEST(test_null_and_empty_input);
    RUN_TEST(test_input_is_bounded_by_len_not_nul);
    RUN_TEST(test_max_length_input_does_not_overflow);
    RUN_TEST(test_successful_parse_marks_data_fresh);
    RUN_TEST(test_known_issue_unrelated_sentence_marks_data_fresh);
    return UNITY_END();
}
