/*
 * Unit tests for the pure helpers in main/RB/RB02_GUIHelpers.c:
 * NMEA coordinate conversion, NMEA checksum, QNH suggestion, model string.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "unity.h"
#include "RB02_GUIHelpers.h"

/* Not exported by any header; RB02.c declares it locally the same way. */
const char *RB_Generate_ModelString(char *buffer, size_t bufferSize);

void setUp(void) {}
void tearDown(void) {}

/* ---- nmea_to_decimal ---------------------------------------------------- */

void test_nmea_to_decimal_latitude(void)
{
    /* 4311.11936 = 43 deg 11.11936 min */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 43.185323f, nmea_to_decimal(4311.11936f));
}

void test_nmea_to_decimal_longitude_with_leading_zero(void)
{
    /* 01207.5 = 12 deg 7.5 min (leading zero is meaningless once numeric) */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.125f, nmea_to_decimal(1207.5f));
}

void test_nmea_to_decimal_zero(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, nmea_to_decimal(0.0f));
}

void test_nmea_to_decimal_whole_degrees(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 45.0f, nmea_to_decimal(4500.0f));
}

/* ---- nmeaChecksum ------------------------------------------------------- */

void test_checksum_classic_gga_example(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x47,
        nmeaChecksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"));
}

void test_checksum_classic_rmc_example(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x6A,
        nmeaChecksum("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"));
}

void test_checksum_leading_dollar_is_optional(void)
{
    TEST_ASSERT_EQUAL_HEX8(nmeaChecksum("$GPGGA,1,2"), nmeaChecksum("GPGGA,1,2"));
}

void test_checksum_stops_at_star(void)
{
    /* Bytes after '*' (the transmitted checksum) must not be folded in */
    TEST_ASSERT_EQUAL_HEX8(nmeaChecksum("GPGGA,1,2"), nmeaChecksum("GPGGA,1,2*FF"));
}

void test_checksum_stops_at_line_ending(void)
{
    TEST_ASSERT_EQUAL_HEX8(nmeaChecksum("GPGGA,1,2"), nmeaChecksum("GPGGA,1,2\r\n"));
    TEST_ASSERT_EQUAL_HEX8(nmeaChecksum("GPGGA,1,2"), nmeaChecksum("GPGGA,1,2\n"));
}

void test_checksum_null_and_empty(void)
{
    TEST_ASSERT_EQUAL_HEX8(0, nmeaChecksum(NULL));
    TEST_ASSERT_EQUAL_HEX8(0, nmeaChecksum(""));
    TEST_ASSERT_EQUAL_HEX8(0, nmeaChecksum("$"));
}

/* ---- RB02_SuggestedQNH -------------------------------------------------- */

void test_qnh_zero_gps_altitude_returns_standard(void)
{
    /* No GPS altitude -> no basis for a suggestion -> ISA standard */
    TEST_ASSERT_EQUAL_UINT16(1013, RB02_SuggestedQNH(0.0f, 95000));
    TEST_ASSERT_EQUAL_UINT16(1013, RB02_SuggestedQNH(0.0005f, 95000));
    TEST_ASSERT_EQUAL_UINT16(1013, RB02_SuggestedQNH(-0.0005f, 95000));
}

void test_qnh_standard_atmosphere_near_sea_level(void)
{
    /* ~300 m in ISA: 97.8 kPa static pressure -> QNH ~1013 */
    TEST_ASSERT_UINT16_WITHIN(2, 1013, RB02_SuggestedQNH(300.0f, 97800));
}

void test_qnh_standard_atmosphere_at_1000m(void)
{
    /* ISA at 1000 m: 89.9 kPa. The linear ft/hPa table is a coarse
     * approximation, so allow a few hPa. */
    TEST_ASSERT_UINT16_WITHIN(3, 1013, RB02_SuggestedQNH(1000.0f, 89875));
}

void test_qnh_increases_with_static_pressure(void)
{
    uint16_t low  = RB02_SuggestedQNH(500.0f, 94000);
    uint16_t high = RB02_SuggestedQNH(500.0f, 96000);
    TEST_ASSERT_GREATER_THAN_UINT16(low, high);
}

void test_qnh_increases_with_altitude_at_same_pressure(void)
{
    uint16_t low  = RB02_SuggestedQNH(200.0f, 95000);
    uint16_t high = RB02_SuggestedQNH(800.0f, 95000);
    TEST_ASSERT_GREATER_THAN_UINT16(low, high);
}

/* ---- RB02_CheckfileExists ----------------------------------------------- */

void test_checkfile_missing_returns_zero(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, RB02_CheckfileExists("/nonexistent/rb-host-test/file.txt"));
}

void test_checkfile_existing_returns_one(void)
{
    char path[] = "/tmp/rb02_checkfile_XXXXXX";
    int fd = mkstemp(path);
    TEST_ASSERT_TRUE_MESSAGE(fd >= 0, "mkstemp failed");
    close(fd);

    TEST_ASSERT_EQUAL_UINT8(1, RB02_CheckfileExists(path));

    unlink(path);
    TEST_ASSERT_EQUAL_UINT8(0, RB02_CheckfileExists(path));
}

/* ---- RB_Generate_ModelString -------------------------------------------- */

void test_model_string_is_terminated_and_fits(void)
{
    char buf[32];
    memset(buf, 'X', sizeof(buf));
    const char *r = RB_Generate_ModelString(buf, sizeof(buf));

    TEST_ASSERT_EQUAL_PTR(buf, r);
    TEST_ASSERT_LESS_THAN_size_t(sizeof(buf), strlen(buf));
}

void test_model_string_uses_only_known_feature_letters(void)
{
    char buf[32];
    RB_Generate_ModelString(buf, sizeof(buf));
    for (const char *p = buf; *p; p++)
        TEST_ASSERT_NOT_NULL_MESSAGE(strchr("TMVIAEGUFBD", *p), "unexpected feature letter");
}

void test_model_string_has_no_duplicate_letters(void)
{
    char buf[32];
    RB_Generate_ModelString(buf, sizeof(buf));
    for (const char *p = buf; *p; p++)
        TEST_ASSERT_NULL_MESSAGE(strchr(p + 1, *p), "duplicate feature letter");
}

void test_model_string_reflects_active_build_flags(void)
{
    /* Checked against whatever main/RB/BuildMachine.h currently selects. */
    char buf[32];
    RB_Generate_ModelString(buf, sizeof(buf));
#ifdef RB_02_DISPLAY_TOUCH
    TEST_ASSERT_NOT_NULL(strchr(buf, 'T'));
#else
    TEST_ASSERT_NULL(strchr(buf, 'T'));
#endif
#ifdef RB02_ESP_BLUETOOTH
    TEST_ASSERT_NOT_NULL(strchr(buf, 'B'));
#else
    TEST_ASSERT_NULL(strchr(buf, 'B'));
#endif
#ifdef RB_ENABLE_GPS
    TEST_ASSERT_NOT_NULL(strchr(buf, 'G'));
#else
    TEST_ASSERT_NULL(strchr(buf, 'G'));
#endif
}

void test_model_string_is_deterministic(void)
{
    char a[32], b[32];
    RB_Generate_ModelString(a, sizeof(a));
    RB_Generate_ModelString(b, sizeof(b));
    TEST_ASSERT_EQUAL_STRING(a, b);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nmea_to_decimal_latitude);
    RUN_TEST(test_nmea_to_decimal_longitude_with_leading_zero);
    RUN_TEST(test_nmea_to_decimal_zero);
    RUN_TEST(test_nmea_to_decimal_whole_degrees);
    RUN_TEST(test_checksum_classic_gga_example);
    RUN_TEST(test_checksum_classic_rmc_example);
    RUN_TEST(test_checksum_leading_dollar_is_optional);
    RUN_TEST(test_checksum_stops_at_star);
    RUN_TEST(test_checksum_stops_at_line_ending);
    RUN_TEST(test_checksum_null_and_empty);
    RUN_TEST(test_qnh_zero_gps_altitude_returns_standard);
    RUN_TEST(test_qnh_standard_atmosphere_near_sea_level);
    RUN_TEST(test_qnh_standard_atmosphere_at_1000m);
    RUN_TEST(test_qnh_increases_with_static_pressure);
    RUN_TEST(test_qnh_increases_with_altitude_at_same_pressure);
    RUN_TEST(test_checkfile_missing_returns_zero);
    RUN_TEST(test_checkfile_existing_returns_one);
    RUN_TEST(test_model_string_is_terminated_and_fits);
    RUN_TEST(test_model_string_uses_only_known_feature_letters);
    RUN_TEST(test_model_string_has_no_duplicate_letters);
    RUN_TEST(test_model_string_reflects_active_build_flags);
    RUN_TEST(test_model_string_is_deterministic);
    return UNITY_END();
}
