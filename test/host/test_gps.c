/*
 * GPS NMEA input: the $GPRMC / $GPGGA scanners and NMEA_ParseBuffer() in
 * main/RB/RB02.c, from a single sentence up to a replayed receiver session.
 *
 * RB02.c cannot be compiled on the host as a whole, so CMakeLists.txt cuts the
 * GPS section out of it at configure time (gen/rb02_gps_slice.inc) and this
 * file #includes it after defining the globals the rest of RB02.c provides.
 * The code under test is the firmware's own text.
 *
 * Transports, framed as on the device:
 *   - USB (RB_Usb.c): one sentence per call, line ending stripped, NUL at len.
 *   - UART / NavCore (uart_fetch_data()): whatever arrived since the previous
 *     100 ms tick, NUL-terminated. Nothing aligns those reads to sentences.
 */
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unity.h"
#include "host_stubs.h"
#include "esp_timer.h"
#include "fixture_lines.h"
#include "RB02_Config.h"
#include "RB02_GUIHelpers.h"

/* ---- What the rest of RB02.c would define ----------------------------------- */
uint8_t Operative_GPS;
uint8_t Operative_BMP280;
int32_t Variometer;
int32_t bmp280Pressure;
float QNH;
int64_t GPSDT;
int64_t GPSLastSpeedKmhReceivedTick;
float FusionAHRSDeltaTrackFromGPS;
float GPSLastTrack;
float GPSLastSpeedKmhForAttitudeComponesation;
float GPSCurrentSpeedKmhForAttitudeComponesation;
bool GPSAccelerationForAttitudeCompensationEnabled;
/* RB02_DriverFactory.c in the firmware; read by the attitude filter. */
float GPSLateralYAcceleration;
float GPSAccelerationForAttitudeCompensation;

static RB02_Status status;
RB02_Status *singletonConfig(void) { return &status; }

/* Static in RB02.c, where it refreshes the altimeter labels. */
static int qnh_updates;
static void RB02_AltimeterQNHUpdated(void) { qnh_updates++; }

#ifdef RB_ENABLE_GPS_DIAG
lv_obj_t *GPSDiag_NMEADebugLine, *GPSDiag_NMEADebugRMC, *GPSDiag_NMEADebugGGA, *GPSDiag_NMEADebugSummary;
#ifndef UART_RX_BUF_SIZE
#define UART_RX_BUF_SIZE 1024
#endif
#endif

/* The NMEA passthrough (settingsEnableNMEAOutput) is a printf. */
static char nmea_out[4096];
static size_t nmea_out_len;
static int capture_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(nmea_out + nmea_out_len, sizeof(nmea_out) - nmea_out_len, fmt, ap);
    va_end(ap);
    if (n > 0)
        nmea_out_len += (size_t)n;
    if (nmea_out_len >= sizeof(nmea_out))
        nmea_out_len = sizeof(nmea_out) - 1;
    return n;
}

/* uart_fetch_data() needs the UART driver; the tests below play its part. */
#undef RB_ENABLE_UART
#define printf capture_printf
#include "rb02_gps_slice.inc"
#undef printf
#undef G

/* ---- Helpers ------------------------------------------------------------------ */

#define TICK_MS 100
#define READ_MAX 1024 /* UART_RX_BUF_SIZE */
#ifndef RB01_GPS_PROTOCOL_USB
#define RB01_GPS_PROTOCOL_USB 3 /* RB02_Defines.h, only under RB_ENABLE_USB_NMEA */
#endif

static gps_t *gps(void) { return &status.NMEA_DATA; }

/* "$<body>*hh\r\n" with a valid checksum. Eight rotating buffers. */
static const char *nmea(const char *body)
{
    static char buf[8][128];
    static unsigned slot;
    char *out = buf[slot++ % 8];
    snprintf(out, sizeof(buf[0]), "$%s*%02X\r\n", body, nmeaChecksum(body));
    return out;
}

/* One read handed to NMEA_ParseBuffer() the way uart_fetch_data() does it:
 * copied into the read buffer and NUL-terminated. Does not touch the clock. */
static void parse_read(const char *data, size_t n, uint8_t source)
{
    static uint8_t buf[READ_MAX + 1];
    TEST_ASSERT_TRUE(n <= READ_MAX);
    memcpy(buf, data, n);
    buf[n] = 0;
    if (n > 1)
        NMEA_ParseBuffer(buf, (int)n, source);
}

/* One tick later, one read of `n` bytes. */
static void read_bytes(const char *data, size_t n)
{
    host_stub_advance_ms(TICK_MS);
    parse_read(data, n, RB01_GPS_PROTOCOL_UART);
}

static void feed(const char *s) { read_bytes(s, strlen(s)); }

static void feed_at_ms(uint32_t ms, const char *s)
{
    host_stub_set_time_us((int64_t)ms * 1000);
    parse_read(s, strlen(s), RB01_GPS_PROTOCOL_UART);
}

static float kmh(float kt) { return kt * 1.852f; }

void setUp(void)
{
    memset(&status, 0, sizeof(status));
    Operative_GPS = 0;
    Operative_BMP280 = 0;
    Variometer = 0;
    bmp280Pressure = 101325;
    QNH = 1013.0f;
    GPSDT = 0;
    GPSLastSpeedKmhReceivedTick = -20000000; /* RB02.c's initial value */
    FusionAHRSDeltaTrackFromGPS = 0;
    GPSLastTrack = 0;
    GPSLastSpeedKmhForAttitudeComponesation = 0;
    GPSCurrentSpeedKmhForAttitudeComponesation = 0;
    GPSAccelerationForAttitudeCompensationEnabled = true;
    GPSLateralYAcceleration = 0;
    GPSAccelerationForAttitudeCompensation = 0;
    qnh_updates = 0;
    nmea_out_len = 0;
    nmea_out[0] = '\0';
    host_stub_set_time_us(10 * 1000000LL);
    host_stub_set_time_autoadvance_us(0);
    host_stub_lvgl_reset();
}

void tearDown(void) {}

/* ---- $GPRMC ------------------------------------------------------------------ */

void test_rmc_sets_time_position_speed_and_track(void)
{
    /* The trailing "003.1,W" is magnetic variation: its W must not touch the
     * longitude hemisphere. */
    feed(nmea("GPRMC,123519.00,A,4807.03800,N,01131.00000,E,022.4,084.4,230394,003.1,W"));
    TEST_ASSERT_EQUAL_UINT8(12, gps()->tim.hour);
    TEST_ASSERT_EQUAL_UINT8(35, gps()->tim.minute);
    TEST_ASSERT_EQUAL_UINT8(19, gps()->tim.second);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 48.1173f, gps()->latitude);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 11.516667f, gps()->longitude);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kmh(22.4f), gps()->speed);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 84.4f, gps()->cog);
}

void test_rmc_south_and_west_are_negative(void)
{
    feed(nmea("GNRMC,134500.00,A,3427.18000,S,05835.40000,W,0.010,52.10,271026,,,A"));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -34.453f, gps()->latitude);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -58.590f, gps()->longitude);
}

void test_rmc_void_fix_keeps_last_position(void)
{
    feed(nmea("GPRMC,123519.00,A,4807.03800,N,01131.00000,E,022.4,084.4,230394,,,A"));
    feed(nmea("GPRMC,123520.00,V,,,,,,,230394,,,N"));
    TEST_ASSERT_EQUAL_UINT8(20, gps()->tim.second);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 48.1173f, gps()->latitude);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 11.516667f, gps()->longitude);
}

void test_rmc_scanner_length_spans_the_checksum(void)
{
    /* NMEA_ParseBuffer() uses the length for the passthrough and to skip ahead,
     * so the scanner must not stop at '*'. */
    const char *s = nmea("GPRMC,123519.00,A,4807.03800,N,01131.00000,E,022.4,084.4,230394,,,A");
    TEST_ASSERT_EQUAL_UINT16(strlen(s) - 2, nmea_RMC_mini_parser((const uint8_t *)s, (uint16_t)strlen(s)));
}

void test_rmc_checksum_does_not_reach_the_last_field(void)
{
    /* Track is the last field before '*' here. Before inChecksum the hex digits
     * were folded into it. */
    feed(nmea("GPRMC,123519.00,A,4807.03800,N,01131.00000,E,022.4,084.4"));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 84.4f, gps()->cog);
}

void test_rmc_truncated_read_does_not_deliver_a_partial_field(void)
{
    feed(nmea("GPRMC,123519.00,A,4807.03800,N,01131.00000,E,022.4,300.0,230394,,,A"));
    const char *cut = "$GPRMC,123520.00,A,4807.03800,N,01131.00000,E,023.0,08";
    nmea_RMC_mini_parser((const uint8_t *)cut, (uint16_t)strlen(cut));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kmh(23.0f), gps()->speed);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 300.0f, gps()->cog); /* not 8.0 */
}

void test_rmc_refreshes_the_gps_timeout(void)
{
    /* rb_increase_lvgl_tick() raises the GPS INOP 5 s after this stamp. */
    feed_at_ms(42000, nmea("GPRMC,123519.00,A,4807.03800,N,01131.00000,E,022.4,084.4,230394,,,A"));
    TEST_ASSERT_EQUAL_INT64(42000000, GPSLastSpeedKmhReceivedTick);
}

void test_rmc_time_with_centiseconds(void)
{
    feed(nmea("GNRMC,061513.00,A,3427.18000,S,05835.40000,W,0.010,52.10,271026,,,A"));
    TEST_ASSERT_EQUAL_UINT8(6, gps()->tim.hour);
    TEST_ASSERT_EQUAL_UINT8(15, gps()->tim.minute);
    TEST_ASSERT_EQUAL_UINT8(13, gps()->tim.second);
}

void test_known_issue_rmc_time_assumes_two_decimals(void)
{
    /* nmea_RMC_UpdatedValueFor() case 1 splits the time field as if it always
     * had exactly two decimals (u-blox "hhmmss.ss") and ignores decimalCounter.
     * MediaTek-based receivers send "hhmmss.sss", others plain "hhmmss"; for
     * those the UTC clock on the Clock screen (update_Clock_lvgl_tick) shows
     * 61:51 or 00:06 instead of 06:15. Fix: divide finalNumber by
     * 10^decimalCounter first, then split hh/mm/ss. */
    static const char *const bodies[] = {
        "GNRMC,061513.000,A,3427.18000,S,05835.40000,W,0.010,52.10,271026,,,A",
        "GNRMC,061513,A,3427.18000,S,05835.40000,W,0.010,52.10,271026,,,A",
    };
    bool wrong = false;
    for (size_t i = 0; i < sizeof(bodies) / sizeof(bodies[0]); i++) {
        memset(&gps()->tim, 0, sizeof(gps()->tim));
        feed(nmea(bodies[i]));
        wrong |= gps()->tim.hour != 6 || gps()->tim.minute != 15 || gps()->tim.second != 13;
    }
    if (wrong)
        TEST_IGNORE_MESSAGE("known issue: RMC time assumes hhmmss.ss, wrong for hhmmss.sss and hhmmss (RB02.c)");
    TEST_ASSERT_FALSE(wrong);
}

/* ---- $GPGGA ------------------------------------------------------------------ */

void test_gga_fix_quality_satellites_and_hdop(void)
{
    feed(nmea("GNGGA,134500.00,3427.18000,S,05835.40000,W,1,09,0.92,3.0,M,13.2,M,,"));
    TEST_ASSERT_EQUAL_UINT8(1, Operative_GPS);
    TEST_ASSERT_TRUE(gps()->valid);
    TEST_ASSERT_EQUAL_INT(GPS_FIX_GPS, gps()->fix);
    TEST_ASSERT_EQUAL_INT(GPS_MODE_3D, gps()->fix_mode);
    TEST_ASSERT_EQUAL_UINT8(9, gps()->sats_in_view);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.92f, gps()->dop_h);
}

void test_gga_lost_fix_clears_valid(void)
{
    feed(nmea("GNGGA,134500.00,3427.18000,S,05835.40000,W,1,09,0.92,3.0,M,13.2,M,,"));
    feed(nmea("GNGGA,134501.00,,,,,0,00,99.99,,,,,,"));
    TEST_ASSERT_EQUAL_UINT8(0, Operative_GPS);
    TEST_ASSERT_FALSE(gps()->valid);
    TEST_ASSERT_EQUAL_INT(GPS_MODE_INVALID, gps()->fix_mode);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, gps()->altitude); /* empty field: kept */
}

void test_gga_altitude_seeds_then_smooths(void)
{
    feed(nmea("GPGGA,123519.00,4807.03800,N,01131.00000,E,1,08,0.9,100.0,M,46.9,M,,"));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, gps()->altitude);
    feed(nmea("GPGGA,123520.00,4807.03800,N,01131.00000,E,1,08,0.9,160.0,M,46.9,M,,"));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 110.0f, gps()->altitude); /* (160 + 5*100) / 6 */
}

void test_gga_altitude_below_sea_level_keeps_its_sign(void)
{
    /* Before the sign flag, '-' seeded the accumulator with -1. */
    feed(nmea("GPGGA,123519.00,5218.48000,N,00445.84000,E,1,08,0.9,-3.4,M,46.9,M,,"));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -3.4f, gps()->altitude);
}

/* ---- Auto QNH (GGA altitude + BMP280 pressure) -------------------------------- */

static const char *gga_300m(void)
{
    return nmea("GPGGA,123519.00,4807.03800,N,01131.00000,E,1,08,0.9,300.0,M,46.9,M,,");
}

static void auto_qnh_ready(float qnh)
{
    status.settingsAutoQNH = 1;
    Operative_BMP280 = 1;
    bmp280Pressure = 97800; /* RB02_SuggestedQNH(300 m, 97800 Pa) == 1013 */
    QNH = qnh;
}

void test_auto_qnh_adopts_a_nearby_suggestion(void)
{
    auto_qnh_ready(1008.0f);
    feed(gga_300m());
    TEST_ASSERT_EQUAL_FLOAT(1013.0f, QNH);
    TEST_ASSERT_EQUAL_INT(1, qnh_updates);
}

void test_auto_qnh_leaves_qnh_alone(void)
{
    struct { const char *why; float qnh; int32_t vario; uint8_t enabled; } cases[] = {
        { "disabled",                 1008.0f,   0, 0 },
        { "within 1 hPa",             1012.0f,   0, 1 },
        { "10 hPa or more away",      1000.0f,   0, 1 },
        { "climbing or descending",   1008.0f, 150, 1 },
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        setUp();
        auto_qnh_ready(cases[i].qnh);
        status.settingsAutoQNH = cases[i].enabled;
        Variometer = cases[i].vario;
        feed(gga_300m());
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(cases[i].qnh, QNH, cases[i].why);
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, qnh_updates, cases[i].why);
    }
}

/* ---- Attitude compensation from successive fixes ------------------------------- */

void test_longitudinal_acceleration_from_successive_fixes(void)
{
    feed_at_ms(10000, nmea("GNRMC,134510.00,A,3427.18000,S,05835.40000,W,40.000,52.10,271026,,,A"));
    feed_at_ms(11000, nmea("GNRMC,134511.00,A,3427.17000,S,05835.39000,W,50.000,52.10,271026,,,A"));
    /* 10 kt in 1 s = 5.144 m/s^2 */
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 10.0f * 1.852f / 3.6f / 9.81f, GPSAccelerationForAttitudeCompensation);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, GPSLateralYAcceleration);
}

void test_lateral_acceleration_through_north(void)
{
    feed_at_ms(10000, nmea("GNRMC,134510.00,A,3427.18000,S,05835.40000,W,100.000,358.00,271026,,,A"));
    feed_at_ms(11000, nmea("GNRMC,134511.00,A,3427.15000,S,05835.40000,W,100.000,2.00,271026,,,A"));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 4.0f, FusionAHRSDeltaTrackFromGPS); /* not -356 */
    /* v * omega = 51.44 m/s * 4 deg/s */
    float expected = (100.0f * 1.852f / 3.6f) * (4.0f * 3.14159265f / 180.0f) / 9.81f;
    TEST_ASSERT_FLOAT_WITHIN(0.005f, expected, GPSLateralYAcceleration);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, GPSAccelerationForAttitudeCompensation);
}

void test_compensation_switched_off_stays_zero(void)
{
    GPSAccelerationForAttitudeCompensationEnabled = false;
    feed_at_ms(10000, nmea("GNRMC,134510.00,A,3427.18000,S,05835.40000,W,40.000,358.00,271026,,,A"));
    feed_at_ms(11000, nmea("GNRMC,134511.00,A,3427.17000,S,05835.39000,W,50.000,2.00,271026,,,A"));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, GPSAccelerationForAttitudeCompensation);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, GPSLateralYAcceleration);
}

void test_known_issue_back_to_back_fixes_saturate_compensation(void)
{
    /* GPSDT is the gap between two nmea_RMC_mini_parser() calls, not between
     * the fixes' UTC stamps. Two $GPRMC in one read - a 5-10 Hz receiver, or
     * any stall of the 100 ms tick - are parsed microseconds apart, so an
     * ordinary 1 kt/s, 1.5 deg/s change computes as hundreds of g and lands on
     * the +-1.5 g / +-2 g clamps. QMI8658.c subtracts both from the
     * accelerometer before the Madgwick update until the next fix. Fix: take DT
     * from the RMC time field (already parsed in case 1). */
    feed_at_ms(10000, nmea("GNRMC,134510.00,A,3427.18000,S,05835.40000,W,100.000,52.00,271026,,,A"));

    char read[256];
    snprintf(read, sizeof(read), "%s%s",
             nmea("GNRMC,134510.10,A,3427.17700,S,05835.39600,W,100.100,52.15,271026,,,A"),
             nmea("GNRMC,134510.20,A,3427.17400,S,05835.39200,W,100.200,52.30,271026,,,A"));
    host_stub_set_time_us(10100000);
    host_stub_set_time_autoadvance_us(40); /* time spent parsing one sentence */
    parse_read(read, strlen(read), RB01_GPS_PROTOCOL_UART);

    float along = 1.0f * 1.852f / 3.6f / 9.81f;                                       /* 1 kt/s */
    float across = (100.2f * 1.852f / 3.6f) * (1.5f * 3.14159265f / 180.0f) / 9.81f;  /* 1.5 deg/s */
    if (fabsf(GPSAccelerationForAttitudeCompensation - along) > 0.02f ||
        fabsf(GPSLateralYAcceleration - across) > 0.02f) {
        static char msg[160];
        snprintf(msg, sizeof(msg),
                 "known issue: two fixes in one read give %.2f g / %.2f g instead of %.2f g / %.2f g (RB02.c GPSDT)",
                 GPSAccelerationForAttitudeCompensation, GPSLateralYAcceleration, along, across);
        TEST_IGNORE_MESSAGE(msg);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.02f, along, GPSAccelerationForAttitudeCompensation);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, across, GPSLateralYAcceleration);
}

/* ---- NMEA_ParseBuffer() ------------------------------------------------------- */

static const char *burst_rmc(void) { return nmea("GNRMC,134500.00,A,3427.18000,S,05835.40000,W,70.000,52.10,271026,,,A"); }
static const char *burst_gga(void) { return nmea("GNGGA,134500.00,3427.18000,S,05835.40000,W,1,09,0.92,120.0,M,13.2,M,,"); }

/* One second of u-blox output in the receiver's order. */
static size_t build_burst(char *out, size_t size)
{
    return (size_t)snprintf(out, size, "%s%s%s%s%s%s",
                            burst_rmc(),
                            nmea("GNVTG,52.10,T,,M,70.000,N,129.640,K,A"),
                            burst_gga(),
                            nmea("GNGSA,A,3,02,05,12,13,15,18,25,29,,,,,1.65,0.92,1.37"),
                            nmea("GPGSV,3,1,10,02,38,075,41,05,62,253,44,12,21,108,36,13,49,154,42"),
                            nmea("GNGLL,3427.18000,S,05835.40000,W,134500.00,A,A"));
}

void test_parse_buffer_picks_rmc_and_gga_out_of_a_burst(void)
{
    char burst[1024];
    feed_at_ms(10000, ""); /* nothing: rxBytes 0 */
    build_burst(burst, sizeof(burst));
    feed(burst);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kmh(70.0f), gps()->speed);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 52.1f, gps()->cog);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -34.453f, gps()->latitude);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 120.0f, gps()->altitude);
    TEST_ASSERT_EQUAL_UINT8(9, gps()->sats_in_view);
}

void test_parse_buffer_accepts_gp_and_gn_talkers(void)
{
    feed(nmea("GPRMC,123519.00,A,4807.03800,N,01131.00000,E,022.4,084.4,230394,,,A"));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 84.4f, gps()->cog);
    feed(nmea("GNRMC,123520.00,A,4807.03800,N,01131.00000,E,022.4,085.4,230394,,,A"));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 85.4f, gps()->cog);
    feed(nmea("GPGGA,123519.00,4807.03800,N,01131.00000,E,1,07,0.9,100.0,M,46.9,M,,"));
    TEST_ASSERT_EQUAL_UINT8(7, gps()->sats_in_view);
    feed(nmea("GNGGA,123520.00,4807.03800,N,01131.00000,E,1,11,0.9,100.0,M,46.9,M,,"));
    TEST_ASSERT_EQUAL_UINT8(11, gps()->sats_in_view);
}

void test_first_rmc_clears_the_no_data_warning(void)
{
    lv_obj_t *label = lv_label_create(NULL);
    status.ui.SettingStatus4UART = label;
    status.ui.SettingStatus4UARTIsFault = 1; /* as uartApplyRates() leaves it */
    feed(burst_gga());
    TEST_ASSERT_EQUAL_UINT8(1, status.ui.SettingStatus4UARTIsFault);
    feed(burst_rmc());
    TEST_ASSERT_EQUAL_UINT8(0, status.ui.SettingStatus4UARTIsFault);
    TEST_ASSERT_EQUAL_STRING("DATA RECEIVED", label->text);
}

void test_nmea_output_echoes_rmc_and_gga_only(void)
{
    char burst[1024], expected[256];
    status.settingsEnableNMEAOutput = 1;
    build_burst(burst, sizeof(burst));
    feed(burst);
    snprintf(expected, sizeof(expected), "%.*s\n%.*s\n",
             (int)strlen(burst_rmc()) - 2, burst_rmc(),
             (int)strlen(burst_gga()) - 2, burst_gga());
    TEST_ASSERT_EQUAL_STRING(expected, nmea_out);
}

void test_nmea_output_off_prints_nothing(void)
{
    char burst[1024];
    build_burst(burst, sizeof(burst));
    feed(burst);
    TEST_ASSERT_EQUAL_size_t(0, nmea_out_len);
}

#ifdef RB01_GPS_PROTOCOL_BLE
void test_bluetooth_gps_is_ignored_unless_selected(void)
{
    const char *s = burst_rmc();
    status.settingsBluetoothGPS = 0;
    parse_read(s, strlen(s), RB01_GPS_PROTOCOL_BLE);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, gps()->latitude);
    parse_read(s, strlen(s), RB01_GPS_PROTOCOL_UART);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -34.453f, gps()->latitude);

    setUp();
    status.settingsBluetoothGPS = 1;
    parse_read(s, strlen(s), RB01_GPS_PROTOCOL_BLE);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -34.453f, gps()->latitude);
}
#endif

/* ---- A sentence split across two reads ---------------------------------------- */

void test_known_issue_split_rmc_flips_the_hemisphere(void)
{
    /* Neither uart_fetch_data() path (UART driver or NavCore stream buffer)
     * reassembles sentences: a read hands NMEA_ParseBuffer() whatever bytes
     * arrived, and an $xxRMC cut by the read boundary is parsed up to the cut,
     * its tail dropped for lack of a '$'. Latitude and longitude are written
     * unsigned and only negated when the S/W field arrives, so a cut on or just
     * after that letter leaves the aircraft in the opposite hemisphere until
     * the next complete fix. Fix: reassemble lines before parsing, as
     * RB05_FlarmParser_FeedStream() does for BLE traffic. */
    char fix2[128], tail[256];
    snprintf(fix2, sizeof(fix2), "%s", nmea("GNRMC,134501.00,A,3427.15000,S,05835.36000,W,70.400,52.10,271026,,,A"));
    const char *fix1 = nmea("GNRMC,134500.00,A,3427.18000,S,05835.40000,W,70.000,52.10,271026,,,A");
    const char *next = nmea("GNVTG,52.10,T,,M,70.400,N,130.381,K,A");
    size_t len = strlen(fix2) - 2;
    int flips = 0, first = -1;

    for (size_t cut = 1; cut < len; cut++) {
        feed(fix1);
        TEST_ASSERT_TRUE(gps()->latitude < 0 && gps()->longitude < 0);
        read_bytes(fix2, cut);
        bool ok = gps()->latitude < 0 && gps()->longitude < 0;
        snprintf(tail, sizeof(tail), "%s%s", fix2 + cut, next);
        feed(tail);
        ok = ok && gps()->latitude < 0 && gps()->longitude < 0;
        if (!ok) {
            flips++;
            if (first < 0)
                first = (int)cut;
        }
    }
    if (flips) {
        static char msg[200];
        snprintf(msg, sizeof(msg),
                 "known issue: %d of %zu cut positions in a $GNRMC flip the hemisphere, first after \"...%.12s\"; reads are not reassembled (RB02.c)",
                 flips, len - 1, fix2 + first - 12);
        TEST_IGNORE_MESSAGE(msg);
    }
    TEST_ASSERT_EQUAL_INT(0, flips);
}

/* ---- Replaying a receiver session --------------------------------------------- */

typedef struct { float lat, lon, kmh, cog; uint8_t hh, mm, ss; } rmc_truth_t;

static char *session;
static size_t session_len;
static rmc_truth_t truth[64];
static int n_truth;
static float truth_altitude; /* GGA altitudes through the firmware's smoothing */

static int split_fields(const char *line, char fields[][24], int max)
{
    int n = 0, k = 0;
    for (const char *p = line + 1; *p && *p != '*' && *p != '\r' && *p != '\n'; p++) {
        if (*p == ',') {
            fields[n][k] = '\0';
            if (++n == max)
                return n;
            k = 0;
        } else if (k < 23) {
            fields[n][k++] = *p;
        }
    }
    fields[n][k] = '\0';
    return n + 1;
}

static double ddmm_to_degrees(const char *f)
{
    double v = strtod(f, NULL), d = floor(v / 100.0);
    return d + (v - d * 100.0) / 60.0;
}

/* Expected values come straight from the fixture text, with stdlib parsing. */
static void load_session(const char *path)
{
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        fprintf(stderr, "cannot open '%s'\n", path);
        abort();
    }
    static char raw[64 * 1024];
    size_t got = fread(raw, 1, sizeof(raw) - 1, fp);
    fclose(fp);
    raw[got] = '\0';

    session = malloc(got + 1);
    session_len = 0;
    for (char *line = raw; *line;) {
        char *eol = strchr(line, '\n');
        size_t n = eol ? (size_t)(eol - line + 1) : strlen(line);
        if (line[0] == '$') {
            memcpy(session + session_len, line, n);
            session_len += n;
            char f[20][24];
            int nf = split_fields(line, f, 20);
            if (strcmp(f[0] + 2, "RMC") == 0 && nf >= 9 && n_truth < 64) {
                rmc_truth_t *t = &truth[n_truth++];
                t->hh = (uint8_t)((f[1][0] - '0') * 10 + f[1][1] - '0');
                t->mm = (uint8_t)((f[1][2] - '0') * 10 + f[1][3] - '0');
                t->ss = (uint8_t)((f[1][4] - '0') * 10 + f[1][5] - '0');
                t->lat = (float)(ddmm_to_degrees(f[3]) * (f[4][0] == 'S' ? -1 : 1));
                t->lon = (float)(ddmm_to_degrees(f[5]) * (f[6][0] == 'W' ? -1 : 1));
                t->kmh = (float)(strtod(f[7], NULL) * 1.852);
                t->cog = (float)strtod(f[8], NULL);
            } else if (strcmp(f[0] + 2, "GGA") == 0 && nf >= 10 && f[9][0]) {
                float alt = (float)strtod(f[9], NULL);
                if (truth_altitude < 0.0001f)
                    truth_altitude = alt;
                truth_altitude = (alt + truth_altitude * 5.0f) / 6.0f;
            }
        }
        line += n;
    }
    session[session_len] = '\0';
}

typedef struct { int delivered, flips; } replay_result_t;

/* After each read: which fixes have been seen whole (speed and track together,
 * speeds are unique in the fixture), and did a known position change sign. */
static void observe(bool *seen, replay_result_t *r)
{
    bool have_fix = false;
    for (int i = 0; i < n_truth; i++) {
        if (!seen[i] && fabsf(gps()->speed - truth[i].kmh) < 1e-3f &&
            fabsf(gps()->cog - truth[i].cog) < 1e-3f) {
            seen[i] = true;
            r->delivered++;
        }
        have_fix |= seen[i];
    }
    if (have_fix && (gps()->latitude >= 0 || gps()->longitude >= 0))
        r->flips++;
}

/* RB_Usb.c: one sentence per call, CR/LF stripped, NUL-terminated. */
static replay_result_t replay_usb_lines(void)
{
    replay_result_t r = { 0, 0 };
    bool seen[64] = { false };
    for (size_t at = 0; at < session_len;) {
        const char *line = session + at;
        size_t n = strcspn(line, "\r\n");
        host_stub_advance_ms(10);
        parse_read(line, n, RB01_GPS_PROTOCOL_USB);
        observe(seen, &r);
        at += n;
        at += strspn(session + at, "\r\n");
    }
    return r;
}

/* uart_fetch_data() when every read happens to end in the idle gap between
 * the receiver's 1 Hz bursts: one whole second of output per read. */
static replay_result_t replay_whole_bursts(void)
{
    replay_result_t r = { 0, 0 };
    bool seen[64] = { false };
    const char *p = session;
    while (*p) {
        const char *next = strstr(p + 1, "$GNRMC");
        size_t n = next ? (size_t)(next - p) : strlen(p);
        host_stub_advance_ms(1000);
        parse_read(p, n, RB01_GPS_PROTOCOL_UART);
        observe(seen, &r);
        p += n;
    }
    return r;
}

/* uart_fetch_data() with reads of `size` bytes, one per tick. */
static replay_result_t replay_chunks(size_t size)
{
    replay_result_t r = { 0, 0 };
    bool seen[64] = { false };
    for (size_t at = 0; at < session_len; at += size) {
        size_t n = session_len - at < size ? session_len - at : size;
        read_bytes(session + at, n);
        observe(seen, &r);
    }
    return r;
}

static void assert_final_state(void)
{
    const rmc_truth_t *last = &truth[n_truth - 1];
    TEST_ASSERT_EQUAL_UINT8(last->hh, gps()->tim.hour);
    TEST_ASSERT_EQUAL_UINT8(last->mm, gps()->tim.minute);
    TEST_ASSERT_EQUAL_UINT8(last->ss, gps()->tim.second);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, last->lat, gps()->latitude);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, last->lon, gps()->longitude);
    /* Every GGA arrived, in order, or the smoothed altitude would differ. */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, truth_altitude, gps()->altitude);
    TEST_ASSERT_TRUE(gps()->valid);
}

void test_session_fixture_is_what_the_tests_assume(void)
{
    TEST_ASSERT_EQUAL_INT(45, n_truth);
    for (int i = 0; i < n_truth; i++) {
        TEST_ASSERT_TRUE(truth[i].lat < 0 && truth[i].lon < 0);
        if (i > 0)
            TEST_ASSERT_TRUE_MESSAGE(truth[i].kmh > truth[i - 1].kmh, "speeds must be unique");
    }
}

void test_usb_line_replay_delivers_every_fix(void)
{
    replay_result_t r = replay_usb_lines();
    TEST_ASSERT_EQUAL_INT(n_truth, r.delivered);
    TEST_ASSERT_EQUAL_INT(0, r.flips);
    assert_final_state();
}

void test_uart_replay_of_whole_bursts_delivers_every_fix(void)
{
    replay_result_t r = replay_whole_bursts();
    TEST_ASSERT_EQUAL_INT(n_truth, r.delivered);
    TEST_ASSERT_EQUAL_INT(0, r.flips);
    assert_final_state();
}

void test_known_issue_uart_reads_split_across_sentences_drop_fixes(void)
{
    /* Same root cause as test_known_issue_split_rmc_flips_the_hemisphere, seen
     * over a whole session: with reads that do not end between sentences, every
     * $GNRMC cut before its track field loses speed and track for that second.
     * Where reads end on hardware depends on the UART driver's FIFO thresholds
     * and the tick, so this sweeps every read size from 80 to 256 bytes rather
     * than modelling it; single sizes that divide the burst length lock the
     * cut to one spot in every sentence and say little on their own. */
    int lost = 0, total = 0, sizes = 0, sizes_flipped = 0;
    for (size_t size = 80; size <= 256; size++) {
        setUp();
        replay_result_t r = replay_chunks(size);
        lost += n_truth - r.delivered;
        total += n_truth;
        sizes++;
        sizes_flipped += r.flips > 0;
    }
    if (lost || sizes_flipped) {
        static char msg[220];
        snprintf(msg, sizeof(msg),
                 "known issue: reads of 80-256 B lose speed/track on %d%% of fixes, hemisphere flips at %d of %d sizes; reads are not reassembled (RB02.c)",
                 100 * lost / total, sizes_flipped, sizes);
        TEST_IGNORE_MESSAGE(msg);
    }
    TEST_ASSERT_EQUAL_INT(0, lost);
    TEST_ASSERT_EQUAL_INT(0, sizes_flipped);
}

/* ---- RB Simulator, profile rb09 -------------------------------------------- */

/* fixtures/rbsim/rb09.nmea is the RB Simulator feeding an RB-09 (its rb09
 * profile), recorded by tools/record_rbsim.sh: $GPRMC and $GPGGA at 5 Hz
 * among the master's $RBATT and $RBAIR, in a steady banked circle with no
 * wind. RB-09 builds have RB_ENABLE_USB_NMEA, so RB_Usb.c hands every one of
 * those lines to NMEA_ParseBuffer(). Each $GPRMC is parsed at the time it
 * carries, which is when the simulator sent it. */

static fixture_lines_t rbsim;

/* Field n of a recorded sentence, after its name */
static double recorded_field(const char *line, int n)
{
    char f[20][FIXTURE_FIELD_MAX];
    int nf = fixture_fields(line, f, 20);
    TEST_ASSERT_TRUE(n + 1 < nf);
    return strtod(f[n + 1], NULL);
}

static double rmc_seconds_of_day(const char *rmc)
{
    double t = recorded_field(rmc, 0);
    double hh = floor(t / 10000), mm = floor((t - hh * 10000) / 100);
    return hh * 3600 + mm * 60 + (t - hh * 10000 - mm * 100);
}

static const char *last_rbsim(const char *name)
{
    for (int i = rbsim.count - 1; i >= 0; i--)
        if (fixture_is(rbsim.line[i], name))
            return rbsim.line[i];
    TEST_FAIL_MESSAGE("sentence not in the recording");
    return NULL;
}

static int rbsim_echoed; /* $GPRMC / $GPGGA the NMEA output passed through */

/* Line by line as RB_Usb.c delivers them. Asserts on the way that the
 * master's own sentences neither change the GPS nor reach the NMEA output.
 * `fix` runs after each $GPRMC. */
static void replay_rbsim(void (*fix)(const char *rmc, int n))
{
    double t0 = -1;
    int fixes = 0;
    rbsim_echoed = 0;
    for (int i = 0; i < rbsim.count; i++)
    {
        const char *line = rbsim.line[i];
        bool rmc = fixture_is(line, "GPRMC"), gga = fixture_is(line, "GPGGA");
        if (rmc)
        {
            double t = rmc_seconds_of_day(line);
            if (t0 < 0)
                t0 = t;
            host_stub_set_time_us(10 * 1000000LL + llround((t - t0) * 1e6));
        }
        gps_t before = *gps();
        nmea_out_len = 0;
        nmea_out[0] = '\0';
        parse_read(line, rbsim.len[i], RB01_GPS_PROTOCOL_USB);
        if (rmc || gga)
        {
            rbsim_echoed += nmea_out_len > 0;
        }
        else
        {
            TEST_ASSERT_EQUAL_MEMORY_MESSAGE(&before, gps(), sizeof(before), line);
            TEST_ASSERT_EQUAL_size_t_MESSAGE(0, nmea_out_len, line);
        }
        if (rmc && fix != NULL)
            fix(line, fixes);
        fixes += rmc;
    }
}

static void check_fix(const char *rmc, int n)
{
    (void)n;
    double t = recorded_field(rmc, 0);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(t / 10000), gps()->tim.hour);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)fmod(t / 100, 100), gps()->tim.minute);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)fmod(t, 100), gps()->tim.second);
    char f[20][FIXTURE_FIELD_MAX];
    fixture_fields(rmc, f, 20);
    double lat = ddmm_to_degrees(f[3]) * (f[4][0] == 'S' ? -1 : 1);
    double lon = ddmm_to_degrees(f[5]) * (f[6][0] == 'W' ? -1 : 1);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, (float)lat, gps()->latitude);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, (float)lon, gps()->longitude);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (float)(recorded_field(rmc, 6) * 1.852), gps()->speed);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (float)recorded_field(rmc, 7), gps()->cog);
}

void test_rbsim_rb09_gps_every_fix_is_delivered(void)
{
    status.settingsEnableNMEAOutput = 1;
    replay_rbsim(check_fix);
    const char *gga = last_rbsim("GPGGA");
    TEST_ASSERT_TRUE(gps()->valid);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)recorded_field(gga, 6), gps()->sats_in_view);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, (float)recorded_field(gga, 7), gps()->dop_h);
}

/* The NMEA output passes a UART receiver's sentences on to USB, but not GPS
 * that itself came in on USB: RB_Usb.c strips the line ending, so the
 * scanner's length is the whole line and the `parsedLength < rxBytes - x`
 * guard in NMEA_ParseBuffer() skips the echo. That keeps an RB-09, or an
 * RB-02 / RB-04 USB build, from sending the master its own GPS back on the
 * same port. Nothing in the code says whether that is the intent, so this
 * pins what it does: change it deliberately or not at all. */
void test_rbsim_rb09_gps_from_usb_is_not_echoed_back(void)
{
    status.settingsEnableNMEAOutput = 1;
    replay_rbsim(NULL);
    TEST_ASSERT_TRUE(gps()->valid);
    TEST_ASSERT_EQUAL_INT(0, rbsim_echoed);
}

/* In a coordinated level turn the GPS sees a centripetal g tan(bank), which
 * is what nmea_RMC_UpdatedValueFor() hands the attitude filter as the
 * lateral acceleration; the speed is steady, so the longitudinal one is 0.
 * The bank is the $RBATT roll of the same recording. A track rounded to
 * 0.1 deg 5 times a second is worth +-0.05 g on a single fix. */
static double lateral_sum;
static int lateral_samples;

static void check_compensation(const char *rmc, int n)
{
    (void)rmc;
    if (n < 2) /* the first two fixes have no previous track or speed */
        return;
    double bank = recorded_field(last_rbsim("RBATT"), 0) * M_PI / 180;
    TEST_ASSERT_FLOAT_WITHIN(0.06f, (float)tan(bank), GPSLateralYAcceleration);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, GPSAccelerationForAttitudeCompensation);
    lateral_sum += GPSLateralYAcceleration;
    lateral_samples++;
}

void test_rbsim_rb09_gps_turn_compensation_matches_the_bank(void)
{
    lateral_sum = 0;
    lateral_samples = 0;
    replay_rbsim(check_compensation);
    double bank = recorded_field(last_rbsim("RBATT"), 0) * M_PI / 180;
    TEST_ASSERT_TRUE(lateral_samples >= 50);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, (float)tan(bank), (float)(lateral_sum / lateral_samples));
}

int main(void)
{
    load_session(RB_TEST_DIR "/fixtures/gps_session.nmea");

    UNITY_BEGIN();
    RUN_TEST(test_rmc_sets_time_position_speed_and_track);
    RUN_TEST(test_rmc_south_and_west_are_negative);
    RUN_TEST(test_rmc_void_fix_keeps_last_position);
    RUN_TEST(test_rmc_scanner_length_spans_the_checksum);
    RUN_TEST(test_rmc_checksum_does_not_reach_the_last_field);
    RUN_TEST(test_rmc_truncated_read_does_not_deliver_a_partial_field);
    RUN_TEST(test_rmc_refreshes_the_gps_timeout);
    RUN_TEST(test_rmc_time_with_centiseconds);
    RUN_TEST(test_known_issue_rmc_time_assumes_two_decimals);

    RUN_TEST(test_gga_fix_quality_satellites_and_hdop);
    RUN_TEST(test_gga_lost_fix_clears_valid);
    RUN_TEST(test_gga_altitude_seeds_then_smooths);
    RUN_TEST(test_gga_altitude_below_sea_level_keeps_its_sign);
    RUN_TEST(test_auto_qnh_adopts_a_nearby_suggestion);
    RUN_TEST(test_auto_qnh_leaves_qnh_alone);

    RUN_TEST(test_longitudinal_acceleration_from_successive_fixes);
    RUN_TEST(test_lateral_acceleration_through_north);
    RUN_TEST(test_compensation_switched_off_stays_zero);
    RUN_TEST(test_known_issue_back_to_back_fixes_saturate_compensation);

    RUN_TEST(test_parse_buffer_picks_rmc_and_gga_out_of_a_burst);
    RUN_TEST(test_parse_buffer_accepts_gp_and_gn_talkers);
    RUN_TEST(test_first_rmc_clears_the_no_data_warning);
    RUN_TEST(test_nmea_output_echoes_rmc_and_gga_only);
    RUN_TEST(test_nmea_output_off_prints_nothing);
#ifdef RB01_GPS_PROTOCOL_BLE
    RUN_TEST(test_bluetooth_gps_is_ignored_unless_selected);
#endif

    RUN_TEST(test_known_issue_split_rmc_flips_the_hemisphere);

    RUN_TEST(test_session_fixture_is_what_the_tests_assume);
    RUN_TEST(test_usb_line_replay_delivers_every_fix);
    RUN_TEST(test_uart_replay_of_whole_bursts_delivers_every_fix);
    RUN_TEST(test_known_issue_uart_reads_split_across_sentences_drop_fixes);

    rbsim = fixture_lines_load(RB_TEST_DIR "/fixtures/rbsim/rb09.nmea");
    RUN_TEST(test_rbsim_rb09_gps_every_fix_is_delivered);
    RUN_TEST(test_rbsim_rb09_gps_from_usb_is_not_echoed_back);
    RUN_TEST(test_rbsim_rb09_gps_turn_compensation_matches_the_bank);
    fixture_lines_free(&rbsim);
    return UNITY_END();
}
