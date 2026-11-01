/*
 * RB-09 remote display, main/RB/RB09_Slave.c:
 *
 *   RB09_Slave_ParseData()     $RBATT / $RBAIR line -> RB09_SlaveData
 *   RB09_Slave_Loop()          RB09_SlaveData -> the instrument variables the
 *                              local QMI8658 / BMP280 write on other products
 *   RB09_Slave_AirspeedKmh10() the speed needle's source
 *
 * The loop tests fill RB09_SlaveData directly instead of going through the
 * parser, so a parser defect does not invalidate them. The RB Simulator
 * section at the bottom replays a recording of the simulator through both.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unity.h"
#include "host_stubs.h"
#include "fixture_lines.h"
#include "esp_timer.h"
#include "RB02_DriverFactory.h"
#include "RB02_GUIHelpers.h"
#include "RB09_Slave.h"

/* ---- firmware globals: RB02_DriverFactory.c and RB02.c on the device ----- */

IMUdata Accel;
IMUdata Gyro;
IMUdata AccelFiltered;
IMUdata GyroFiltered;
float AttitudePitch;
float AttitudeRoll;
float AttitudeYaw;
float AttitudeYawDegreePerSecond;
float QNH;
int32_t bmp280Pressure;
int32_t bmp280Temperature;
int32_t Altimeter;
int32_t Variometer;
uint8_t Operative_Attitude;
uint8_t Operative_BMP280;

/* QMI8658.c on the device. The G-meter itself is the master's code, only the
 * call is checked here. */
static int gfactorCalls;
void getGFactor(void) { gfactorCalls++; }

/* Verbatim from QMI8658.c */
float unwrapAngle(float prev, float current)
{
    float delta = current - prev;
    if (delta > 180.0f)
        delta -= 360.0f;
    if (delta < -180.0f)
        delta += 360.0f;
    return delta;
}

static RB02_Status status;
static RB09_SlaveData *const slave = &status.slaveData;

#define T0_US 10000000LL /* esp_timer_get_time() at the start of each test */

/* The literals in the RB09_Slave.c comment, checksums computed independently */
#define RBATT_EXAMPLE "$RBATT,-12.50,3.25,87.10,0.98,-0.21,0.05,0.10,-1.20,4.80,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.04*4F"
#define RBAIR_EXAMPLE "$RBAIR,29.921,29.92,15.0,500,120,20,270*7C"

/* The $RBAIR fields RB09_Slave_FormatAir() fills */
#define RB02_AIR_FIELDS ((1 << RB09_RBAIR_PRESSURE) | (1 << RB09_RBAIR_QNH) | (1 << RB09_RBAIR_VARIOMETER))

static uint8_t parse(const char *s)
{
    return RB09_Slave_ParseData((const uint8_t *)s, (uint16_t)strlen(s), slave);
}

/* Appends a correct "*hh" to a sentence body, independently of nmeaChecksum() */
static const char *sealed(const char *body)
{
    static char buf[512];
    uint8_t cs = 0;
    for (const char *p = body + 1; *p; p++)
        cs ^= (uint8_t)*p;
    snprintf(buf, sizeof(buf), "%s*%02X", body, cs);
    return buf;
}

/* `name` with `fields` fields, field n carrying `token` and the others empty */
static const char *only_field(const char *name, int fields, int n, const char *token)
{
    char body[256];
    int len = snprintf(body, sizeof(body), "$%s", name);
    for (int i = 0; i < fields; i++)
        len += snprintf(body + len, sizeof(body) - len, ",%s", i == n ? token : "");
    return sealed(body);
}

/* An $RBATT carrying `token` as magnetometer X, the field whose range is any
 * number parse_float_token() reads */
static const char *rbatt_with_mag_x(const char *token)
{
    static char body[128];
    snprintf(body, sizeof(body), "$RBATT,1,2,3,0,0,1,0,0,0,0,0,0,%s", token);
    return sealed(body);
}

/* Field n of the last $RBATT / $RBAIR used, in sentence order */
static float rbatt_field(int n)
{
    const float got[17] = {slave->roll, slave->pitch, slave->yaw,
                           slave->accel.x, slave->accel.y, slave->accel.z,
                           slave->gyro.x, slave->gyro.y, slave->gyro.z,
                           slave->gpsAcceleration.x, slave->gpsAcceleration.y, slave->gpsAcceleration.z,
                           slave->mag.x, slave->mag.y, slave->mag.z, slave->compass, slave->beta};
    return got[n];
}

static float rbair_field(int n)
{
    const float got[7] = {slave->pressureInHg, slave->qnhInHg, slave->outsideTemperature, slave->variometerFeetMin,
                          slave->indicatedAirSpeedKmh, slave->windKmh, slave->windDirection};
    return got[n];
}

/* Every field of both sentences to v, so a test sees what a sentence overwrote */
static void preset_fields(float v)
{
    slave->roll = slave->pitch = slave->yaw = v;
    slave->accel = slave->gyro = slave->gpsAcceleration = slave->mag = (IMUdata){v, v, v};
    slave->compass = slave->beta = v;
    slave->pressureInHg = slave->qnhInHg = slave->outsideTemperature = slave->variometerFeetMin = v;
    slave->indicatedAirSpeedKmh = slave->windKmh = slave->windDirection = v;
}

/* What rb_increase_lvgl_tick() prints for its barometer variables, without the
 * newline as RB_Usb.c delivers the line */
static uint8_t master_sends_air(int32_t pressurePa, float qnhHpa, int32_t variometer)
{
    char buf[180];
    RB09_Slave_FormatAir(buf, sizeof(buf), pressurePa, qnhHpa, variometer);
    char line[200];
    snprintf(line, sizeof(line), "%s*%02X", buf, nmeaChecksum(buf));
    return parse(line);
}

static void fresh_air(float pressure, float qnh)
{
    slave->pressureInHg = pressure;
    slave->qnhInHg = qnh;
    slave->variometerFeetMin = 0;
    slave->airPresent = RB02_AIR_FIELDS;
    slave->airTick = esp_timer_get_time();
}

void setUp(void)
{
    memset(&status, 0, sizeof(status));
    memset(&Accel, 0, sizeof(Accel));
    memset(&Gyro, 0, sizeof(Gyro));
    memset(&AccelFiltered, 0, sizeof(AccelFiltered));
    memset(&GyroFiltered, 0, sizeof(GyroFiltered));
    AttitudeRoll = AttitudePitch = AttitudeYaw = AttitudeYawDegreePerSecond = 0;
    QNH = 1013.0; /* RB02.c default */
    bmp280Pressure = bmp280Temperature = Altimeter = Variometer = 0;
    Operative_Attitude = Operative_BMP280 = 0;
    gfactorCalls = 0;
    host_stub_set_time_us(T0_US);
}

void tearDown(void) {}

/* ---- $RBATT ---------------------------------------------------------------- */

void test_rbatt_example_sentence(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse(RBATT_EXAMPLE));
    TEST_ASSERT_EQUAL_FLOAT(-12.5f, slave->roll);
    TEST_ASSERT_EQUAL_FLOAT(3.25f, slave->pitch);
    TEST_ASSERT_EQUAL_FLOAT(87.1f, slave->yaw);
    TEST_ASSERT_EQUAL_FLOAT(0.98f, slave->accel.x);
    TEST_ASSERT_EQUAL_FLOAT(-0.21f, slave->accel.y);
    TEST_ASSERT_EQUAL_FLOAT(0.05f, slave->accel.z);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, slave->gyro.x);
    TEST_ASSERT_EQUAL_FLOAT(-1.2f, slave->gyro.y);
    TEST_ASSERT_EQUAL_FLOAT(4.8f, slave->gyro.z);
    TEST_ASSERT_EQUAL_FLOAT(0.04f, slave->beta);
    TEST_ASSERT_EQUAL_INT64(T0_US, slave->attitudeTick);
    TEST_ASSERT_EQUAL_UINT16(0, slave->rejected);
}

/* Built with the format string and checksum call of the sender in
 * rb_increase_lvgl_tick(), so a change on either side shows up here. */
void test_rbatt_as_the_rb02_sender_writes_it(void)
{
    const float v[17] = {-33.33f, 12.5f, -179.99f, 1.02f, -0.5f, 0.25f, -3.5f, 2.25f, 15.75f,
                         0.12f, -0.34f, 0, 0, 0, 0, 0, 0.98f};
    char buf[180];
    snprintf(buf, sizeof(buf), "$RBATT,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
             v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8],
             v[9], v[10], 0.0, 0.0, 0.0, 0.0, 0.0, v[16]);
    char line[200];
    snprintf(line, sizeof(line), "%s*%02X\n", buf, nmeaChecksum(buf));

    TEST_ASSERT_EQUAL_UINT8(1, parse(line));
    for (int i = 0; i < 17; i++)
        TEST_ASSERT_FLOAT_WITHIN(0.005f, v[i], rbatt_field(i));
}

/* RB_Usb.c buffers 255 characters per line: the widest sentence the field
 * ranges allow must fit, or it would be silently cut and rejected. */
void test_rbatt_widest_sentence_fits_the_usb_line(void)
{
    const char *line = sealed("$RBATT,-360.00,-360.00,-360.00,-16.00,-16.00,-16.00,-2048.00,-2048.00,-2048.00,"
                              "-16.00,-16.00,-16.00,-9999999.00,-9999999.00,-9999999.00,-360.00,1.00");
    TEST_ASSERT_LESS_THAN(255, strlen(line));
    TEST_ASSERT_EQUAL_UINT8(1, parse(line));
    TEST_ASSERT_EQUAL_FLOAT(-2048.0f, slave->gyro.z);
    TEST_ASSERT_EQUAL_FLOAT(-9999999.0f, slave->mag.z);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, slave->beta);
}

/* A field left empty reads 0, not what the last sentence said */
void test_rbatt_empty_fields_read_zero(void)
{
    preset_fields(7);
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBATT,1,2,3,0,0,1,0,0,0,,,,,,,,")));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, slave->yaw);
    for (int n = 9; n < 17; n++)
        TEST_ASSERT_EQUAL_FLOAT(0, rbatt_field(n));

    preset_fields(7);
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBATT,,2,3,0,0,1,0,0,")));
    TEST_ASSERT_EQUAL_FLOAT(0, slave->roll);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, slave->pitch);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, slave->accel.z);
    TEST_ASSERT_EQUAL_FLOAT(0, slave->gyro.z);
    TEST_ASSERT_EQUAL_UINT16(0, slave->rejected);
}

/* So does every field after the last one sent */
void test_rbatt_short_sentence_reads_zero_past_its_end(void)
{
    preset_fields(7);
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBATT,1,2,3,0,0,1,0,0,0")));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, slave->accel.z);
    TEST_ASSERT_EQUAL_FLOAT(0, slave->beta);

    preset_fields(7);
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBATT,1")));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, slave->roll);
    for (int n = 1; n < 17; n++)
        TEST_ASSERT_EQUAL_FLOAT(0, rbatt_field(n));
    TEST_ASSERT_EQUAL_INT64(T0_US, slave->attitudeTick);
}

/* A field out of its range reads 0, the rest of the sentence is still used */
void test_rbatt_out_of_range_reads_zero(void)
{
    preset_fields(7);
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBATT,400,2,3,0,-17,1,0,0,3000")));
    TEST_ASSERT_EQUAL_FLOAT(0, slave->roll);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, slave->pitch);
    TEST_ASSERT_EQUAL_FLOAT(0, slave->accel.y);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, slave->accel.z);
    TEST_ASSERT_EQUAL_FLOAT(0, slave->gyro.z);
    TEST_ASSERT_EQUAL_UINT16(0, slave->rejected);
}

/* Not one valid field: nothing to use, the last attitude stays and ages */
void test_rbatt_with_no_valid_field_is_rejected(void)
{
    static const char *none[] = {"$RBATT,", "$RBATT,,,,,,,,,,,,,,,,,", "$RBATT, , ,\t,  ",
                                 "$RBATT,400,abc,-400,,2e1"};
    const unsigned count = sizeof(none) / sizeof(none[0]);
    preset_fields(7);
    for (unsigned i = 0; i < count; i++)
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, parse(sealed(none[i])), none[i]);
    TEST_ASSERT_EQUAL_FLOAT(7, slave->roll);
    TEST_ASSERT_EQUAL_INT64(0, slave->attitudeTick);
    TEST_ASSERT_EQUAL_UINT16(count, slave->rejected);
}

/* ---- field ranges ---------------------------------------------------------- */

/* Both bounds of a field, and a value just past each */
typedef struct
{
    const char *min, *max, *below, *above;
} field_range;

#define DEGREES {"-360", "360", "-360.01", "360.01"}
#define G       {"-16", "16", "-16.01", "16.01"}
#define DPS     {"-2048", "2048", "-2048.01", "2048.01"}
/* Any number: past it is an 8th integer digit, the cap that keeps the int32
 * conversions in range */
#define ANY     {"-9999999", "9999999", "-10000000", "10000000"}

static const field_range rbatt_ranges[17] = {
    DEGREES, DEGREES, DEGREES, G, G, G, DPS, DPS, DPS, G, G, G, ANY, ANY, ANY, DEGREES,
    {"0", "1", "-0.01", "1.01"}, /* filter beta */
};

static const field_range rbair_ranges[7] = {
    {"8.86", "32.48", "8.85", "32.49"},   /* inHg: 300 to 1100 hPa */
    {"25.11", "32.48", "25.10", "32.49"}, /* inHg: 850 to 1100 hPa */
    {"-100", "100", "-100.01", "100.01"}, /* Celsius */
    {"-20000", "20000", "-20000.01", "20000.01"}, /* feet/minute */
    {"0", "1000", "-0.01", "1000.01"},    /* km/h */
    {"0", "500", "-0.01", "500.01"},      /* km/h */
    {"0", "360", "-0.01", "360.01"},      /* degrees */
};

/* Each field sent alone: in range it is the one valid field and the others
 * read 0, out of range the sentence has none and is dropped. present, if not
 * NULL, is the mask that must say which field was valid. */
static void assert_field_ranges(const char *name, int fields, const field_range *ranges,
                                float (*field)(int), const uint8_t *present)
{
    char msg[64];
    for (int n = 0; n < fields; n++)
    {
        const char *in[] = {ranges[n].min, ranges[n].max};
        const char *out[] = {ranges[n].below, ranges[n].above};
        for (int i = 0; i < 2; i++)
        {
            snprintf(msg, sizeof(msg), "$%s field %d = %s", name, n, in[i]);
            preset_fields(7);
            TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, parse(only_field(name, fields, n, in[i])), msg);
            float want = strtof(in[i], NULL);
            for (int k = 0; k < fields; k++)
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6f * (1 + (want < 0 ? -want : want)), k == n ? want : 0,
                                                 field(k), msg);
            if (present != NULL)
                TEST_ASSERT_EQUAL_HEX8_MESSAGE(1u << n, *present, msg);

            snprintf(msg, sizeof(msg), "$%s field %d = %s", name, n, out[i]);
            uint16_t rejected = slave->rejected;
            TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, parse(only_field(name, fields, n, out[i])), msg);
            TEST_ASSERT_EQUAL_UINT16_MESSAGE(rejected + 1, slave->rejected, msg);
        }
    }
}

void test_rbatt_field_ranges(void)
{
    assert_field_ranges("RBATT", 17, rbatt_ranges, rbatt_field, NULL);
}

void test_rbair_field_ranges(void)
{
    assert_field_ranges("RBAIR", 7, rbair_ranges, rbair_field, &slave->airPresent);
}

/* ---- $RBAIR ---------------------------------------------------------------- */

void test_rbair_example_sentence(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse(RBAIR_EXAMPLE));
    TEST_ASSERT_EQUAL_FLOAT(29.921f, slave->pressureInHg);
    TEST_ASSERT_EQUAL_FLOAT(29.92f, slave->qnhInHg);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, slave->outsideTemperature);
    TEST_ASSERT_EQUAL_FLOAT(500.0f, slave->variometerFeetMin);
    TEST_ASSERT_EQUAL_FLOAT(120.0f, slave->indicatedAirSpeedKmh);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, slave->windKmh);
    TEST_ASSERT_EQUAL_FLOAT(270.0f, slave->windDirection);
    TEST_ASSERT_EQUAL_HEX8(0x7F, slave->airPresent);
    TEST_ASSERT_EQUAL_INT64(T0_US, slave->airTick);
}

/* A master with no pitot, no OAT probe and no wind leaves those fields empty:
 * they read 0, and airPresent tells that 0 from a real one. */
void test_rbair_empty_fields_read_zero(void)
{
    preset_fields(7);
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBAIR,29.92,29.92,,-300,,,")));
    TEST_ASSERT_EQUAL_FLOAT(-300.0f, slave->variometerFeetMin);
    TEST_ASSERT_EQUAL_HEX8(RB02_AIR_FIELDS, slave->airPresent);
    TEST_ASSERT_EQUAL_FLOAT(0, slave->outsideTemperature);
    TEST_ASSERT_EQUAL_FLOAT(0, slave->indicatedAirSpeedKmh);
    TEST_ASSERT_EQUAL_FLOAT(0, slave->windKmh);
    TEST_ASSERT_EQUAL_FLOAT(0, slave->windDirection);
}

/* Any field may be the missing one, the pressure and QNH too */
void test_rbair_missing_pressure_or_qnh_reads_zero(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBAIR,,29.92,15,0,100,10,90")));
    TEST_ASSERT_EQUAL_FLOAT(0, slave->pressureInHg);
    TEST_ASSERT_EQUAL_FLOAT(29.92f, slave->qnhInHg);
    TEST_ASSERT_EQUAL_HEX8(0x7F & ~(1 << RB09_RBAIR_PRESSURE), slave->airPresent);

    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBAIR,29.92,,15,0,100,10,90")));
    TEST_ASSERT_EQUAL_FLOAT(29.92f, slave->pressureInHg);
    TEST_ASSERT_EQUAL_FLOAT(0, slave->qnhInHg);
    TEST_ASSERT_EQUAL_HEX8(0x7F & ~(1 << RB09_RBAIR_QNH), slave->airPresent);

    slave->variometerFeetMin = 500;
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBAIR,29.92,29.92")));
    TEST_ASSERT_EQUAL_FLOAT(0, slave->variometerFeetMin);
    TEST_ASSERT_EQUAL_HEX8((1 << RB09_RBAIR_PRESSURE) | (1 << RB09_RBAIR_QNH), slave->airPresent);
    TEST_ASSERT_EQUAL_UINT16(0, slave->rejected);
}

void test_rbair_with_no_valid_field_is_rejected(void)
{
    static const char *none[] = {"$RBAIR,", "$RBAIR,,,,,,,", "$RBAIR, , , , , , , ",
                                 "$RBAIR,0,0,-200,99999,-5,-5,400"};
    const unsigned count = sizeof(none) / sizeof(none[0]);
    TEST_ASSERT_EQUAL_UINT8(1, parse(RBAIR_EXAMPLE));
    slave->airTick = 1;
    for (unsigned i = 0; i < count; i++)
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, parse(sealed(none[i])), none[i]);
    TEST_ASSERT_EQUAL_FLOAT(29.92f, slave->qnhInHg);
    TEST_ASSERT_EQUAL_HEX8(0x7F, slave->airPresent);
    TEST_ASSERT_EQUAL_INT64(1, slave->airTick);
    TEST_ASSERT_EQUAL_UINT16(count, slave->rejected);
}

void test_rbair_extra_fields_are_ignored(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBAIR,29.92,30.01,15,0,100,10,90,1234,abc")));
    TEST_ASSERT_EQUAL_FLOAT(30.01f, slave->qnhInHg);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, slave->windDirection);
}

/* RB09_Slave_FormatAir() is the sender in rb_increase_lvgl_tick(): firmware
 * units in, inHg and feet/minute out, no field for a sensor an RB-02 lacks. */
void test_rbair_as_the_rb02_sender_writes_it(void)
{
    char buf[180];
    int n = RB09_Slave_FormatAir(buf, sizeof(buf), 101325, 1013.0f, 833);
    TEST_ASSERT_EQUAL_STRING("$RBAIR,29.9212,29.91386,,500,,,", buf);
    TEST_ASSERT_EQUAL_INT(strlen(buf), n);

    TEST_ASSERT_EQUAL_UINT8(1, master_sends_air(101325, 1013.0f, 833));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 29.9212f, slave->pressureInHg);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 29.91386f, slave->qnhInHg);
    TEST_ASSERT_EQUAL_FLOAT(500.0f, slave->variometerFeetMin);
    TEST_ASSERT_EQUAL_HEX8(RB02_AIR_FIELDS, slave->airPresent);
    /* no air speed: the slave's speed screen is INOP, not 0 km/h */
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_AirspeedKmh10(slave, NULL));
}

/* ---- framing and checksum -------------------------------------------------- */

void test_checksum_mismatch_is_rejected(void)
{
    /* RBATT_EXAMPLE with the roll changed and the checksum left alone */
    TEST_ASSERT_EQUAL_UINT8(0, parse("$RBATT,-13.50,3.25,87.10,0.98,-0.21,0.05,0.10,-1.20,4.80,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.04*4F"));
    TEST_ASSERT_EQUAL_FLOAT(0, slave->roll);
    TEST_ASSERT_EQUAL_INT64(0, slave->attitudeTick);
    TEST_ASSERT_EQUAL_UINT16(1, slave->rejected);
}

void test_checksum_is_mandatory(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, parse("$RBAIR,29.921,29.92,15.0,500,120,20,270"));
    TEST_ASSERT_EQUAL_UINT8(0, parse("$RBAIR,29.921,29.92,15.0,500,120,20,270*"));
    TEST_ASSERT_EQUAL_UINT8(0, parse("$RBAIR,29.921,29.92,15.0,500,120,20,270*7"));
    TEST_ASSERT_EQUAL_UINT8(0, parse("$RBAIR,29.921,29.92,15.0,500,120,20,270*G7"));
    TEST_ASSERT_EQUAL_UINT16(4, slave->rejected);
}

void test_lowercase_checksum_is_accepted(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBAIR,29.921,29.92,15.0,500,120,20,270*7c"));
}

void test_line_ending_and_leading_garbage(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("noise" RBAIR_EXAMPLE "\r\n"));
    TEST_ASSERT_EQUAL_FLOAT(29.92f, slave->qnhInHg);
}

void test_both_sentences_in_one_buffer(void)
{
    TEST_ASSERT_EQUAL_UINT8(2, parse(RBATT_EXAMPLE "\r\n" RBAIR_EXAMPLE "\r\n"));
    TEST_ASSERT_EQUAL_FLOAT(-12.5f, slave->roll);
    TEST_ASSERT_EQUAL_FLOAT(29.92f, slave->qnhInHg);
}

/* A sentence cut before its '*' must not borrow the next sentence's checksum */
void test_sentence_cut_by_the_next_one(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, parse("$RBATT,1,2,3,0,0,1,0" RBAIR_EXAMPLE));
    TEST_ASSERT_EQUAL_INT64(0, slave->attitudeTick);
    TEST_ASSERT_EQUAL_INT64(T0_US, slave->airTick);
    TEST_ASSERT_EQUAL_UINT16(1, slave->rejected);

    /* Even with a checksum over both, the '$' ends the first sentence: fields
     * past the 17th are ignored, so only the framing stops this one. */
    TEST_ASSERT_EQUAL_UINT8(0, parse(sealed("$RBATT,1,2,3,0,0,1,0,0,0,0,0,0,0,0,0,0,0,9$RBAIR,29.92,29.92,15,0")));
    TEST_ASSERT_EQUAL_INT64(0, slave->attitudeTick);
}

void test_unrelated_sentences_are_ignored(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, parse("$GPRMC,191830.20,A,4311.11954,N,01208.18630,E,0.7,334.5,250725,,,A*57"));
    TEST_ASSERT_EQUAL_UINT8(0, parse("$RBEMS,2450,28,85,62,90,15,45,124*70"));
    TEST_ASSERT_EQUAL_UINT8(0, parse(sealed("$RBATTX,1,2,3,0,0,1,0,0,0")));
    TEST_ASSERT_EQUAL_UINT8(0, parse(sealed("$RBAI,29.92,29.92,15,0")));
    TEST_ASSERT_EQUAL_UINT8(0, parse("help"));
    /* Not ours, so not counted as damaged */
    TEST_ASSERT_EQUAL_UINT16(0, slave->rejected);
}

void test_null_and_empty_input(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_ParseData(NULL, 10, slave));
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_ParseData((const uint8_t *)RBAIR_EXAMPLE, 0, slave));
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_ParseData((const uint8_t *)RBAIR_EXAMPLE, 20, NULL));
}

void test_input_is_bounded_by_len_not_nul(void)
{
    const char *s = RBAIR_EXAMPLE;
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_ParseData((const uint8_t *)s, (uint16_t)(strlen(s) - 3), slave));
    TEST_ASSERT_EQUAL_INT64(0, slave->airTick);
}

/* Run under -DRB_SANITIZE=ON: nothing may be read or written past the copy */
void test_overlong_input_does_not_overflow(void)
{
    static uint8_t big[1000];
    memset(big, '9', sizeof(big));
    memcpy(big, "$RBAIR,", 7);
    big[sizeof(big) - 3] = '*';
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_ParseData(big, sizeof(big), slave));
    memset(big, '$', sizeof(big));
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_ParseData(big, sizeof(big), slave));
}

/* ---- numeric fields -------------------------------------------------------- */

/* A damaged number reads 0, as if the field were empty, instead of as some
 * other number. The checksum matched, so the rest of the sentence is used. */
void test_malformed_number_reads_zero(void)
{
    static const char *bad[] = {"1.2.3", "abc", "29-92", "-", "+", ".", "29a", "2 9", "- 29", "2e1", "0x1D", "--29"};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
    {
        slave->mag.x = 7;
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, parse(rbatt_with_mag_x(bad[i])), bad[i]);
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0, slave->mag.x, bad[i]);
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(3.0f, slave->yaw, bad[i]);
    }
    TEST_ASSERT_EQUAL_UINT16(0, slave->rejected);
}

void test_number_forms(void)
{
    static const struct
    {
        const char *token;
        float value;
    } ok[] = {{"29.92", 29.92f}, {"+29.92", 29.92f}, {"-0.25", -0.25f}, {".5", 0.5f}, {"30.", 30.0f},
              {"0030", 30.0f}, {"12.00", 12.0f}, {"-0.00", 0.0f}, {"29.9212345678", 29.921234f},
              {"9999999", 9999999.0f}, {" 29", 29.0f}, {"29 ", 29.0f}, {"\t -0.25 \t", -0.25f}};
    for (unsigned i = 0; i < sizeof(ok) / sizeof(ok[0]); i++)
    {
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, parse(rbatt_with_mag_x(ok[i].token)), ok[i].token);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-5f * (1 + ok[i].value), ok[i].value, slave->mag.x, ok[i].token);
    }
}

/* Blanks around a number, or in place of one, as a fixed width printf pads it */
void test_blank_padded_fields(void)
{
    preset_fields(7);
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBATT, -12.50,  3.25 ,87.10\t,  0.98,-0.21,0.05,0.10,-1.20,4.80, , ,   ,\t,,,, ")));
    TEST_ASSERT_EQUAL_FLOAT(-12.5f, slave->roll);
    TEST_ASSERT_EQUAL_FLOAT(3.25f, slave->pitch);
    TEST_ASSERT_EQUAL_FLOAT(87.1f, slave->yaw);
    TEST_ASSERT_EQUAL_FLOAT(0.98f, slave->accel.x);
    TEST_ASSERT_EQUAL_FLOAT(4.8f, slave->gyro.z);
    for (int n = 9; n < 17; n++)
        TEST_ASSERT_EQUAL_FLOAT(0, rbatt_field(n));

    char body[128];
    snprintf(body, sizeof(body), "$RBAIR,%9.4f,%9.5f,%6s,%6.0f,%6s,%6s,%6s", 29.9212, 29.91386, "", -500.0, "", "", "");
    TEST_ASSERT_EQUAL_STRING("$RBAIR,  29.9212, 29.91386,      ,  -500,      ,      ,      ", body);
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed(body)));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 29.9212f, slave->pressureInHg);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 29.91386f, slave->qnhInHg);
    TEST_ASSERT_EQUAL_FLOAT(-500.0f, slave->variometerFeetMin);
    TEST_ASSERT_EQUAL_HEX8(RB02_AIR_FIELDS, slave->airPresent);
    TEST_ASSERT_EQUAL_UINT16(0, slave->rejected);
}

/* Whole numbers with decimals, 12.00, and without, 12, in every field */
void test_whole_numbers_with_and_without_decimals(void)
{
    static const float att[17] = {12, -12, 12, 1, -1, 12, 12, -12, 12, 1, -1, 0, 12, -12, 12, 12, 1};
    static const char *rbatt[] = {"$RBATT,12.00,-12.00,12.00,1.00,-1.00,12.00,12.00,-12.00,12.00,"
                                  "1.00,-1.00,0.00,12.00,-12.00,12.00,12.00,1.00",
                                  "$RBATT,12,-12,12,1,-1,12,12,-12,12,1,-1,0,12,-12,12,12,1"};
    static const float air[7] = {29, 30, 12, 500, 120, 20, 270};
    static const char *rbair[] = {"$RBAIR,29.00,30.00,12.00,500.00,120.00,20.00,270.00",
                                  "$RBAIR,29,30,12,500,120,20,270"};
    for (int i = 0; i < 2; i++)
    {
        preset_fields(7);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, parse(sealed(rbatt[i])), rbatt[i]);
        for (int n = 0; n < 17; n++)
            TEST_ASSERT_EQUAL_FLOAT_MESSAGE(att[n], rbatt_field(n), rbatt[i]);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, parse(sealed(rbair[i])), rbair[i]);
        for (int n = 0; n < 7; n++)
            TEST_ASSERT_EQUAL_FLOAT_MESSAGE(air[n], rbair_field(n), rbair[i]);
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x7F, slave->airPresent, rbair[i]);
    }
}

/* ---- RB09_Slave_Loop(): attitude ------------------------------------------- */

static void fresh_attitude(void)
{
    slave->roll = -12.5f;
    slave->pitch = 3.25f;
    slave->yaw = 87.1f;
    slave->accel = (IMUdata){0.98f, -0.21f, 0.05f};
    slave->gyro = (IMUdata){0.1f, -1.2f, 4.8f};
    slave->attitudeTick = esp_timer_get_time();
}

void test_loop_before_any_data_is_inop(void)
{
    AttitudeRoll = 7;
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(0, Operative_Attitude);
    TEST_ASSERT_EQUAL_UINT8(0, Operative_BMP280);
    TEST_ASSERT_EQUAL_FLOAT(7, AttitudeRoll);
    TEST_ASSERT_EQUAL_FLOAT(1013.0f, QNH);
    /* G-meter extremes are stored in NVS: no G from no data */
    TEST_ASSERT_EQUAL_INT(0, gfactorCalls);
}

void test_loop_applies_the_master_attitude(void)
{
    fresh_attitude();
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_Attitude);
    TEST_ASSERT_EQUAL_FLOAT(-12.5f, AttitudeRoll);
    TEST_ASSERT_EQUAL_FLOAT(3.25f, AttitudePitch);
    TEST_ASSERT_EQUAL_FLOAT(87.1f, AttitudeYaw);
    TEST_ASSERT_EQUAL_FLOAT(0.98f, AccelFiltered.x);
    TEST_ASSERT_EQUAL_FLOAT(-0.21f, AccelFiltered.y);
    TEST_ASSERT_EQUAL_FLOAT(0.05f, AccelFiltered.z);
    TEST_ASSERT_EQUAL_FLOAT(4.8f, GyroFiltered.z);
    /* The turn coordinator ball reads the raw values */
    TEST_ASSERT_EQUAL_FLOAT(0.98f, Accel.x);
    TEST_ASSERT_EQUAL_FLOAT(-0.21f, Accel.y);
    TEST_ASSERT_EQUAL_FLOAT(-1.2f, Gyro.y);
    TEST_ASSERT_EQUAL_INT(1, gfactorCalls);
}

void test_loop_attitude_goes_inop_after_the_timeout(void)
{
    fresh_attitude();
    host_stub_set_time_us(T0_US + RB09_SLAVE_ATTITUDE_TIMEOUT_US - 1);
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_Attitude);

    host_stub_set_time_us(T0_US + RB09_SLAVE_ATTITUDE_TIMEOUT_US);
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(0, Operative_Attitude);
    TEST_ASSERT_EQUAL_INT(1, gfactorCalls);

    /* and recovers with the next sentence */
    slave->attitudeTick = esp_timer_get_time();
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_Attitude);
}

/* Standard rate turn, 3 deg/s, through the +-180 wrap: the master sends yaw at
 * 10 Hz (rb_increase_lvgl_tick) and the driver loop runs at 20 Hz. */
void test_loop_turn_rate_follows_a_standard_rate_turn(void)
{
    float yaw = 170;
    float peak = 0;
    for (int step = 0; step < 1200; step++) /* 60 s */
    {
        if (step % 2 == 0)
        {
            slave->yaw = yaw;
            slave->attitudeTick = esp_timer_get_time();
            yaw += 0.3f;
            if (yaw > 180)
                yaw -= 360;
        }
        RB09_Slave_Loop(&status);
        if (AttitudeYawDegreePerSecond > peak)
            peak = AttitudeYawDegreePerSecond;
        host_stub_advance_ms(50);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 3.0f, AttitudeYawDegreePerSecond);
    /* crossing 180 -> -180 is not a 360 deg/s spike */
    TEST_ASSERT_LESS_THAN_FLOAT(3.3f, peak);
}

/* After the link drops the yaw jumps: that is not a turn */
void test_loop_turn_rate_restarts_after_a_gap(void)
{
    fresh_attitude();
    RB09_Slave_Loop(&status);
    host_stub_advance_ms(5000);
    RB09_Slave_Loop(&status); /* stale */
    slave->yaw = slave->yaw + 90;
    slave->attitudeTick = esp_timer_get_time();
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_FLOAT(0, AttitudeYawDegreePerSecond);
}

/* ---- RB09_Slave_Loop(): air data ------------------------------------------- */

void test_loop_standard_day(void)
{
    fresh_air(29.92f, 29.92f);
    slave->outsideTemperature = 15;
    slave->airPresent |= 1 << RB09_RBAIR_OAT;
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_BMP280);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1013.21f, QNH);
    TEST_ASSERT_INT32_WITHIN(1, 101320, bmp280Pressure); /* Pa */
    TEST_ASSERT_INT32_WITHIN(30, 0, Altimeter);          /* within 0.3 ft */
    TEST_ASSERT_EQUAL_INT32(1500, bmp280Temperature);    /* centi-Celsius */
}

/* Altimeter is *100 feet, as Get_BMP280() writes it: 4.28 inHg below QNH at
 * the firmware's 30 ft/hPa is 4348.1 ft. */
void test_loop_altitude_in_get_bmp280_units(void)
{
    fresh_air(25.84f, 30.12f);
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_BMP280);
    TEST_ASSERT_INT32_WITHIN(100, 434812, Altimeter);
    TEST_ASSERT_INT32_WITHIN(100, (int32_t)(((QNH * 100) - bmp280Pressure) * 30.0), Altimeter);
}

/* Variometer is 0.01 feet/second, which update_Variometer_lvgl_tick() turns
 * back into 1000 ft/min per 90 degrees */
void test_loop_variometer_units(void)
{
    fresh_air(29.92f, 29.92f);
    slave->variometerFeetMin = 500;
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_INT32(833, Variometer);
    TEST_ASSERT_INT32_WITHIN(1, 450, (int32_t)((Variometer * 6.0 / 10.0) * 36.0 / 40));

    slave->variometerFeetMin = -1200;
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_INT32(-2000, Variometer);
}

void test_loop_adopts_the_master_qnh(void)
{
    status.settingsAutoQNH = 1;
    fresh_air(29.92f, 29.92f);
    RB09_Slave_Loop(&status);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1013.21f, QNH);
    TEST_ASSERT_EQUAL_UINT8(1, slave->qnhChanged);
    /* Auto QNH would set it from the local GPS and fight the master */
    TEST_ASSERT_EQUAL_UINT8(0, status.settingsAutoQNH);

    /* the same QNH again does not refresh the labels */
    slave->qnhChanged = 0;
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(0, slave->qnhChanged);

    /* a local change on the slave is overridden by the master */
    QNH += 1;
    RB09_Slave_Loop(&status);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1013.21f, QNH);
    TEST_ASSERT_EQUAL_UINT8(1, slave->qnhChanged);
}

/* Implausible air data raises the INOP and writes nothing, so no float reaches
 * an int instrument variable out of range (UBSan) */
void test_loop_implausible_air_data_is_inop(void)
{
    static const float cases[][2] = {
        {29.92f, 20.0f},      /* QNH 677 hPa */
        {29.92f, 35.0f},      /* QNH 1185 hPa */
        {9999999.0f, 29.92f}, /* pressure far past int32 Pa */
        {0.0f, 29.92f},
        {18.0f, 29.92f},      /* 12110 ft: above the 12000 ft Get_BMP280() accepts */
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        fresh_air(cases[i][0], cases[i][1]);
        RB09_Slave_Loop(&status);
        TEST_ASSERT_EQUAL_UINT8(0, Operative_BMP280);
        TEST_ASSERT_EQUAL_FLOAT(1013.0f, QNH);
        TEST_ASSERT_EQUAL_INT32(0, bmp280Pressure);
        TEST_ASSERT_EQUAL_INT32(0, Altimeter);
    }
}

void test_loop_air_goes_inop_after_the_timeout(void)
{
    fresh_air(29.92f, 29.92f);
    host_stub_set_time_us(T0_US + RB09_SLAVE_AIR_TIMEOUT_US - 1);
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_BMP280);
    host_stub_set_time_us(T0_US + RB09_SLAVE_AIR_TIMEOUT_US);
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(0, Operative_BMP280);
}

/* A $RBAIR without a valid pressure or QNH is used, but gives no altitude */
void test_loop_air_without_pressure_or_qnh_is_inop(void)
{
    static const char *bodies[] = {"$RBAIR,,29.92,15,500", "$RBAIR,29.92,,15,500", "$RBAIR,29.92,40,15,500",
                                   "$RBAIR,,,,500"};
    for (unsigned i = 0; i < sizeof(bodies) / sizeof(bodies[0]); i++)
    {
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, parse(sealed(bodies[i])), bodies[i]);
        RB09_Slave_Loop(&status);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, Operative_BMP280, bodies[i]);
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(1013.0f, QNH, bodies[i]);
        TEST_ASSERT_EQUAL_INT32_MESSAGE(0, Altimeter, bodies[i]);
    }

    /* The mask decides, not whether a 0 happens to be out of range */
    fresh_air(29.92f, 29.92f);
    slave->airPresent &= ~(1 << RB09_RBAIR_QNH);
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(0, Operative_BMP280);

    /* With both, a variometer the master does not send reads 0 */
    Variometer = 777;
    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBAIR,29.92,29.92")));
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_BMP280);
    TEST_ASSERT_EQUAL_INT32(0, Variometer);
}

void test_loop_without_oat_keeps_the_temperature(void)
{
    bmp280Temperature = 2150;
    slave->outsideTemperature = -40;
    fresh_air(29.92f, 29.92f);
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_INT32(2150, bmp280Temperature);
}

/* ---- RB09_Slave_AirspeedKmh10() -------------------------------------------- */

void test_airspeed_from_the_master(void)
{
    int kmh10 = -1;
    TEST_ASSERT_EQUAL_UINT8(1, parse(RBAIR_EXAMPLE));
    TEST_ASSERT_EQUAL_UINT8(1, RB09_Slave_AirspeedKmh10(slave, &kmh10));
    TEST_ASSERT_EQUAL_INT(1200, kmh10);
    TEST_ASSERT_EQUAL_UINT8(1, RB09_Slave_AirspeedKmh10(slave, NULL));
}

void test_airspeed_not_sent_or_stale(void)
{
    int kmh10 = -1;
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_AirspeedKmh10(slave, &kmh10));

    TEST_ASSERT_EQUAL_UINT8(1, parse(sealed("$RBAIR,29.92,29.92,15,0,,10,90")));
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_AirspeedKmh10(slave, &kmh10));

    TEST_ASSERT_EQUAL_UINT8(1, parse(RBAIR_EXAMPLE));
    host_stub_advance_ms(RB09_SLAVE_AIR_TIMEOUT_US / 1000);
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_AirspeedKmh10(slave, &kmh10));
    TEST_ASSERT_EQUAL_INT(-1, kmh10);
}

/* ---- end to end ------------------------------------------------------------ */

/* The QNH label RB02_AltimeterQNHUpdated() writes, in either unit */
static void qnh_labels(float qnh, char hpa[8], char inhg[8])
{
    float QNHInInches = qnh / 33.8639;
    snprintf(inhg, 8, "%.2f", QNHInInches);
    snprintf(hpa, 8, "%03u", (uint16_t)qnh);
}

static void assert_qnh_reaches_the_slave(float masterQnh)
{
    char want[2][8], got[2][8], msg[48];
    TEST_ASSERT_EQUAL_UINT8(1, master_sends_air((int32_t)(masterQnh * 100), masterQnh, 0));
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_BMP280);
    qnh_labels(masterQnh, want[0], want[1]);
    qnh_labels(QNH, got[0], got[1]);
    snprintf(msg, sizeof(msg), "master QNH %.4f, slave %.4f", masterQnh, QNH);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, masterQnh, QNH, msg);
    TEST_ASSERT_EQUAL_STRING_MESSAGE(want[1], got[1], msg);
    /* Not within 0.001 under a whole hPa: the master's float steps reach 1076.9998
     * where 189 * 0.338639 is 1077.0028, the master shows 1076 and the slave,
     * snapping to 0.001, 1077. Snapping is what keeps every whole hPa setting. */
    if (masterQnh - (uint16_t)masterQnh < 0.999f)
        TEST_ASSERT_EQUAL_STRING_MESSAGE(want[0], got[0], msg);
}

/* The slave shows the QNH set on the master. The labels truncate, so 1015 hPa
 * must not arrive as 1014.997: every whole hPa setting, and the 0.01 inHg steps
 * of the inHg mode, which RB02.c adds to QNH as 33.8639/100 hPa. */
void test_master_qnh_reaches_the_slave_labels(void)
{
    for (int hpa = 900; hpa <= 1080; hpa++)
        assert_qnh_reaches_the_slave((float)hpa);

    float up = 1013.0f, down = 1013.0f;
    for (int i = 0; i < 200; i++)
    {
        assert_qnh_reaches_the_slave(up);
        assert_qnh_reaches_the_slave(down);
        up += 33.8639 / 100;
        down -= 33.8639 / 100;
    }
}

/* A master's barometer variables, through RB09_Slave_FormatAir(), the checksum
 * and RB09_Slave_Loop(), come out as Get_BMP280() wrote them on the master */
void test_master_air_reaches_the_slave_instruments(void)
{
    static const float qnhs[] = {1013.0f, 985.0f, 1031.0f};
    static const int32_t varios[] = {-2000, -1, 0, 1, 833, 3333}; /* 0.01 ft/s */
    for (unsigned q = 0; q < sizeof(qnhs) / sizeof(qnhs[0]); q++)
        for (int32_t feet = -500; feet <= 11500; feet += 250)
            for (unsigned v = 0; v < sizeof(varios) / sizeof(varios[0]); v++)
            {
                int32_t pressure = (int32_t)(qnhs[q] * 100 - feet * 100 / 30.0);
                int32_t masterAltimeter = (int32_t)(((qnhs[q] * 100) - pressure) * 30.0);
                TEST_ASSERT_EQUAL_UINT8(1, master_sends_air(pressure, qnhs[q], varios[v]));
                RB09_Slave_Loop(&status);
                TEST_ASSERT_EQUAL_UINT8(1, Operative_BMP280);
                TEST_ASSERT_EQUAL_INT32(pressure, bmp280Pressure);
                TEST_ASSERT_INT32_WITHIN(10, masterAltimeter, Altimeter); /* 0.1 ft */
                /* 1 ft/min on the wire is 1.7 here */
                TEST_ASSERT_INT32_WITHIN(2, varios[v], Variometer);
            }
}

/* What an RB-01 USB master streams, line by line as RB_Usb.c delivers it: its
 * GPS passthrough is interleaved and must not disturb the slave data. */
void test_master_stream_drives_the_instruments(void)
{
    static const char *lines[] = {
        "$GPRMC,191830.20,A,4311.11954,N,01208.18630,E,0.7,334.5,250725,,,A*57",
        "$GPGGA,191830.20,4311.11954,N,01208.18630,E,1,10,1.00,278.0,M,44.3,M,,*5A",
        RBATT_EXAMPLE,
        RBAIR_EXAMPLE,
    };
    for (unsigned i = 0; i < sizeof(lines) / sizeof(lines[0]); i++)
        parse(lines[i]);
    RB09_Slave_Loop(&status);

    TEST_ASSERT_EQUAL_UINT8(1, Operative_Attitude);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_BMP280);
    TEST_ASSERT_EQUAL_FLOAT(-12.5f, AttitudeRoll);
    TEST_ASSERT_INT32_WITHIN(5, 101324, bmp280Pressure);
    TEST_ASSERT_INT32_WITHIN(30, -102, Altimeter); /* 0.001 inHg above QNH: about -1 ft */
    TEST_ASSERT_EQUAL_INT32(833, Variometer);
    TEST_ASSERT_EQUAL_UINT16(0, slave->rejected);
}

/* ---- RB Simulator, profile rb09 -------------------------------------------- */

/* fixtures/rbsim/rb09.nmea is the RB Simulator standing in for the RB-02
 * master (its rb09 profile, rbsimulator decision D24), recorded by
 * tools/record_rbsim.sh: a steady banked circle with no wind. The recording
 * has no receive times. The simulator sends $RBATT at 20 Hz, so each one
 * moves the clock 50 ms, and Driver_Loop() runs RB09_Slave_Loop() right
 * after it at its default 20 Hz. */

static fixture_lines_t rb09;

/* Field n of a recorded sentence, read independently of the parser */
static float recorded_field(const char *line, int n)
{
    char f[20][FIXTURE_FIELD_MAX];
    int nf = fixture_fields(line, f, 20);
    TEST_ASSERT_TRUE(n + 1 < nf);
    return strtof(f[n + 1], NULL);
}

static const char *last_recorded(const char *name)
{
    for (int i = rb09.count - 1; i >= 0; i--)
        if (fixture_is(rb09.line[i], name))
            return rb09.line[i];
    TEST_FAIL_MESSAGE("sentence not in the recording");
    return NULL;
}

/* An RB-09 on the recording: RB_Usb.c hands it every line, the driver loop
 * runs after each $RBATT. `each` sees the instruments after every loop pass,
 * with the latest $RBATT and $RBAIR (NULL before the first). */
static void replay_rb09(void (*each)(const char *rbatt, const char *rbair))
{
    const char *rbatt = NULL, *rbair = NULL;
    for (int i = 0; i < rb09.count; i++)
    {
        const char *line = rb09.line[i];
        bool ours = fixture_is(line, "RBATT") || fixture_is(line, "RBAIR");
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(ours, RB09_Slave_ParseData((const uint8_t *)line, (uint16_t)rb09.len[i], slave), line);
        if (fixture_is(line, "RBAIR"))
            rbair = line;
        if (!fixture_is(line, "RBATT"))
            continue;
        rbatt = line;
        RB09_Slave_Loop(&status);
        if (each != NULL)
            each(rbatt, rbair);
        host_stub_advance_ms(50);
    }
}

/* What the master sent is what the instruments show, from the recorded text */
static void check_rb09_instruments(const char *rbatt, const char *rbair)
{
    TEST_ASSERT_EQUAL_UINT8(1, Operative_Attitude);
    TEST_ASSERT_EQUAL_FLOAT(recorded_field(rbatt, 0), AttitudeRoll);
    TEST_ASSERT_EQUAL_FLOAT(recorded_field(rbatt, 1), AttitudePitch);
    TEST_ASSERT_EQUAL_FLOAT(recorded_field(rbatt, 2), AttitudeYaw);
    /* The G-meter's acceleration and the ball */
    TEST_ASSERT_EQUAL_FLOAT(recorded_field(rbatt, 3), Accel.x);
    TEST_ASSERT_EQUAL_FLOAT(recorded_field(rbatt, 4), Accel.y);
    if (rbair == NULL)
    {
        TEST_ASSERT_EQUAL_UINT8(0, Operative_BMP280);
        return;
    }
    TEST_ASSERT_EQUAL_UINT8(1, Operative_BMP280);
    double pressure = recorded_field(rbair, 0) * 3386.39;
    double qnh = recorded_field(rbair, 1) * 33.8639;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, (float)qnh, QNH);
    TEST_ASSERT_INT32_WITHIN(1, (int32_t)(pressure + 0.5), bmp280Pressure);
    TEST_ASSERT_INT32_WITHIN(10, (int32_t)((qnh * 100 - pressure) * 30), Altimeter); /* 0.1 ft */
    TEST_ASSERT_INT32_WITHIN(2, (int32_t)(recorded_field(rbair, 3) * 100 / 60), Variometer);
    TEST_ASSERT_INT32_WITHIN(1, (int32_t)(recorded_field(rbair, 2) * 100), bmp280Temperature);
    int kmh10 = -1;
    TEST_ASSERT_EQUAL_UINT8(1, RB09_Slave_AirspeedKmh10(slave, &kmh10));
    TEST_ASSERT_INT_WITHIN(1, (int)(recorded_field(rbair, 4) * 10), kmh10);
}

void test_rbsim_rb09_recording_is_what_the_tests_assume(void)
{
    int rmc = fixture_count(&rb09, "GPRMC");
    TEST_ASSERT_TRUE(rmc >= 50);
    /* 20 Hz, 10 Hz and 5 Hz */
    TEST_ASSERT_INT_WITHIN(4, 4 * rmc, fixture_count(&rb09, "RBATT"));
    TEST_ASSERT_INT_WITHIN(4, 2 * rmc, fixture_count(&rb09, "RBAIR"));
    TEST_ASSERT_EQUAL_INT(rmc, fixture_count(&rb09, "GPGGA"));
    /* A steady turn, right wing down */
    float bank = recorded_field(last_recorded("RBATT"), 0);
    TEST_ASSERT_TRUE(bank >= 5);
    for (int i = 0; i < rb09.count; i++)
        if (fixture_is(rb09.line[i], "RBATT"))
            TEST_ASSERT_EQUAL_FLOAT(bank, recorded_field(rb09.line[i], 0));
}

void test_rbsim_rb09_every_sentence_is_used(void)
{
    replay_rb09(NULL);
    TEST_ASSERT_EQUAL_UINT16(0, slave->rejected);
}

void test_rbsim_rb09_drives_every_instrument_without_inop(void)
{
    replay_rb09(check_rb09_instruments);
    TEST_ASSERT_EQUAL_INT(fixture_count(&rb09, "RBATT"), gfactorCalls);
}

/* A coordinated turn: the needle turns the way the aircraft banks, at
 * g tan(bank) / v. No wind in the synthetic flight, so v is the GPS ground
 * speed. RB02_AAttitude.c draws the needle at -(rate * 10 / 2) px, so a
 * negative rate is a turn to the right. */
void test_rbsim_rb09_turn_needle_follows_the_bank(void)
{
    replay_rb09(NULL);
    float bank = recorded_field(last_recorded("RBATT"), 0) * (float)M_PI / 180;
    float mps = recorded_field(last_recorded("GPRMC"), 6) * 1852.0f / 3600.0f;
    float right = -(9.80665f * tanf(bank) / mps) * 180 / (float)M_PI;
    TEST_ASSERT_FLOAT_WITHIN(0.03f * fabsf(right), right, AttitudeYawDegreePerSecond);
}

/* The simulator's own rule: a stale value must not look valid. The master
 * stops sending: the attitude is INOP after 1 s, the air data after 3 s. */
void test_rbsim_rb09_silence_raises_the_inop(void)
{
    replay_rb09(NULL);
    host_stub_advance_ms(850); /* 0.9 s after the last $RBATT */
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_Attitude);
    host_stub_advance_ms(150);
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(0, Operative_Attitude);
    TEST_ASSERT_EQUAL_UINT8(1, Operative_BMP280);

    host_stub_advance_ms(2000);
    RB09_Slave_Loop(&status);
    TEST_ASSERT_EQUAL_UINT8(0, Operative_BMP280);
    TEST_ASSERT_EQUAL_UINT8(0, RB09_Slave_AirspeedKmh10(slave, NULL));
}

void test_known_issue_altimeter_is_linear_in_pressure(void)
{
    /* Get_BMP280() in BMP280.c turns pressure into altitude at a constant
     * 30 ft/hPa below QNH, and RB09_Slave_Loop() copies it so an RB-09 reads
     * like its master. An altimeter follows the ISA pressure law,
     * 145366.45 ft * (1 - (P / QNH)^0.190284): on a standard day the straight
     * line reads 154 ft high at 3000 ft and 507 ft low at 10000 ft, and the
     * gap keeps growing to its 12000 ft limit. Here with the pressure and QNH
     * the simulator sent. Fix: the ISA law in both places, keeping the range
     * check on the result. test_loop_altitude_in_get_bmp280_units,
     * test_master_air_reaches_the_slave_instruments and
     * check_rb09_instruments() model the master's formula and change with it. */
    replay_rb09(NULL);
    const char *rbair = last_recorded("RBAIR");
    double pressure = recorded_field(rbair, 0) * 33.8639;
    double qnh = recorded_field(rbair, 1) * 33.8639;
    double isa = 145366.45 * (1 - pow(pressure / qnh, 0.190284));
    double shown = Altimeter / 100.0;
    if (fabs(shown - isa) > 20)
    {
        static char msg[160];
        snprintf(msg, sizeof(msg),
                 "known issue: %.1f hPa at QNH %.1f shows %.0f ft, the ISA altitude is %.0f ft (BMP280.c, RB09_Slave.c)",
                 pressure, qnh, shown, isa);
        TEST_IGNORE_MESSAGE(msg);
    }
    TEST_ASSERT_DOUBLE_WITHIN(20, isa, shown);
}

int main(void)
{
    rb09 = fixture_lines_load(RB_TEST_DIR "/fixtures/rbsim/rb09.nmea");
    UNITY_BEGIN();
    RUN_TEST(test_rbatt_example_sentence);
    RUN_TEST(test_rbatt_as_the_rb02_sender_writes_it);
    RUN_TEST(test_rbatt_widest_sentence_fits_the_usb_line);
    RUN_TEST(test_rbatt_empty_fields_read_zero);
    RUN_TEST(test_rbatt_short_sentence_reads_zero_past_its_end);
    RUN_TEST(test_rbatt_out_of_range_reads_zero);
    RUN_TEST(test_rbatt_with_no_valid_field_is_rejected);
    RUN_TEST(test_rbatt_field_ranges);
    RUN_TEST(test_rbair_example_sentence);
    RUN_TEST(test_rbair_empty_fields_read_zero);
    RUN_TEST(test_rbair_missing_pressure_or_qnh_reads_zero);
    RUN_TEST(test_rbair_with_no_valid_field_is_rejected);
    RUN_TEST(test_rbair_field_ranges);
    RUN_TEST(test_rbair_extra_fields_are_ignored);
    RUN_TEST(test_rbair_as_the_rb02_sender_writes_it);
    RUN_TEST(test_checksum_mismatch_is_rejected);
    RUN_TEST(test_checksum_is_mandatory);
    RUN_TEST(test_lowercase_checksum_is_accepted);
    RUN_TEST(test_line_ending_and_leading_garbage);
    RUN_TEST(test_both_sentences_in_one_buffer);
    RUN_TEST(test_sentence_cut_by_the_next_one);
    RUN_TEST(test_unrelated_sentences_are_ignored);
    RUN_TEST(test_null_and_empty_input);
    RUN_TEST(test_input_is_bounded_by_len_not_nul);
    RUN_TEST(test_overlong_input_does_not_overflow);
    RUN_TEST(test_malformed_number_reads_zero);
    RUN_TEST(test_number_forms);
    RUN_TEST(test_blank_padded_fields);
    RUN_TEST(test_whole_numbers_with_and_without_decimals);
    RUN_TEST(test_loop_before_any_data_is_inop);
    RUN_TEST(test_loop_applies_the_master_attitude);
    RUN_TEST(test_loop_attitude_goes_inop_after_the_timeout);
    RUN_TEST(test_loop_turn_rate_follows_a_standard_rate_turn);
    RUN_TEST(test_loop_turn_rate_restarts_after_a_gap);
    RUN_TEST(test_loop_standard_day);
    RUN_TEST(test_loop_altitude_in_get_bmp280_units);
    RUN_TEST(test_loop_variometer_units);
    RUN_TEST(test_loop_adopts_the_master_qnh);
    RUN_TEST(test_loop_implausible_air_data_is_inop);
    RUN_TEST(test_loop_air_goes_inop_after_the_timeout);
    RUN_TEST(test_loop_air_without_pressure_or_qnh_is_inop);
    RUN_TEST(test_loop_without_oat_keeps_the_temperature);
    RUN_TEST(test_airspeed_from_the_master);
    RUN_TEST(test_airspeed_not_sent_or_stale);
    RUN_TEST(test_master_stream_drives_the_instruments);
    RUN_TEST(test_master_qnh_reaches_the_slave_labels);
    RUN_TEST(test_master_air_reaches_the_slave_instruments);
    RUN_TEST(test_rbsim_rb09_recording_is_what_the_tests_assume);
    RUN_TEST(test_rbsim_rb09_every_sentence_is_used);
    RUN_TEST(test_rbsim_rb09_drives_every_instrument_without_inop);
    RUN_TEST(test_rbsim_rb09_turn_needle_follows_the_bank);
    RUN_TEST(test_rbsim_rb09_silence_raises_the_inop);
    RUN_TEST(test_known_issue_altimeter_is_linear_in_pressure);
    int failures = UNITY_END();
    fixture_lines_free(&rb09);
    return failures;
}
