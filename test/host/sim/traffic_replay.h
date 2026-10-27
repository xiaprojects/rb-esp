/*
 * Replay a FLARM/NMEA fixture through the firmware's traffic parser the way a
 * real transport delivers it.
 *
 * The point of the transport modes is that the firmware's entry points frame
 * data differently, and that difference has been a real source of bugs:
 *
 *   REPLAY_LINE   - RB_Usb.c buffers console bytes until '\n' and hands over
 *                   one complete sentence to RB05_FlarmParser_ParseData().
 *   REPLAY_BULK   - one stream read carrying the whole buffer.
 *   REPLAY_BLE    - MTU-sized fragments through RB05_FlarmParser_FeedStream(),
 *                   which is what RB05_ESP_Bluetooth.c does with each
 *                   ESP_GATTC_NOTIFY_EVT payload.
 *   REPLAY_BLE_NO_REASSEMBLY
 *                 - the same fragments handed straight to ParseData, i.e. the
 *                   old BLE path with no reassembly. Kept so the regression
 *                   stays pinned.
 *
 * The fixture's $GPRMC / $GPGGA timestamps drive `datetime`, which is what the
 * parser uses to stamp and age traffic, so replay advances the device clock
 * exactly as a real session would.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef enum {
    REPLAY_LINE = 0,           /* one whole sentence per parser call */
    REPLAY_BULK,               /* the whole buffer in one parser call */
    REPLAY_BLE,                /* fragments via the reassembler */
    REPLAY_BLE_NO_REASSEMBLY,  /* fragments straight to the parser */
} replay_mode_t;

/* Default fragment size for the BLE modes: the default BLE MTU is 23, which
 * leaves 20 bytes of payload. */
#define REPLAY_BLE_DEFAULT_CHUNK 20

typedef struct {
    char  *data;
    size_t len;
} replay_fixture_t;

/* Load a fixture file. Lines starting with '#' are stripped as comments.
 * Aborts the test with a clear message if the file cannot be read. */
replay_fixture_t replay_load(const char *path);
void replay_free(replay_fixture_t *f);

/* Replay the whole fixture. `chunk` is the fragment size for the BLE modes
 * (0 selects REPLAY_BLE_DEFAULT_CHUNK); ignored by the other modes.
 * Returns the number of parser/reassembler calls made. */
unsigned replay_all(const replay_fixture_t *f, replay_mode_t mode, size_t chunk);

/* Replay only up to and including the Nth cycle, where a cycle starts at each
 * $GPRMC. Lets a test stop at a known instant. Returns calls made. */
unsigned replay_cycles(const replay_fixture_t *f, replay_mode_t mode,
                       size_t chunk, unsigned cycles);

/* Set `datetime` directly (hour, minute, second), e.g. to age traffic out
 * without feeding more sentences. Also clears any partial reassembly. */
void replay_set_clock(uint8_t hour, uint8_t minute, uint8_t second);

/* Seconds-of-day for the current `datetime`, the same expression the firmware
 * uses to stamp traffic. */
uint32_t replay_clock_seconds(void);

/* The traffic parser printf()s a line per sentence. Replay swallows that by
 * default so test output stays readable; set RB_TEST_VERBOSE=1 in the
 * environment, or call this with false, to let it through. */
void replay_set_quiet(bool quiet);
