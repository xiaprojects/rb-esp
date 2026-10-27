#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "traffic_replay.h"
#include "RB02_Config.h"
#include "RB05_FlarmParser.h"

/* The firmware's RTC/GPS clock. */
extern datetime_t datetime;

replay_fixture_t replay_load(const char *path)
{
    replay_fixture_t f = { NULL, 0 };
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        fprintf(stderr, "replay_load: cannot open '%s'\n", path);
        abort();
    }
    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    char *raw = malloc((size_t)size + 1);
    size_t got = fread(raw, 1, (size_t)size, fp);
    raw[got] = '\0';
    fclose(fp);

    /* Strip comment lines. CRLF is preserved inside sentences: the parser has
     * to cope with it. */
    char *out = malloc(got + 1);
    size_t o = 0;
    const char *p = raw;
    while (*p) {
        const char *eol = strchr(p, '\n');
        size_t line = eol ? (size_t)(eol - p + 1) : strlen(p);
        if (*p != '#') {
            memcpy(out + o, p, line);
            o += line;
        }
        if (!eol) break;
        p += line;
    }
    out[o] = '\0';
    free(raw);

    f.data = out;
    f.len = o;
    return f;
}

void replay_free(replay_fixture_t *f)
{
    free(f->data);
    f->data = NULL;
    f->len = 0;
}

void replay_set_clock(uint8_t hour, uint8_t minute, uint8_t second)
{
    datetime.hour = hour;
    datetime.minute = minute;
    datetime.second = second;
    /* A test that jumps the clock is starting a new scenario; a sentence left
     * half-reassembled from the previous one would leak into it. */
    RB05_FlarmParser_ResetStream();
}

uint32_t replay_clock_seconds(void)
{
    return (uint32_t)datetime.second
         + (uint32_t)datetime.minute * 60u
         + (uint32_t)datetime.hour * 3600u;
}

/* "$GPRMC,191830.20,..." / "$GPGGA,191830.20,..." -> advance the clock. */
static void maybe_update_clock(const char *line, size_t len)
{
    if (len < 13 || line[0] != '$') return;
    const char *tag = line + 1;
    bool is_time = (strncmp(tag, "GPRMC,", 6) == 0 || strncmp(tag, "GNRMC,", 6) == 0
                 || strncmp(tag, "GPGGA,", 6) == 0 || strncmp(tag, "GNGGA,", 6) == 0);
    if (!is_time) return;

    const char *t = tag + 6;
    for (int i = 0; i < 6; i++)
        if (t[i] < '0' || t[i] > '9') return;

    datetime.hour   = (uint8_t)((t[0] - '0') * 10 + (t[1] - '0'));
    datetime.minute = (uint8_t)((t[2] - '0') * 10 + (t[3] - '0'));
    datetime.second = (uint8_t)((t[4] - '0') * 10 + (t[5] - '0'));
}

/* ---- device-log suppression --------------------------------------------- */
static int s_quiet = -1;      /* -1 = not yet resolved from the environment */
static int s_saved_stdout = -1;

void replay_set_quiet(bool quiet) { s_quiet = quiet ? 1 : 0; }

static void quiet_begin(void)
{
    if (s_quiet < 0) {
        const char *v = getenv("RB_TEST_VERBOSE");
        s_quiet = (v && *v && strcmp(v, "0") != 0) ? 0 : 1;
    }
    if (!s_quiet || s_saved_stdout >= 0) return;

    fflush(stdout);
    int devnull = open("/dev/null", O_WRONLY);
    if (devnull < 0) return;
    s_saved_stdout = dup(STDOUT_FILENO);
    dup2(devnull, STDOUT_FILENO);
    close(devnull);
}

static void quiet_end(void)
{
    if (s_saved_stdout < 0) return;
    fflush(stdout);
    dup2(s_saved_stdout, STDOUT_FILENO);
    close(s_saved_stdout);
    s_saved_stdout = -1;
}

/* ---- delivery ----------------------------------------------------------- */

/* Byte offset at which to stop: the start of cycle `cycles` (0 = whole file). */
static size_t stop_offset(const replay_fixture_t *f, unsigned cycles)
{
    if (cycles == 0) return f->len;
    unsigned seen = 0;
    for (size_t i = 0; i + 7 <= f->len; i++) {
        if (f->data[i] == '$' && strncmp(f->data + i + 1, "GPRMC,", 6) == 0) {
            if (++seen > cycles) return i;
        }
    }
    return f->len;
}

/* Deliver [0, stop) in `chunk`-sized fragments, advancing the clock from the
 * lines each fragment completes. Fragments deliberately ignore line
 * boundaries, which is the whole point of the BLE modes. */
static unsigned deliver_fragments(const replay_fixture_t *f, size_t stop,
                                  size_t chunk, bool reassemble)
{
    unsigned calls = 0;
    size_t sent = 0, scanned = 0;

    while (sent < stop) {
        size_t take = (stop - sent) < chunk ? (stop - sent) : chunk;

        while (scanned < sent + take && scanned < stop) {
            const char *nl = memchr(f->data + scanned, '\n', stop - scanned);
            size_t line = nl ? (size_t)(nl - (f->data + scanned) + 1) : (stop - scanned);
            maybe_update_clock(f->data + scanned, line);
            scanned += line;
            if (!nl) break;
        }

        const uint8_t *p = (const uint8_t *)(f->data + sent);
        if (reassemble) {
            RB05_FlarmParser_FeedStream(p, (uint16_t)take);
        } else {
            RB05_FlarmParser_ParseData(p, (uint16_t)take);
        }
        calls++;
        sent += take;
    }
    return calls;
}

static unsigned replay_cycles_inner(const replay_fixture_t *f, replay_mode_t mode,
                                    size_t chunk, unsigned cycles)
{
    if (chunk == 0) chunk = REPLAY_BLE_DEFAULT_CHUNK;
    const size_t stop = stop_offset(f, cycles);

    if (mode == REPLAY_BLE || mode == REPLAY_BLE_NO_REASSEMBLY) {
        return deliver_fragments(f, stop, chunk, mode == REPLAY_BLE);
    }

    if (mode == REPLAY_BULK) {
        /* One read carrying everything. Walk the lines first so the clock ends
         * up where a real device's RTC would be when the read is parsed. */
        size_t scanned = 0;
        while (scanned < stop) {
            const char *nl = memchr(f->data + scanned, '\n', stop - scanned);
            size_t line = nl ? (size_t)(nl - (f->data + scanned) + 1) : (stop - scanned);
            maybe_update_clock(f->data + scanned, line);
            scanned += line;
            if (!nl) break;
        }
        RB05_FlarmParser_ParseData((const uint8_t *)f->data, (uint16_t)stop);
        return 1;
    }

    /* REPLAY_LINE */
    unsigned calls = 0;
    size_t i = 0;
    while (i < stop) {
        const char *nl = memchr(f->data + i, '\n', stop - i);
        size_t line = nl ? (size_t)(nl - (f->data + i) + 1) : (stop - i);
        maybe_update_clock(f->data + i, line);
        RB05_FlarmParser_ParseData((const uint8_t *)(f->data + i), (uint16_t)line);
        calls++;
        i += line;
    }
    return calls;
}

unsigned replay_cycles(const replay_fixture_t *f, replay_mode_t mode,
                       size_t chunk, unsigned cycles)
{
    quiet_begin();
    unsigned calls = replay_cycles_inner(f, mode, chunk, cycles);
    quiet_end();
    return calls;
}

unsigned replay_all(const replay_fixture_t *f, replay_mode_t mode, size_t chunk)
{
    return replay_cycles(f, mode, chunk, 0);
}
