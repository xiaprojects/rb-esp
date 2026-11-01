/*
 * A recorded NMEA fixture as the lines RB_Usb.c hands to the parsers: one
 * sentence per entry, CR/LF stripped, NUL-terminated at its length. Lines
 * starting with '#' are fixture comments and are dropped.
 *
 * The field splitter is the tests' own, independent of the firmware's
 * scanners, so expectations computed with it do not come from the code
 * under test.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    char  **line;
    size_t *len;
    int     count;
    char   *text; /* backing store for line[] */
} fixture_lines_t;

/* Aborts with a clear message if the file cannot be read. */
fixture_lines_t fixture_lines_load(const char *path);
void fixture_lines_free(fixture_lines_t *f);

/* Is `line` the sentence `name` ("RBATT", "GPRMC", ...)? */
bool fixture_is(const char *line, const char *name);

/* Lines of the fixture that are the sentence `name`. */
int fixture_count(const fixture_lines_t *f, const char *name);

/* Splits a sentence into fields[0] = its name without '$', then each field up
 * to the checksum. Longer fields are truncated to FIXTURE_FIELD_MAX - 1
 * characters. Returns the number of fields, name included. */
#define FIXTURE_FIELD_MAX 24
int fixture_fields(const char *line, char fields[][FIXTURE_FIELD_MAX], int max);
