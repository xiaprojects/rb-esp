#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fixture_lines.h"

fixture_lines_t fixture_lines_load(const char *path)
{
    fixture_lines_t f = { NULL, NULL, 0, NULL };
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        fprintf(stderr, "fixture_lines_load: cannot open '%s'\n", path);
        abort();
    }
    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    f.text = malloc((size_t)size + 1);
    size_t got = fread(f.text, 1, (size_t)size, fp);
    f.text[got] = '\0';
    fclose(fp);

    /* At most one line per '\n', plus a last one without it. */
    int max = 1;
    for (size_t i = 0; i < got; i++)
        max += f.text[i] == '\n';
    f.line = malloc(sizeof(*f.line) * (size_t)max);
    f.len = malloc(sizeof(*f.len) * (size_t)max);

    for (char *p = f.text; *p;) {
        size_t n = strcspn(p, "\r\n");
        char *next = p + n;
        next += strspn(next, "\r\n");
        p[n] = '\0';
        if (n > 0 && p[0] != '#') {
            f.line[f.count] = p;
            f.len[f.count] = n;
            f.count++;
        }
        p = next;
    }
    return f;
}

void fixture_lines_free(fixture_lines_t *f)
{
    free(f->line);
    free(f->len);
    free(f->text);
    memset(f, 0, sizeof(*f));
}

bool fixture_is(const char *line, const char *name)
{
    size_t n = strlen(name);
    return line[0] == '$' && strncmp(line + 1, name, n) == 0 && line[1 + n] == ',';
}

int fixture_count(const fixture_lines_t *f, const char *name)
{
    int n = 0;
    for (int i = 0; i < f->count; i++)
        n += fixture_is(f->line[i], name);
    return n;
}

int fixture_fields(const char *line, char fields[][FIXTURE_FIELD_MAX], int max)
{
    int n = 0, k = 0;
    for (const char *p = line + 1; *p && *p != '*' && *p != '\r' && *p != '\n'; p++) {
        if (*p == ',') {
            fields[n][k] = '\0';
            if (++n == max)
                return n;
            k = 0;
        } else if (k < FIXTURE_FIELD_MAX - 1) {
            fields[n][k++] = *p;
        }
    }
    fields[n][k] = '\0';
    return n + 1;
}
