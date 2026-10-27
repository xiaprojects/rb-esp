/*
 * Recording LVGL fake — see include/lvgl.h and host_stubs.h.
 */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "host_stubs.h"
#include "lvgl.h"

#define LV_STUB_POOL 256

static lv_obj_t s_pool[LV_STUB_POOL];
static size_t s_pool_used;
static char s_last_label[512];
static lv_coord_t s_hor_res = 480, s_ver_res = 480;

const lv_font_t lv_font_montserrat_14, lv_font_montserrat_16, lv_font_montserrat_24,
                lv_font_montserrat_32, lv_font_montserrat_48;

static lv_obj_t *alloc_obj(lv_obj_t *parent)
{
    if (s_pool_used >= LV_STUB_POOL) {
        fprintf(stderr, "lvgl stub: object pool exhausted (%d)\n", LV_STUB_POOL);
        return NULL;
    }
    lv_obj_t *o = &s_pool[s_pool_used++];
    memset(o, 0, sizeof(*o));
    o->parent = parent;
    o->in_use = true;
    return o;
}

lv_obj_t *lv_obj_create(lv_obj_t *parent)   { return alloc_obj(parent); }
lv_obj_t *lv_img_create(lv_obj_t *parent)   { return alloc_obj(parent); }
lv_obj_t *lv_label_create(lv_obj_t *parent) { return alloc_obj(parent); }

void lv_obj_set_size(lv_obj_t *o, lv_coord_t w, lv_coord_t h) { if (o) { o->w = w; o->h = h; } }
void lv_obj_set_width(lv_obj_t *o, lv_coord_t w)  { if (o) o->w = w; }
void lv_obj_set_height(lv_obj_t *o, lv_coord_t h) { if (o) o->h = h; }

void lv_obj_align(lv_obj_t *o, lv_align_t align, lv_coord_t x, lv_coord_t y)
{
    if (!o) return;
    o->align = align; o->x = x; o->y = y; o->align_calls++;
}

void lv_obj_set_scrollbar_mode(lv_obj_t *o, lv_scrollbar_mode_t m) { (void)o; (void)m; }
void lv_obj_add_flag(lv_obj_t *o, lv_obj_flag_t f)   { if (o) o->flags |= f; }
void lv_obj_clear_flag(lv_obj_t *o, lv_obj_flag_t f) { if (o) o->flags &= ~f; }
bool lv_obj_has_flag(const lv_obj_t *o, lv_obj_flag_t f) { return o && (o->flags & f) == f; }

void lv_obj_set_style_bg_color(lv_obj_t *o, lv_color_t c, lv_style_selector_t s) { (void)o; (void)c; (void)s; }
void lv_obj_set_style_radius(lv_obj_t *o, lv_coord_t r, lv_style_selector_t s) { (void)o; (void)r; (void)s; }
void lv_obj_set_style_text_font(lv_obj_t *o, const lv_font_t *f, lv_style_selector_t s) { (void)o; (void)f; (void)s; }
void lv_obj_set_style_text_align(lv_obj_t *o, lv_text_align_t a, lv_style_selector_t s) { (void)o; (void)a; (void)s; }
void lv_obj_set_style_text_color(lv_obj_t *o, lv_color_t c, lv_style_selector_t s) { (void)o; (void)c; (void)s; }

void lv_img_set_src(lv_obj_t *o, const void *src) { if (o) o->src = src; }
void lv_img_set_angle(lv_obj_t *o, int16_t angle) { if (o) o->angle = angle; }

void lv_label_set_text(lv_obj_t *o, const char *text)
{
    strncpy(s_last_label, text ? text : "", sizeof(s_last_label) - 1);
    s_last_label[sizeof(s_last_label) - 1] = '\0';
    if (!o) return;
    strncpy(o->text, text ? text : "", sizeof(o->text) - 1);
    o->text[sizeof(o->text) - 1] = '\0';
    o->text_calls++;
}
void lv_label_set_long_mode(lv_obj_t *o, lv_label_long_mode_t m) { (void)o; (void)m; }

lv_color_t lv_color_white(void) { lv_color_t c = { 0xFFFF }; return c; }
lv_coord_t lv_disp_get_hor_res(lv_disp_t *d) { (void)d; return s_hor_res; }
lv_coord_t lv_disp_get_ver_res(lv_disp_t *d) { (void)d; return s_ver_res; }

/* LVGL returns sin(angle) scaled to [-32767, 32767]. */
int16_t lv_trigo_sin(int16_t angle)
{
    return (int16_t)lrint(sin((double)angle * M_PI / 180.0) * LV_TRIGO_SIN_MAX);
}
int16_t lv_trigo_cos(int16_t angle) { return lv_trigo_sin((int16_t)(angle + 90)); }

void lv_sqrt(uint32_t x, lv_sqrt_res_t *q, uint32_t mask)
{
    (void)mask;
    double r = sqrt((double)x);
    q->i = (uint16_t)r;
    q->f = (uint16_t)((r - q->i) * 256.0);
}

/* ---- test control ------------------------------------------------------- */
const char *host_stub_last_label_text(void) { return s_last_label; }
void host_stub_lvgl_reset(void)
{
    memset(s_pool, 0, sizeof(s_pool));
    s_pool_used = 0;
    s_last_label[0] = '\0';
    s_hor_res = s_ver_res = 480;
}
void host_stub_lvgl_set_resolution(lv_coord_t w, lv_coord_t h) { s_hor_res = w; s_ver_res = h; }
size_t host_stub_lvgl_object_count(void) { return s_pool_used; }
