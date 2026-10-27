/*
 * Host-test stub for LVGL 8.x — a *recording* fake.
 *
 * Objects are real structs from a fixed pool; the setters used by the firmware
 * store what they were given (position, size, angle, flags, text, image src)
 * so tests can assert on what the UI *would* show. Trigonometry and sqrt are
 * implemented faithfully because the radar geometry depends on them.
 *
 * Only what the modules under test reference is declared. Anything missing is
 * a compile error on purpose: add it here (and to ../lvgl_stubs.c) rather than
 * pulling in real LVGL.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#define LVGL_VERSION_MAJOR 8
#define LVGL_VERSION_MINOR 4

#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0
#define LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_LARGE_CONST
#define LV_IMG_PX_SIZE_ALPHA_BYTE 3
#define LV_SIZE_CONTENT ((lv_coord_t)0x2001)
#define LV_RADIUS_CIRCLE 0x7FFF
#define LV_TRIGO_SIN_MAX 32767
#define LV_TRIGO_SHIFT 15

typedef int16_t lv_coord_t;
typedef struct { uint16_t full; } lv_color_t;
typedef struct { int _unused; } lv_font_t;
typedef struct _lv_event_t { int _unused; } lv_event_t;
typedef struct _lv_disp_t { int _unused; } lv_disp_t;
typedef uint32_t lv_style_selector_t;
typedef uint32_t lv_obj_flag_t;

enum { LV_IMG_CF_TRUE_COLOR = 4, LV_IMG_CF_TRUE_COLOR_ALPHA = 5, LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED = 6 };

typedef struct {
    struct { uint32_t cf : 5; uint32_t always_zero : 3; uint32_t reserved : 2; uint32_t w : 11; uint32_t h : 11; } header;
    uint32_t data_size;
    const uint8_t *data;
} lv_img_dsc_t;

typedef enum {
    LV_ALIGN_DEFAULT = 0, LV_ALIGN_TOP_LEFT, LV_ALIGN_TOP_MID, LV_ALIGN_TOP_RIGHT,
    LV_ALIGN_BOTTOM_LEFT, LV_ALIGN_BOTTOM_MID, LV_ALIGN_BOTTOM_RIGHT,
    LV_ALIGN_LEFT_MID, LV_ALIGN_RIGHT_MID, LV_ALIGN_CENTER
} lv_align_t;
typedef enum { LV_SCROLLBAR_MODE_OFF = 0, LV_SCROLLBAR_MODE_ON, LV_SCROLLBAR_MODE_ACTIVE, LV_SCROLLBAR_MODE_AUTO } lv_scrollbar_mode_t;
typedef enum { LV_TEXT_ALIGN_AUTO = 0, LV_TEXT_ALIGN_LEFT, LV_TEXT_ALIGN_CENTER, LV_TEXT_ALIGN_RIGHT } lv_text_align_t;
typedef enum { LV_LABEL_LONG_WRAP = 0, LV_LABEL_LONG_DOT, LV_LABEL_LONG_SCROLL, LV_LABEL_LONG_SCROLL_CIRCULAR, LV_LABEL_LONG_CLIP } lv_label_long_mode_t;
enum {
    LV_OBJ_FLAG_HIDDEN     = (1u << 0),
    LV_OBJ_FLAG_CLICKABLE  = (1u << 1),
    LV_OBJ_FLAG_SCROLLABLE = (1u << 4),
};

/* Recorded object state. Tests may read these fields directly. */
typedef struct _lv_obj_t {
    struct _lv_obj_t *parent;
    lv_align_t align;
    lv_coord_t x, y;          /* last lv_obj_align offsets */
    lv_coord_t w, h;          /* last lv_obj_set_size / set_width / set_height */
    int16_t angle;            /* last lv_img_set_angle, 0.1 deg units */
    lv_obj_flag_t flags;      /* LV_OBJ_FLAG_* currently set */
    const void *src;          /* last lv_img_set_src */
    char text[512];           /* last lv_label_set_text */
    uint32_t align_calls;
    uint32_t text_calls;
    bool in_use;
} lv_obj_t;

typedef struct { uint16_t i; uint16_t f; } lv_sqrt_res_t;

/* objects */
lv_obj_t *lv_obj_create(lv_obj_t *parent);
lv_obj_t *lv_img_create(lv_obj_t *parent);
lv_obj_t *lv_label_create(lv_obj_t *parent);
void lv_obj_set_size(lv_obj_t *obj, lv_coord_t w, lv_coord_t h);
void lv_obj_set_width(lv_obj_t *obj, lv_coord_t w);
void lv_obj_set_height(lv_obj_t *obj, lv_coord_t h);
void lv_obj_align(lv_obj_t *obj, lv_align_t align, lv_coord_t x_ofs, lv_coord_t y_ofs);
void lv_obj_set_scrollbar_mode(lv_obj_t *obj, lv_scrollbar_mode_t mode);
void lv_obj_add_flag(lv_obj_t *obj, lv_obj_flag_t f);
void lv_obj_clear_flag(lv_obj_t *obj, lv_obj_flag_t f);
bool lv_obj_has_flag(const lv_obj_t *obj, lv_obj_flag_t f);
/* styles (ignored) */
void lv_obj_set_style_bg_color(lv_obj_t *obj, lv_color_t c, lv_style_selector_t sel);
void lv_obj_set_style_radius(lv_obj_t *obj, lv_coord_t r, lv_style_selector_t sel);
void lv_obj_set_style_text_font(lv_obj_t *obj, const lv_font_t *f, lv_style_selector_t sel);
void lv_obj_set_style_text_align(lv_obj_t *obj, lv_text_align_t a, lv_style_selector_t sel);
void lv_obj_set_style_text_color(lv_obj_t *obj, lv_color_t c, lv_style_selector_t sel);
/* image */
void lv_img_set_src(lv_obj_t *obj, const void *src);
void lv_img_set_angle(lv_obj_t *obj, int16_t angle);
/* label */
void lv_label_set_text(lv_obj_t *obj, const char *text);
void lv_label_set_long_mode(lv_obj_t *obj, lv_label_long_mode_t mode);
/* misc */
lv_color_t lv_color_white(void);
lv_coord_t lv_disp_get_hor_res(lv_disp_t *disp);
lv_coord_t lv_disp_get_ver_res(lv_disp_t *disp);
int16_t lv_trigo_sin(int16_t angle);
int16_t lv_trigo_cos(int16_t angle);
void lv_sqrt(uint32_t x, lv_sqrt_res_t *q, uint32_t mask);

extern const lv_font_t lv_font_montserrat_14;
extern const lv_font_t lv_font_montserrat_16;
extern const lv_font_t lv_font_montserrat_24;
extern const lv_font_t lv_font_montserrat_32;
extern const lv_font_t lv_font_montserrat_48;
