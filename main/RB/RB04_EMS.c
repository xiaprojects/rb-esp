/**
 * This file is part of RB.
 *
 * Copyright (C) 2025 XIAPROJECTS SRL
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.

 * This source is part of the project RB:
 * 01 -> Display with Synthetic vision, Autopilot and ADSB
 * 02 -> Display with SixPack
 * 03 -> Display with Autopilot, ADSB, Radio, Flight Computer
 * 04 -> Display with EMS: Engine monitoring system
 * 05 -> Display with Stratux BLE Traffic
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/
#include "RB04_EMS.h"
#ifdef RB_ENABLE_EMS
#include "RB02_GUIHelpers.h"
#include <stdio.h>
#include "RB04_NMEA_Parser.c"

#define RB_EMS_VERTICAL_BAR_WIDTH 38
#define RB_EMS_VERTICAL_BAR_HEIGHT 260
#define RB_EMS_VERTICAL_BAR_SPACE 2

/* Scale ranges and thresholds (Fahrenheit) */
#define RB_EMS_CHT_MIN_F 100
#define RB_EMS_CHT_MAX_F 400
#define RB_EMS_CHT_THRESH_GREEN 180
#define RB_EMS_CHT_THRESH_YELLOW 330
#define RB_EMS_CHT_THRESH_RED 361

#define RB_EMS_EGT_MIN_F 750
#define RB_EMS_EGT_MAX_F 1450
#define RB_EMS_EGT_THRESH_GREEN 1200
#define RB_EMS_EGT_THRESH_YELLOW 1350
#define RB_EMS_EGT_THRESH_RED 1451

/* Scale tick labels are computed dynamically from min/max values */

typedef struct
{
    lv_obj_t *label;
    lv_obj_t *labelContainer;
    lv_obj_t *display;
    lv_obj_t *barFill;
    int16_t thresholdGreen;
    int16_t thresholdYellow;
    int16_t thresholdRed;
    lv_obj_t *triangleDirection;
    lv_obj_t *maxLine;
    int16_t maxValue;
    int16_t value;
    float valueAverage;

} RB04_EMSSWidget;

typedef struct
{
    RB04_EMSSWidget topLeft;
    RB04_EMSSWidget topRight;
    RB04_EMSSWidget bottomLeft;
    RB04_EMSSWidget bottomRight;
    RB04_EMSSWidget verticalBars[RB_EMS_VERTICAL_BARS_NUMBER * 2];

} RB04_EMSStatus;

lv_obj_t *RB04_EMS_DrawLine(lv_obj_t *parent, lv_coord_t width, lv_coord_t height, lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *line = lv_obj_create(parent);
    lv_obj_set_scrollbar_mode(line, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(line, width, height);
    lv_obj_set_style_bg_color(line, lv_color_make(45, 45, 45), 0);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(line, 0, LV_PART_MAIN);
    lv_obj_align(line, LV_ALIGN_CENTER, x, y);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
    return line;
}
/*
lv_obj_t *RB04_EMS_CreateHorizontalThresholds(lv_obj_t *parent, RB04_EMSStatus *emsStatus)
{
    const int temps[] = {100, 200, 300, 350, 400};
    const lv_coord_t bar_h = RB_EMS_VERTICAL_BAR_HEIGHT;
    for (size_t i = 0; i < (sizeof(temps) / sizeof(temps[0])); i++)
    {
        int t = temps[i];
        float normalized = (t - 100) / 300.0f;
        if (normalized < 0) normalized = 0;
        if (normalized > 1) normalized = 1;
        lv_coord_t heightFromBottom = (lv_coord_t)(normalized * (float)bar_h);
        lv_coord_t y = (bar_h / 2) - heightFromBottom;
        RB04_EMS_DrawLine(parent, 440, 2, 0, y);
    }

    return NULL;
}
*/

lv_obj_t *RB04_EMS_CreateVerticalBar(lv_obj_t *parent, RB04_EMSSWidget *widget, uint8_t index)
{
    lv_coord_t x = 0;
    lv_coord_t y = 0;
    lv_coord_t w = RB_EMS_VERTICAL_BAR_WIDTH;
    lv_coord_t h = RB_EMS_VERTICAL_BAR_HEIGHT;
    lv_coord_t xpos = (lv_coord_t)(x + (index * (w + RB_EMS_VERTICAL_BAR_SPACE)) - (RB_EMS_VERTICAL_BARS_NUMBER * 2 / 2.0 * (w + RB_EMS_VERTICAL_BAR_SPACE)) + (w + RB_EMS_VERTICAL_BAR_SPACE) / 2.0);
    widget->display = RB04_EMS_DrawLine(parent, w, h, xpos, y);

    if (widget->display)
    {
        /* colored fill inside the white bar */
        lv_obj_t *barFill = lv_obj_create(widget->display);
        lv_obj_set_size(barFill, w, 0);
        lv_obj_set_style_bg_color(barFill, lv_color_make(0, 200, 0), 0);
        lv_obj_set_style_border_width(barFill, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(barFill, 0, LV_PART_MAIN);
        lv_obj_align(barFill, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_clear_flag(barFill, LV_OBJ_FLAG_CLICKABLE);
        widget->barFill = barFill;

        /* value label below each bar */
        widget->label = lv_label_create(parent);
        lv_label_set_text(widget->label, "--");
        lv_obj_set_style_text_color(widget->label, lv_color_make(255, 255, 255), 0);
        lv_obj_align(widget->label, LV_ALIGN_CENTER, xpos, -y - h / 2 - 14);

        /* Determine logical sensor for this position and label appropriately.
         * We display pairs: CHT1, EGT1, CHT2, EGT2, ... so even indexes are CHTs.
         */
        int sensorIndex = index / 2; /* 0-based */
        int isCHT = (index % 2) == 0;

        lv_obj_t *title = lv_label_create(parent);
        char titleString[10];
        if (isCHT)
        {
            snprintf(titleString, sizeof(titleString), "CHT%d", sensorIndex + 1);
            widget->thresholdGreen = RB_EMS_CHT_THRESH_GREEN;
            widget->thresholdYellow = RB_EMS_CHT_THRESH_YELLOW;
            widget->thresholdRed = RB_EMS_CHT_THRESH_RED;
        }
        else
        {
            snprintf(titleString, sizeof(titleString), "EGT%d", sensorIndex + 1);
            widget->thresholdGreen = RB_EMS_EGT_THRESH_GREEN;
            widget->thresholdYellow = RB_EMS_EGT_THRESH_YELLOW;
            widget->thresholdRed = RB_EMS_EGT_THRESH_RED;
        }
        lv_label_set_text(title, titleString);
        lv_obj_set_style_text_color(title, lv_color_make(255, 255, 255), 0);
        lv_obj_align(title, LV_ALIGN_CENTER, xpos, y + h / 2 + 14);

        widget->value = -9999; /* sentinel: force first update */

        /* triangle indicator above current value */
        widget->triangleDirection = lv_label_create(parent);

        lv_obj_set_style_text_color(widget->triangleDirection, lv_color_make(255, 255, 255), 0);
        lv_obj_set_style_text_font(widget->triangleDirection, &lv_font_montserrat_32, 0);
        lv_obj_clear_flag(widget->triangleDirection, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_align(widget->triangleDirection, LV_ALIGN_CENTER, xpos, -(h / 2));

        /* per-bar max indicator (hidden initially) */
        widget->maxLine = lv_obj_create(parent);
        lv_obj_set_size(widget->maxLine, w, 3);
        lv_obj_set_style_bg_color(widget->maxLine, lv_color_make(255, 255, 255), 0);
        lv_obj_set_style_border_width(widget->maxLine, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(widget->maxLine, 0, LV_PART_MAIN);
        lv_obj_clear_flag(widget->maxLine, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(widget->maxLine, LV_OBJ_FLAG_HIDDEN);
        lv_obj_align(widget->maxLine, LV_ALIGN_CENTER, xpos, -(h / 2));
        widget->maxValue = -9999;
    }

    return widget->display;
}

lv_obj_t *RB04_EMS_CreateScreen(lv_obj_t *parent, RB02_Status *status)
{
    // RB02_GUIHelpers_CreateBase(parent, &EMSBackground); // DEMO
    // return NULL;

    if (status->ui.ems == NULL)
    {
        status->ui.ems = lv_mem_alloc(sizeof(RB04_EMSStatus));
        if (status->ui.ems != NULL)
        {
            lv_memset_00(status->ui.ems, sizeof(RB04_EMSStatus));
        }
    }

    RB04_EMSStatus *emsStatus = (RB04_EMSStatus *)status->ui.ems;

    /* Use the embedded EMS background image as the base to be pixel-perfect */

    /* Header text on top of background */
    lv_obj_t *title = lv_label_create(parent);
    lv_label_set_text(title, "RB-04");
    lv_obj_set_style_text_color(title, lv_color_make(255, 255, 255), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, -6);

    lv_obj_t *unit = lv_label_create(parent);
    if (status->ui.displayFarenheit)
    {
        lv_label_set_text(unit, "FAHRENHEIT");
    }
    else
    {
        lv_label_set_text(unit, "CELSIUS");
    }
    lv_obj_set_style_text_color(unit, lv_color_make(255, 255, 255), 0);
    lv_obj_align(unit, LV_ALIGN_TOP_MID, 0, 6);

    emsStatus->topLeft.display = lv_label_create(parent);
    lv_label_set_text(emsStatus->topLeft.display, "0°");
    lv_obj_set_style_text_font(emsStatus->topLeft.display, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(emsStatus->topLeft.display, lv_color_make(255, 255, 255), 0);
    lv_obj_align(emsStatus->topLeft.display, LV_ALIGN_TOP_MID, -65, 14);

    emsStatus->topLeft.label = lv_label_create(parent);
    lv_label_set_text(emsStatus->topLeft.label, "OIL");
    lv_obj_set_style_text_color(emsStatus->topLeft.label, lv_color_make(255, 255, 255), 0);
    lv_obj_align(emsStatus->topLeft.label, LV_ALIGN_TOP_MID, -65, 2);

    emsStatus->topRight.display = lv_label_create(parent);
    lv_label_set_text(emsStatus->topRight.display, "0");
    lv_obj_set_style_text_font(emsStatus->topRight.display, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(emsStatus->topRight.display, lv_color_make(255, 255, 255), 0);
    lv_obj_align(emsStatus->topRight.display, LV_ALIGN_TOP_MID, 65, 14);

    emsStatus->topRight.label = lv_label_create(parent);
    lv_label_set_text(emsStatus->topRight.label, "MAP");
    lv_obj_set_style_text_color(emsStatus->topRight.label, lv_color_make(255, 255, 255), 0);
    lv_obj_align(emsStatus->topRight.label, LV_ALIGN_TOP_MID, 65, 2);

    emsStatus->bottomLeft.display = lv_label_create(parent);
    lv_label_set_text(emsStatus->bottomLeft.display, "0°");
    lv_obj_set_style_text_font(emsStatus->bottomLeft.display, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(emsStatus->bottomLeft.display, lv_color_make(255, 255, 255), 0);
    lv_obj_align(emsStatus->bottomLeft.display, LV_ALIGN_BOTTOM_MID, -65, -2);

    emsStatus->bottomLeft.label = lv_label_create(parent);
    lv_label_set_text(emsStatus->bottomLeft.label, "OUT TEMP.");
    lv_obj_set_style_text_color(emsStatus->bottomLeft.label, lv_color_make(255, 255, 255), 0);
    lv_obj_align(emsStatus->bottomLeft.label, LV_ALIGN_BOTTOM_MID, -65, -48);

    emsStatus->bottomRight.display = lv_label_create(parent);
    lv_label_set_text(emsStatus->bottomRight.display, "0");
    lv_obj_set_style_text_font(emsStatus->bottomRight.display, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(emsStatus->bottomRight.display, lv_color_make(255, 255, 255), 0);
    lv_obj_align(emsStatus->bottomRight.display, LV_ALIGN_BOTTOM_MID, 65, -2);

    emsStatus->bottomRight.label = lv_label_create(parent);
    lv_label_set_text(emsStatus->bottomRight.label, "RPM");
    lv_obj_set_style_text_color(emsStatus->bottomRight.label, lv_color_make(255, 255, 255), 0);
    lv_obj_align(emsStatus->bottomRight.label, LV_ALIGN_BOTTOM_MID, 65, -48);

    /* Create the 8 vertical bars as children of the background so they align
     * directly over the artwork. */
    for (uint8_t x = 0; x < RB_EMS_VERTICAL_BARS_NUMBER * 2; x++)
    {
        RB04_EMS_CreateVerticalBar(parent, &(emsStatus->verticalBars[x]), x);
    }

    /* Create left (CHT) and right (EGT) scale labels using dynamic steps.
     * Step is calculated from MAX-MIN and rounded to nearest 10.
     */
    const lv_coord_t bar_h = RB_EMS_VERTICAL_BAR_HEIGHT;
    const int numTicks = RB_EMS_VERTICAL_BARS_NUMBER;

    int leftRange = RB_EMS_CHT_MAX_F - RB_EMS_CHT_MIN_F;
    int rightRange = RB_EMS_EGT_MAX_F - RB_EMS_EGT_MIN_F;

    /* raw step (may be fractional) across (numTicks-1) intervals */
    float rawStepLeft = (numTicks > 1) ? (leftRange / (float)(numTicks - 1)) : (float)leftRange;
    float rawStepRight = (numTicks > 1) ? (rightRange / (float)(numTicks - 1)) : (float)rightRange;

    /* round to nearest 10 (avoid math.h): */
    int stepLeft = (int)((rawStepLeft / 10.0f) + 0.5f) * 10;
    int stepRight = (int)((rawStepRight / 10.0f) + 0.5f) * 10;
    if (stepLeft <= 0)
        stepLeft = 10;
    if (stepRight <= 0)
        stepRight = 10;

    for (int i = 0; i < numTicks; i++)
    {
        /* left tick value from MAX downward */
        int tickLeft = RB_EMS_CHT_MAX_F - i * stepLeft;
        if (tickLeft < RB_EMS_CHT_MIN_F)
            tickLeft = RB_EMS_CHT_MIN_F;
        float n = (leftRange > 0) ? (tickLeft - RB_EMS_CHT_MIN_F) / (float)leftRange : 0.0f;
        if (n < 0)
            n = 0;
        if (n > 1)
            n = 1;
        lv_coord_t heightFromBottom = (lv_coord_t)(n * (float)bar_h);
        lv_coord_t y = (bar_h / 2) - heightFromBottom;
        char buf[20];
        if (!status->ui.displayFarenheit)
        {
            snprintf(buf, sizeof(buf), "%d°", ((int)((tickLeft - 32.5) * 5.0f / 9.0f)) / 10 * 10);
        }
        else
        {
            snprintf(buf, sizeof(buf), "%d°", tickLeft);
        }
        lv_obj_t *l = lv_label_create(parent);
        lv_label_set_text(l, buf);
        lv_obj_set_style_text_color(l, lv_color_make(255, 255, 255), 0);
        lv_obj_align(l, LV_ALIGN_CENTER, -180, y);

        /* right tick value from MAX downward */
        int tickRight = RB_EMS_EGT_MAX_F - i * stepRight;
        if (tickRight < RB_EMS_EGT_MIN_F)
            tickRight = RB_EMS_EGT_MIN_F;
        float nr = (rightRange > 0) ? (tickRight - RB_EMS_EGT_MIN_F) / (float)rightRange : 0.0f;
        if (nr < 0)
            nr = 0;
        if (nr > 1)
            nr = 1;
        lv_coord_t heightFromBottomR = (lv_coord_t)(nr * (float)bar_h);
        lv_coord_t yr = (bar_h / 2) - heightFromBottomR;

        if (!status->ui.displayFarenheit)
        {
            snprintf(buf, sizeof(buf), "%d°", ((int)((tickRight - 32.5) * 5.0f / 9.0f)) / 10 * 10);
        }
        else
        {
            snprintf(buf, sizeof(buf), "%d°", tickRight);
        }
        lv_obj_t *r = lv_label_create(parent);
        lv_label_set_text(r, buf);
        lv_obj_set_style_text_color(r, lv_color_make(255, 255, 255), 0);
        lv_obj_align(r, LV_ALIGN_CENTER, 180, yr);
    }

    return parent;
}

void RB04_EMS_Tick(RB02_Status *status, RB04_EMSData *newEmsData)
{
    if (status == NULL || newEmsData == NULL)
        return;

    RB04_EMSStatus *emsStatus = (RB04_EMSStatus *)status->ui.ems;
    if (emsStatus == NULL)
        return;
    char buf[24];
    if (status->ui.displayFarenheit)
    {
        if (emsStatus->topLeft.display && emsStatus->topLeft.value != newEmsData->oilTemperature)
        {
            snprintf(buf, sizeof(buf), "%d°", (int)(newEmsData->oilTemperature * 9.0f / 5.0f + 32.0f + 0.5f));
            lv_label_set_text(emsStatus->topLeft.display, buf);
            emsStatus->topLeft.value = newEmsData->oilTemperature;
        }
        if (emsStatus->topRight.display && emsStatus->topRight.value != newEmsData->manifoldPressure)
        {
            snprintf(buf, sizeof(buf), "%d", newEmsData->manifoldPressure);
            lv_label_set_text(emsStatus->topRight.display, buf);
            emsStatus->topRight.value = newEmsData->manifoldPressure;
        }
        if (emsStatus->bottomLeft.display && emsStatus->bottomLeft.value != newEmsData->outTemperature)
        {
            snprintf(buf, sizeof(buf), "%d°", (int)(newEmsData->outTemperature * 9.0f / 5.0f + 32.0f + 0.5f));
            lv_label_set_text(emsStatus->bottomLeft.display, buf);
            emsStatus->bottomLeft.value = newEmsData->outTemperature;
        }
        if (emsStatus->bottomRight.display && emsStatus->bottomRight.value != newEmsData->rpm)
        {
            snprintf(buf, sizeof(buf), "%d", newEmsData->rpm);
            lv_label_set_text(emsStatus->bottomRight.display, buf);
            emsStatus->bottomRight.value = newEmsData->rpm;
        }
    }
    else
    {
        if (emsStatus->topLeft.display && emsStatus->topLeft.value != newEmsData->oilTemperature)
        {
            snprintf(buf, sizeof(buf), "%d°", newEmsData->oilTemperature);
            lv_label_set_text(emsStatus->topLeft.display, buf);
            emsStatus->topLeft.value = newEmsData->oilTemperature;
        }
        if (emsStatus->topRight.display && emsStatus->topRight.value != newEmsData->manifoldPressure)
        {
            snprintf(buf, sizeof(buf), "%d", newEmsData->manifoldPressure);
            lv_label_set_text(emsStatus->topRight.display, buf);
            emsStatus->topRight.value = newEmsData->manifoldPressure;
        }
        if (emsStatus->bottomLeft.display && emsStatus->bottomLeft.value != newEmsData->outTemperature)
        {
            snprintf(buf, sizeof(buf), "%d°", newEmsData->outTemperature);
            lv_label_set_text(emsStatus->bottomLeft.display, buf);
            emsStatus->bottomLeft.value = newEmsData->outTemperature;
        }
        if (emsStatus->bottomRight.display && emsStatus->bottomRight.value != newEmsData->rpm)
        {
            snprintf(buf, sizeof(buf), "%d", newEmsData->rpm);
            lv_label_set_text(emsStatus->bottomRight.display, buf);
            emsStatus->bottomRight.value = newEmsData->rpm;
        }
    }
    const lv_coord_t bar_w = RB_EMS_VERTICAL_BAR_WIDTH;
    const lv_coord_t bar_h = RB_EMS_VERTICAL_BAR_HEIGHT;

    for (int i = 0; i < RB_EMS_VERTICAL_BARS_NUMBER * 2; i++)
    {
        RB04_EMSSWidget *w = &emsStatus->verticalBars[i];
        if (w == NULL || w->barFill == NULL)
            continue;

        int sensor = i / 2;       /* logical sensor index 0..(N-1) */
        int isCHT = (i % 2) == 0; /* even positions = CHT, odd = EGT */
        int16_t tempC;
        if (isCHT)
            tempC = newEmsData->chtCelsius[sensor];
        else
            tempC = newEmsData->egtCelsius[sensor];

        /* convert to Fahrenheit for scale mapping */
        int16_t tempF = (int16_t)(tempC * 9.0f / 5.0f + 32.0f + 0.5f);

        /* Optimization: skip redraw for small fluctuations (<5°) */
        if (w->value != 0)
        {
            int16_t d = tempF - w->value;
            if (d < 0)
                d = -d;
            if (d < 4)
            {
                w->valueAverage = (w->valueAverage * 10 + tempF) / 11.0f;
                continue; /* no significant change */
            }
        }

        /* pick scale per-group using defines */
        int16_t minF = isCHT ? RB_EMS_CHT_MIN_F : RB_EMS_EGT_MIN_F;
        int16_t maxF = isCHT ? RB_EMS_CHT_MAX_F : RB_EMS_EGT_MAX_F;
        float normalized = (tempF - (float)minF) / (float)(maxF - minF);
        if (normalized < 0)
            normalized = 0;
        if (normalized > 1)
            normalized = 1;

        lv_coord_t fillH = (lv_coord_t)(normalized * (float)bar_h);
        lv_obj_set_size(w->barFill, bar_w, fillH);
        lv_obj_align(w->barFill, LV_ALIGN_CENTER, 0, bar_h / 2 - fillH / 2);

        /* color thresholds */
        lv_color_t color;
        if (tempF <= w->thresholdGreen)
            color = lv_color_make(0, 0, 0);
        if (tempF <= w->thresholdYellow)
            color = lv_color_make(0, 255, 0);
        else if (tempF <= w->thresholdRed)
            color = lv_color_make(255, 255, 0);
        else
            color = lv_color_make(255, 0, 0);
        lv_obj_set_style_bg_color(w->barFill, color, 0);

        /* update per-bar numeric label */

        if (status->ui.displayFarenheit)
            snprintf(buf, sizeof(buf), "%d°", tempF);
        else
            snprintf(buf, sizeof(buf), "%d°", tempC);
        if (w->label)
            lv_label_set_text(w->label, buf);

        /* triangle indicator: align above the filled portion */
        int xpos = (int)((i * (bar_w + RB_EMS_VERTICAL_BAR_SPACE)) - (RB_EMS_VERTICAL_BARS_NUMBER * 2 / 2.0 * (bar_w + RB_EMS_VERTICAL_BAR_SPACE)) + (bar_w + RB_EMS_VERTICAL_BAR_SPACE) / 2.0);
        int triY = (int)((bar_h / 2) - fillH + 12);
        if (w->triangleDirection)
        {
            lv_obj_align(w->triangleDirection, LV_ALIGN_CENTER, xpos, triY);
            if (w->valueAverage > tempF + 10)
                lv_label_set_text(w->triangleDirection, LV_SYMBOL_DOWN);
            else if (w->valueAverage < tempF - 10)
                lv_label_set_text(w->triangleDirection, LV_SYMBOL_UP);
            else
                lv_label_set_text(w->triangleDirection, "="); /* hide if no change */
        }

        /* per-bar maximum indicator: update max and position the line */
        if (w->maxLine)
        {
            if (tempF > w->maxValue)
                w->maxValue = tempF;
            if (w->maxValue > -9999)
            {
                float normalizedMax = (w->maxValue - (float)minF) / (float)(maxF - minF);
                if (normalizedMax < 0)
                    normalizedMax = 0;
                if (normalizedMax > 1)
                    normalizedMax = 1;
                lv_coord_t maxFillH = (lv_coord_t)(normalizedMax * (float)bar_h);
                lv_coord_t maxY = (bar_h / 2) - maxFillH;
                lv_obj_clear_flag(w->maxLine, LV_OBJ_FLAG_HIDDEN);
                lv_obj_align(w->maxLine, LV_ALIGN_CENTER, xpos, maxY);
            }
            else
            {
                lv_obj_add_flag(w->maxLine, LV_OBJ_FLAG_HIDDEN);
            }
        }

        w->value = tempF;
        w->valueAverage = (w->valueAverage * 10 + tempF) / 11.0f;
    }
}
void RB04_EMS_Touch_N(void *status)
{
}
void RB04_EMS_Touch_S(void *status)
{
}
void RB04_EMS_Touch_W(void *status)
{
}
void RB04_EMS_Touch_E(void *status)
{
}
#endif