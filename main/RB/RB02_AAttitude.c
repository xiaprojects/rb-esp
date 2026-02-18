/**
 * This file is part of RB.
 *
 * Copyright (C) 2024 XIAPROJECTS SRL
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
#include "RB02_AAttitude.h"

#ifdef RB_ENABLE_AAT
#include "RB02_GUIHelpers.h"
#include <stdio.h>
#include <math.h>
#include "QMI8658.h"
#include "images/AAttitudeBackgroundPlain.c"
#include "images/AAttitudeBackgroundTriangleLeft.c"
#include "images/AAttitudeBackgroundTriangleRight.c"
#include "images/AAttitudeSkyLine.c"
#include "images/AAttitudeTopTriangle.c"
#include "images/AAttitudeTopLine.c"
#include "images/AAttitudeTopIndicator.c"

RB02_AdvancedAttitude_Status advancedAttitude_Status;

extern float GFactor;
extern uint8_t isKmh;
extern uint16_t degreeStart;
extern uint16_t degreeEnd;
extern uint16_t speedKtStart;
extern uint16_t speedKtEnd;
extern uint16_t speedWhite;
extern uint16_t speedGreen;
extern uint16_t speedYellow;
extern uint16_t speedRed;
extern float AttitudeYawDegreePerSecond;
extern IMUdata AccelFiltered;
extern const lv_res_t Screen_TurnSlip_Obj_Ball_Size;
extern float AttitudePitch;
extern float AttitudeRoll;

#define RB_AAT_START_LEFT_ARC 225
#define RB_AAT_START_RIGHT_ARC 135
#define RB_AAT_ARC_SIZE 6
//

static float sinfDegree(float angleDegree)
{
    int16_t angle = angleDegree;
    int16_t sinRes100 = lv_trigo_sin(angle);
    float result = sinRes100;
    result = result / 32767.0;
    return result;
}
static float cosfDegree(float angleDegree)
{
    int16_t angle = (angleDegree + 90);
    int16_t sinRes100 = lv_trigo_sin(angle);
    float result = sinRes100;
    result = result / 32767.0;
    return result;
}

int8_t pointAboveOrBelow(float x1, float y1, float x2, float y2, float px, float py)
{
    float cross = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1);
    if (cross > 0)
        return 1;
    if (cross < 0)
        return 0;
    // return 2;
    return 0;
}

typedef struct
{
    float x, y;
} Point;

void diameterEndpoints(float cx, float cy, float r, float angleDeg, Point *A, Point *B)
{
    // float angleRad = angleDeg * PI / 180.0f;
    float dx = cosfDegree(angleDeg);
    float dy = sinfDegree(angleDeg);

    A->x = cx + r * dx;
    A->y = cy + r * dy;
    B->x = cx - r * dx;
    B->y = cy - r * dy;
}

void lit(int8_t *matrix, float p, float r, uint8_t sizeRow, lv_obj_t **tiles)
{
    float radiansP = p * PI / 180.0f;

    //
    //
    float isLineBelow = sinf(radiansP) * sizeRow / 2.0f;
    float centerX = sizeRow / 2.0f;
    float centerY = sizeRow / 2.0f + isLineBelow;
    float radius = sizeRow / 2.0f;
    Point A, B;
    diameterEndpoints(centerX, centerY, radius, r, &A, &B);

    for (uint8_t y = 0; y < sizeRow; y++)
    {
        for (uint8_t x = 0; x < sizeRow; x++)
        {
            if (tiles[y * sizeRow + x] == NULL)
            {
                continue;
            }
            float px = x + 0.5;
            float py = y + 0.5;
            int8_t newLit = pointAboveOrBelow(A.x, A.y, B.x, B.y, px, py);
            if (matrix[y * sizeRow + x] == newLit)
            {
            }
            else
            {
                matrix[y * sizeRow + x] = newLit;

                if (newLit == 0)
                {
                    // lv_obj_add_flag(tiles[y * sizeRow + x], LV_OBJ_FLAG_HIDDEN);
                    lv_obj_set_style_bg_color(tiles[y * sizeRow + x], lv_color_make(128, 48, 0), 0);
                }
                else
                {
                    // lv_obj_clear_flag(tiles[y * sizeRow + x], LV_OBJ_FLAG_HIDDEN);
                    lv_obj_set_style_bg_color(tiles[y * sizeRow + x], lv_color_make(32, 168, 224), 0);
                }
            }
        }
    }
}

int8_t RB02_AdvancedAttitude_SkyMatrixAlign(RB02_AdvancedAttitude_Status *aaStatus, float pitch, float roll)
{

    int8_t rr = 10;
    int8_t rp = 10;

    //
    if (aaStatus->SkyMatrix == NULL)
    {
        aaStatus->SkyMatrix = (int8_t *)malloc(sizeof(int8_t) * RB_AAT_SKY_TILES * RB_AAT_SKY_TILES);

        if (aaStatus->SkyMatrix == NULL)
        {
            return -1;
        }
        memset(aaStatus->SkyMatrix, 2, sizeof(int8_t) * RB_AAT_SKY_TILES * RB_AAT_SKY_TILES);

        for (uint8_t y = 0; y < RB_AAT_SKY_TILES; y++)
        {
            for (uint8_t x = 0; x < RB_AAT_SKY_TILES; x++)
            {

                aaStatus->SkyTiles[y * RB_AAT_SKY_TILES + x] = NULL;

                int16_t tileX = -SCREEN_WIDTH / 2 + ((SCREEN_WIDTH / RB_AAT_SKY_TILES) * x + (SCREEN_WIDTH / RB_AAT_SKY_TILES / 2.0));
                int16_t tileY = -SCREEN_HEIGHT / 2 + ((SCREEN_HEIGHT / RB_AAT_SKY_TILES) * y + (SCREEN_HEIGHT / RB_AAT_SKY_TILES / 2.0));

                int32_t hypothenus = tileX * tileX + tileY * tileY;
                if (hypothenus > (SCREEN_WIDTH / 2) * (SCREEN_WIDTH / 2) + 4000 || tileY > 170)
                {
                    continue;
                }

                lv_obj_t *skyTile = lv_obj_create(aaStatus->lv_parent);
                lv_obj_clear_flag( skyTile, LV_OBJ_FLAG_CLICKABLE);
                lv_obj_set_scrollbar_mode(skyTile, LV_SCROLLBAR_MODE_OFF);
                // lv_obj_set_style_border_color(skyTile, lv_color_make(0, 0, 255), 0);
                lv_obj_set_size(skyTile, SCREEN_WIDTH / RB_AAT_SKY_TILES, SCREEN_HEIGHT / RB_AAT_SKY_TILES);
                lv_obj_set_style_bg_color(skyTile, lv_color_make(32, 168, 224), 0);
                lv_obj_set_style_border_width(skyTile, 0, LV_PART_MAIN);
                lv_obj_set_style_radius(skyTile, 0, LV_PART_MAIN);
                // lv_obj_add_flag(skyTile, LV_OBJ_FLAG_HIDDEN);
                lv_obj_align(skyTile, LV_ALIGN_CENTER, tileX, tileY);
                aaStatus->SkyTiles[y * RB_AAT_SKY_TILES + x] = skyTile;
            }
        }
    }
    //
    //

    lit(aaStatus->SkyMatrix, pitch * (rp / 10.0), -roll * (rr / 10.0), RB_AAT_SKY_TILES, aaStatus->SkyTiles);
    return 0;
}

//

void RB02_AdvancedAttitude_LinesAlign(RB02_AdvancedAttitude_Status *aaStatus, float pitch, float roll)
{
    float moveY = 4.0 * pitch;
    lv_obj_set_pos(aaStatus->Lines, 0, moveY);
    lv_img_set_angle(aaStatus->lv_roll, -roll * 10.0);

    lv_img_set_angle(aaStatus->AttitudeLine, -roll * 10.0);
    lv_obj_set_pos(aaStatus->AttitudeLine, 0, moveY);
}

//
void RB02_AdvancedAttitude_RollPitchAlign(RB02_AdvancedAttitude_Status *aaStatus, float pitch, float roll)
{
    float moveY = 4.0 * pitch;
    if (moveY > 120)
        moveY = 120;
    else
    {
        if (moveY < -100)
        {
            moveY = -100;
        }
    }

    // 1.1.1 Rotation of Aircraft symbol
    lv_img_set_angle(aaStatus->lv_pitch, -roll * 10.0);
    lv_obj_set_pos(aaStatus->lv_pitch, 0, -moveY + lv_obj_get_height(aaStatus->lv_pitch) / 2);

    lv_img_set_angle(aaStatus->lv_roll, -roll * 10.0);
}

int16_t RB02_AdvancedAttitude_MoveItems(lv_obj_t *item, int16_t degree, int16_t displacementX, int16_t displacementY)
{
    int16_t sin = lv_trigo_sin(degree - 90) / 327;
    int16_t cos = lv_trigo_cos(degree - 90) / 327;

    int16_t x = (cos * 150) / 100;
    int16_t y = (sin * 230) / 100;

    lv_obj_align(item, LV_ALIGN_CENTER, x + displacementX, y + displacementY);
    return y + displacementY;
}

int16_t RB02_AdvancedAttitude_ProgressBoth(RB02_AdvancedAttitude_Status *aaStatus, int32_t value, int32_t start, int32_t end)
{
    float degree = 0;
    if (value < start)
    {
        degree = 0;
    }
    else
    {
        if (value > end)
        {
            degree = 90;
        }
        else
        {
            if (end - start <= 0)
            {
                degree = 0;
            }
            else
            {
                degree = 90.0 * ((float)value - start) / (end - start);
            }
        }
    }

    return degree;
}

void RB02_AdvancedAttitude_MoveBall(lv_obj_t *item, uint16_t degree)
{
    int16_t sin = lv_trigo_sin(degree - 90) / 327;
    int16_t cos = lv_trigo_cos(degree - 90) / 327;

    int16_t x = (cos * 220) / 100;
    int16_t y = (sin * 220) / 100;

    lv_obj_align(item, LV_ALIGN_CENTER, x, y);
}

void RB02_AdvancedAttitude_Tick(RB02_AdvancedAttitude_Status *aaStatus, gps_t *gpsStatus, int32_t Altimeter, float QNH, int32_t Variometer)
{
    char buf[10];

    int16_t AltimeterInFeet = Altimeter / 100.0;

    if (aaStatus->Altimeter != AltimeterInFeet)
    {
        aaStatus->Altimeter = AltimeterInFeet;
        if (AltimeterInFeet > 100 || AltimeterInFeet < -100)
        {
            snprintf(buf, sizeof(buf), "%02d", abs(AltimeterInFeet % 100));
            lv_label_set_text(aaStatus->lv_altimeterF, buf);
            snprintf(buf, sizeof(buf), "%d", AltimeterInFeet / 100);
            lv_label_set_text(aaStatus->lv_altimeterM, buf);
        }
        else
        {
            snprintf(buf, sizeof(buf), "%d", AltimeterInFeet);
            lv_label_set_text(aaStatus->lv_altimeterF, buf);
            lv_label_set_text(aaStatus->lv_altimeterM, "");
        }

        // 1.1.24 Auto Scale Height
        if (aaStatus->advancedAttitudeMaxHeigh100 * 100 < AltimeterInFeet)
        {
            aaStatus->advancedAttitudeMaxHeigh100 = AltimeterInFeet / 100;
        }

        int16_t degreeAltimeterInFeet = RB_AAT_START_RIGHT_ARC - RB02_AdvancedAttitude_ProgressBoth(aaStatus, AltimeterInFeet, 0, aaStatus->advancedAttitudeMaxHeigh100 * 100);
        int16_t lineY = SCREEN_HEIGHT / 2 + RB02_AdvancedAttitude_MoveItems(aaStatus->lv_altimeterF, degreeAltimeterInFeet, 46, 0);
        RB02_AdvancedAttitude_MoveItems(aaStatus->lv_altimeterM, degreeAltimeterInFeet, -42, 0);
        RB02_AdvancedAttitude_MoveItems(aaStatus->lv_altimeter_background, degreeAltimeterInFeet, 0, 0);
        RB02_AdvancedAttitude_MoveItems(aaStatus->lv_altimeter_unit, degreeAltimeterInFeet, 0, -34);
        RB02_AdvancedAttitude_MoveItems(aaStatus->lv_variometer, degreeAltimeterInFeet, 0, +36);

        for (int position = 0; position < RB_AAT_ARC_NUMBERS; position++)
        {
            lv_coord_t y = lv_obj_get_y(aaStatus->lv_right_arcs[position]);
            if (lineY - 32 < y)
            {
                lv_obj_clear_flag(aaStatus->lv_right_arcs[position], LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                lv_obj_add_flag(aaStatus->lv_right_arcs[position], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    float isKt = 1.852;
    if (isKmh == 1)
    {
        isKt = 1.0;
    }
    int16_t speedValue = gpsStatus->speed / isKt;

    if (aaStatus->Speed != speedValue)
    {
        aaStatus->Speed = speedValue;
        snprintf(buf, sizeof(buf), "%d", speedValue);
        lv_label_set_text(aaStatus->lv_speed, buf);

        int16_t degreeSpeed = RB_AAT_START_LEFT_ARC + RB02_AdvancedAttitude_ProgressBoth(aaStatus, gpsStatus->speed, speedKtStart, speedKtEnd);
        RB02_AdvancedAttitude_MoveItems(aaStatus->lv_speed, degreeSpeed, 0, 0);
        RB02_AdvancedAttitude_MoveItems(aaStatus->lv_speed_background, degreeSpeed, 0, 0);
        RB02_AdvancedAttitude_MoveItems(aaStatus->lv_speed_unit, degreeSpeed, 0, -34);
        RB02_AdvancedAttitude_MoveItems(aaStatus->lv_gmeter, degreeSpeed, 0, +36);
    }

    if (aaStatus->Variometer != Variometer)
    {
        aaStatus->Variometer = Variometer;
        snprintf(buf, sizeof(buf), "%+ld", Variometer);
        lv_label_set_text(aaStatus->lv_variometer, buf);

        if (Variometer > 0)
        {
            for (int position = 0; position < RB_AAT_ARC_NUMBERS / 2; position++)
            {
                lv_obj_add_flag(aaStatus->lv_right_arcs2[position], LV_OBJ_FLAG_HIDDEN);
            }
            for (int position = RB_AAT_ARC_NUMBERS / 2; position < RB_AAT_ARC_NUMBERS; position++)
            {
                if ((Variometer / 100) > position - RB_AAT_ARC_NUMBERS / 2)
                {
                    lv_obj_clear_flag(aaStatus->lv_right_arcs2[position], LV_OBJ_FLAG_HIDDEN);
                }
                else
                {
                    lv_obj_add_flag(aaStatus->lv_right_arcs2[position], LV_OBJ_FLAG_HIDDEN);
                }
            }
        }
        else
        {
            for (int position = 0; position >= RB_AAT_ARC_NUMBERS / 2; position++)
            {
                lv_obj_add_flag(aaStatus->lv_right_arcs2[position], LV_OBJ_FLAG_HIDDEN);
            }
            for (int position = RB_AAT_ARC_NUMBERS / 2; position >= 0; position--)
            {
                if (-1 * (Variometer / 100) > (RB_AAT_ARC_NUMBERS / 2 - position))
                {
                    lv_obj_clear_flag(aaStatus->lv_right_arcs2[position], LV_OBJ_FLAG_HIDDEN);
                }
                else
                {
                    lv_obj_add_flag(aaStatus->lv_right_arcs2[position], LV_OBJ_FLAG_HIDDEN);
                }
            }
        }
    }

    if (aaStatus->GFactor != (int8_t)(GFactor * 10))
    {
        aaStatus->GFactor = GFactor * 10;
        snprintf(buf, sizeof(buf), "%.1fG", GFactor);
        lv_label_set_text(aaStatus->lv_gmeter, buf);
    }

    if (aaStatus->AttitudeYawDegreePerSecond != AttitudeYawDegreePerSecond)
    {
        aaStatus->AttitudeYawDegreePerSecond = AttitudeYawDegreePerSecond;
        if (AttitudeYawDegreePerSecond > 1)
        {
            snprintf(buf, sizeof(buf), "%.0f°", AttitudeYawDegreePerSecond);
        }
        else
        {
            if (AttitudeYawDegreePerSecond < -1)
            {
                snprintf(buf, sizeof(buf), "%.0f°", -AttitudeYawDegreePerSecond);
            }
            else
            {
                buf[0] = 0;
            }
        }

        if (AttitudeYawDegreePerSecond > 0)
        {
            lv_obj_set_size(aaStatus->lv_gyro_pink, (AttitudeYawDegreePerSecond * 10), 16);
        }
        else
        {
            lv_obj_set_size(aaStatus->lv_gyro_pink, (-AttitudeYawDegreePerSecond * 10), 16);
        }
        lv_obj_align(aaStatus->lv_gyro_pink, LV_ALIGN_CENTER, -(AttitudeYawDegreePerSecond * 10 / 2), 165);
        lv_label_set_text(aaStatus->lv_gyro, buf);
    }

    if (aaStatus->QNH != QNH)
    {
        aaStatus->QNH = QNH;
        if(singletonConfig()->qnhIsInInches){
            float QNHInInches = QNH / 33.8639;
            snprintf(buf, sizeof(buf), "%.2f", QNHInInches);
        } else {
            snprintf(buf, sizeof(buf), "%.0f", QNH);
        }
        lv_label_set_text(aaStatus->lv_qnh, buf);
    }

    if (aaStatus->Track != (int16_t)gpsStatus->cog)
    {
        aaStatus->Track = gpsStatus->cog;
        snprintf(buf, sizeof(buf), "%.0f°", gpsStatus->cog);
        lv_label_set_text(aaStatus->lv_track, buf);
    }

    if (aaStatus->AttitudePitch != (int8_t)AttitudePitch || aaStatus->AttitudeRoll != (int8_t)AttitudeRoll)
    {
        aaStatus->AttitudePitch = AttitudePitch;
        aaStatus->AttitudeRoll = AttitudeRoll;
        RB02_AdvancedAttitude_SkyMatrixAlign(aaStatus, AttitudePitch, AttitudeRoll);
        // RB02_AdvancedAttitude_RollPitchAlign(aaStatus, AttitudePitch, AttitudeRoll);
        RB02_AdvancedAttitude_LinesAlign(aaStatus, AttitudePitch, AttitudeRoll);
    }

    if (aaStatus->BallFactor != AccelFiltered.y * 10)
    {
        aaStatus->BallFactor = AccelFiltered.y * 10;
        RB02_AdvancedAttitude_MoveBall(aaStatus->lv_ball, 180 + AccelFiltered.y * 45);
    }
}

lv_color16_t RB02_AdvancedAttitude_GenerateColorAtIndexLeft(int index, int32_t value)
{
    float degreeTotal = degreeEnd - degreeStart;
    float degreeWhite = RB_AAT_ARC_NUMBERS * (speedWhite - degreeStart) / degreeTotal;
    float degreeGreen = RB_AAT_ARC_NUMBERS * (speedGreen - degreeStart) / degreeTotal;
    float degreeYellow = RB_AAT_ARC_NUMBERS * (speedYellow - degreeStart) / degreeTotal;
    float degreeRed = RB_AAT_ARC_NUMBERS * (speedRed - degreeStart) / degreeTotal;

    if (index > degreeRed)
    {
        return lv_color_make(255, 0, 0);
    }
    else
    {

        if (index > degreeYellow)
        {
            return lv_color_make(255, 255, 0);
        }
        else
        {
            if (index > degreeGreen)
            {
                return lv_color_make(0, 255, 0);
            }
            else
            {
                if (index > degreeWhite)
                {
                    return lv_color_make(255, 255, 255);
                }
                else
                {
                }
            }
        }
    }

    return lv_color_make(0x3c, 0x3c, 0x3c);
}
lv_color16_t RB02_AdvancedAttitude_GenerateColorAtIndexRight(int index, int32_t value)
{
    return lv_color_make(240, 240, 240);
}

lv_obj_t *RB02_AdvancedAttitude_CreateArcSlice(lv_obj_t *parent, lv_color16_t color, uint16_t degree)
{

    lv_obj_t *_pinkRect = lv_obj_create(parent);
    lv_obj_set_scrollbar_mode(_pinkRect, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_border_color(_pinkRect, color, 0);
    lv_obj_set_size(_pinkRect, 30, 18);
    lv_obj_set_style_bg_color(_pinkRect, color, 0);
    lv_obj_set_style_radius(_pinkRect, 4, LV_PART_MAIN);
    lv_obj_clear_flag( _pinkRect, LV_OBJ_FLAG_CLICKABLE);
    int16_t sin = lv_trigo_sin(degree - 90) / 327;
    int16_t cos = lv_trigo_cos(degree - 90) / 327;

    lv_obj_align(_pinkRect, LV_ALIGN_CENTER, (cos * 230) / 100, (sin * 230) / 100);

    // After
    return _pinkRect;
}

lv_obj_t *RB02_AdvancedAttitude_DrawLine(lv_obj_t *parent, lv_coord_t width, lv_coord_t height, lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *line = lv_obj_create(parent);
    lv_obj_set_scrollbar_mode(line, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(line, width, height);
    lv_obj_set_style_bg_color(line, lv_color_make(255, 255, 255), 0);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(line, 0, LV_PART_MAIN);
    lv_obj_align(line, LV_ALIGN_CENTER, x, y);
    lv_obj_clear_flag( line, LV_OBJ_FLAG_CLICKABLE);
    return line;
}

lv_obj_t *RB02_AdvancedAttitude_DrawText(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, const char *text)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(label, LV_ALIGN_CENTER, x, y);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_label_set_text(label, text);
    return label;
}

lv_obj_t *RB02_AdvancedAttitude_CreateScreen(RB02_AdvancedAttitude_Status *aaStatus)
{

    aaStatus->Altimeter = 1;
    aaStatus->QNH = 1;
    aaStatus->Variometer = 1;
    aaStatus->GFactor = 0;
    aaStatus->AttitudeYawDegreePerSecond = 1;
    aaStatus->BallFactor = 1;
    aaStatus->AttitudePitch = 1;
    aaStatus->AttitudeRoll = 1;
    aaStatus->Speed = 1;
    aaStatus->Track = 1;
    aaStatus->advancedAttitudeMaxHeigh100 = 3500 / 100;
/*
    if (aaStatus->lv_parent != NULL && false)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);

        lv_img_set_src(backgroundImage, &AAttitudeBackgroundPlain);

        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);
    }
*/
    RB02_AdvancedAttitude_SkyMatrixAlign(aaStatus, 0, 0);
    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);

        lv_img_set_src(backgroundImage, &AAttitudeSkyLine);

        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        aaStatus->AttitudeLine = backgroundImage;
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *cont = lv_obj_create(aaStatus->lv_parent);
        lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_size(cont, 120, 242);
        lv_obj_set_style_bg_opa(cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(cont, 0, LV_PART_MAIN);
        lv_obj_clear_flag( cont, LV_OBJ_FLAG_CLICKABLE);
        aaStatus->Lines = cont;
    }

    // RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 60, 2, 0, -160);
    // RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 20, 2, 0, -140);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 60, 2, 0, -120);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 20, 2, 0, -100);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 60, 2, 0, -80);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 20, 2, 0, -60);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 60, 2, 0, -40);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 20, 2, 0, -20);

    // RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 60, 2, 0, 160);
    // RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 20, 2, 0, 140);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 60, 2, 0, 120);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 20, 2, 0, 100);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 60, 2, 0, 80);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 20, 2, 0, 60);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 60, 2, 0, 40);
    RB02_AdvancedAttitude_DrawLine(aaStatus->Lines, 20, 2, 0, 20);

    // RB02_AdvancedAttitude_DrawText(aaStatus->Lines, 46, -120, "30");
    RB02_AdvancedAttitude_DrawText(aaStatus->Lines, 46, -80, "20");
    RB02_AdvancedAttitude_DrawText(aaStatus->Lines, 46, -40, "10");
    RB02_AdvancedAttitude_DrawText(aaStatus->Lines, 46, 40, "10");
    RB02_AdvancedAttitude_DrawText(aaStatus->Lines, 46, 80, "20");
    // RB02_AdvancedAttitude_DrawText(aaStatus->Lines, 46, 120, "30");
    // RB02_AdvancedAttitude_DrawText(aaStatus->Lines, -46, -120, "30");
    RB02_AdvancedAttitude_DrawText(aaStatus->Lines, -46, -80, "20");
    RB02_AdvancedAttitude_DrawText(aaStatus->Lines, -46, -40, "10");
    RB02_AdvancedAttitude_DrawText(aaStatus->Lines, -46, 40, "10");
    RB02_AdvancedAttitude_DrawText(aaStatus->Lines, -46, 80, "20");
    // RB02_AdvancedAttitude_DrawText(aaStatus->Lines, -46, 120, "30");

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, -170);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(label, lv_color_black(), 0);

        lv_label_set_text(label, RB_PRODUCT_TITLE "\nGPS ASSISTED\nNOT CERTIFIED");
    }

    for (int position = 0; position < RB_AAT_ARC_NUMBERS; position++)
    {
        uint16_t degree = RB_AAT_START_LEFT_ARC + RB_AAT_ARC_SIZE * position;
        aaStatus->lv_left_arcs[position] = RB02_AdvancedAttitude_CreateArcSlice(aaStatus->lv_parent, RB02_AdvancedAttitude_GenerateColorAtIndexLeft(position, 0), degree);
        lv_obj_clear_flag(aaStatus->lv_left_arcs[position], LV_OBJ_FLAG_CLICKABLE);
    }

    for (int position = 0; position < RB_AAT_ARC_NUMBERS; position++)
    {
        uint16_t degree = RB_AAT_START_RIGHT_ARC - RB_AAT_ARC_SIZE * position;
         lv_obj_t *arc=RB02_AdvancedAttitude_CreateArcSlice(aaStatus->lv_parent, lv_color_make(0x3c, 0x3c, 0x3c), degree);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    }

    for (int position = 0; position < RB_AAT_ARC_NUMBERS; position++)
    {
        uint16_t degree = RB_AAT_START_RIGHT_ARC - RB_AAT_ARC_SIZE * position;
        aaStatus->lv_right_arcs[position] = RB02_AdvancedAttitude_CreateArcSlice(aaStatus->lv_parent, RB02_AdvancedAttitude_GenerateColorAtIndexRight(position, 0), degree);
        lv_obj_add_flag(aaStatus->lv_right_arcs[position], LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(aaStatus->lv_right_arcs[position], LV_OBJ_FLAG_CLICKABLE);
    }

    // TODO: Shallbe unified with altimeter
    for (int position = 0; position < RB_AAT_ARC_NUMBERS; position++)
    {
        uint16_t degree = RB_AAT_START_RIGHT_ARC - RB_AAT_ARC_SIZE * position;
        aaStatus->lv_right_arcs2[position] = RB02_AdvancedAttitude_CreateArcSlice(aaStatus->lv_parent, lv_color_make(255, 0, 255), degree);
        lv_obj_add_flag(aaStatus->lv_right_arcs2[position], LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(aaStatus->lv_right_arcs2[position], LV_OBJ_FLAG_CLICKABLE);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *white = lv_obj_create(aaStatus->lv_parent);
        lv_obj_clear_flag(white, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(white, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_style_border_color(white, lv_color_white(), 0);
        lv_obj_set_size(white, 4, Screen_TurnSlip_Obj_Ball_Size);
        lv_obj_set_style_radius(white, 0, LV_PART_MAIN);
        lv_obj_align(white, LV_ALIGN_CENTER, 0 - Screen_TurnSlip_Obj_Ball_Size / 2 - 4, SCREEN_HEIGHT / 2 - Screen_TurnSlip_Obj_Ball_Size / 2);
        lv_obj_set_style_bg_color(white, lv_color_white(), 0);
    }
    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *white = lv_obj_create(aaStatus->lv_parent);
        lv_obj_clear_flag(white, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(white, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_style_border_color(white, lv_color_white(), 0);
        lv_obj_set_size(white, 4, Screen_TurnSlip_Obj_Ball_Size);
        lv_obj_set_style_radius(white, 0, LV_PART_MAIN);
        lv_obj_align(white, LV_ALIGN_CENTER, 0 + Screen_TurnSlip_Obj_Ball_Size / 2 + 4, SCREEN_HEIGHT / 2 - Screen_TurnSlip_Obj_Ball_Size / 2);
        lv_obj_set_style_bg_color(white, lv_color_white(), 0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 170, -10);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);

        lv_label_set_text(label, "feet");
        aaStatus->lv_altimeter_unit = label;
    }
    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, -170, -10);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        if (isKmh == 1)
        {
            lv_label_set_text(label, "KMH");
        }
        else
        {
            lv_label_set_text(label, "KT");
        }

        aaStatus->lv_speed_unit = label;
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, -170, 16);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);

        lv_obj_set_style_border_color(label, lv_color_black(), 0);
        lv_obj_set_style_bg_color(label, lv_color_black(), 0);

        aaStatus->lv_gmeter = label;
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *roundTile = lv_obj_create(aaStatus->lv_parent);
        lv_obj_clear_flag(roundTile, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(roundTile, 80, 28);
        lv_obj_align(roundTile, LV_ALIGN_CENTER, -105, 190);
        lv_obj_set_style_border_color(roundTile, lv_color_black(), 0);
        lv_obj_set_style_bg_color(roundTile, lv_color_black(), 0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, -105, 190);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);

        lv_obj_set_style_border_color(label, lv_color_black(), 0);
        lv_obj_set_style_bg_color(label, lv_color_black(), 0);

        aaStatus->lv_gyro = label;
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 170, 16);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);

        lv_obj_set_style_border_color(label, lv_color_black(), 0);
        lv_obj_set_style_bg_color(label, lv_color_black(), 0);

        aaStatus->lv_variometer = label;
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *roundTile = lv_obj_create(aaStatus->lv_parent);
        lv_obj_clear_flag(roundTile, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(roundTile, 80, 28);
        lv_obj_align(roundTile, LV_ALIGN_CENTER, 105, 190);
        lv_obj_set_style_border_color(roundTile, lv_color_black(), 0);
        lv_obj_set_style_bg_color(roundTile, lv_color_black(), 0);
    }
    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 105, 190);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);

        lv_obj_set_style_border_color(label, lv_color_black(), 0);
        lv_obj_set_style_bg_color(label, lv_color_black(), 0);

        aaStatus->lv_qnh = label;
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);

        lv_img_set_src(backgroundImage, &AAttitudeBackgroundTriangleRight);

        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        aaStatus->lv_altimeter_background = backgroundImage;
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);

        lv_img_set_src(backgroundImage, &AAttitudeBackgroundTriangleLeft);

        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        aaStatus->lv_speed_background = backgroundImage;
    }
    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, 88, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 155, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);

        aaStatus->lv_altimeterF = label;
    }
    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, 88, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 155, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);

        aaStatus->lv_altimeterM = label;
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, -155, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);

        aaStatus->lv_speed = label;
    }
    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *roundTile = lv_obj_create(aaStatus->lv_parent);
        lv_obj_clear_flag(roundTile, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(roundTile, 80, 28);
        lv_obj_align(roundTile, LV_ALIGN_CENTER, 0, 185);
        lv_obj_set_style_border_color(roundTile, lv_color_black(), 0);
        lv_obj_set_style_bg_color(roundTile, lv_color_black(), 0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *label = lv_label_create(aaStatus->lv_parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 4, 185);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);

        lv_obj_set_style_border_color(label, lv_color_black(), 0);
        lv_obj_set_style_bg_color(label, lv_color_black(), 0);

        aaStatus->lv_track = label;
    }

    /*

    if (aaStatus->lv_parent != NULL && false)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);
        lv_img_set_src(backgroundImage, AircraftIndicatorMiddle);
        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);
        lv_img_set_pivot(backgroundImage, AircraftIndicatorMiddle->header.w / 2, 0);
        lv_obj_set_pos(backgroundImage, 0, AircraftIndicatorMiddle->header.h / 2);
        aaStatus->lv_pitch = backgroundImage;
    }
        */

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *line = lv_obj_create(aaStatus->lv_parent);
        lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(line, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(line, 90, 12);
        lv_obj_set_style_bg_color(line, lv_color_make(255, 0, 0), 0);
        lv_obj_set_style_border_width(line, 2, LV_PART_MAIN);
        lv_obj_set_style_radius(line, 4, LV_PART_MAIN);
        lv_obj_align(line, LV_ALIGN_CENTER, -70, 0);
    }
    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *line = lv_obj_create(aaStatus->lv_parent);
        lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(line, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(line, 90, 12);
        lv_obj_set_style_bg_color(line, lv_color_make(255, 0, 0), 0);
        lv_obj_set_style_border_width(line, 2, LV_PART_MAIN);
        lv_obj_set_style_radius(line, 4, LV_PART_MAIN);
        lv_obj_align(line, LV_ALIGN_CENTER, 70, 0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *_screenBall = lv_obj_create(aaStatus->lv_parent);
        lv_obj_clear_flag(_screenBall, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(_screenBall, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(_screenBall, 24, 24);

        lv_obj_align(_screenBall, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(_screenBall, lv_color_make(255, 0, 0), 0);
        lv_obj_set_style_border_color(_screenBall, lv_color_white(), 0);
        lv_obj_set_style_radius(_screenBall, LV_RADIUS_CIRCLE, 0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);
        lv_obj_clear_flag(backgroundImage, LV_OBJ_FLAG_CLICKABLE);
        lv_img_set_src(backgroundImage, &AAttitudeTopLine);
        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_set_pos(backgroundImage, 0, -240 + (&AAttitudeTopLine)->header.h / 2);
        lv_img_set_pivot(backgroundImage, (&AAttitudeTopLine)->header.w / 2, 240);

        lv_img_set_angle(backgroundImage, -15 * 10.0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);
        lv_obj_clear_flag(backgroundImage, LV_OBJ_FLAG_CLICKABLE);
        lv_img_set_src(backgroundImage, &AAttitudeTopLine);
        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_set_pos(backgroundImage, 0, -240 + (&AAttitudeTopLine)->header.h / 2);
        lv_img_set_pivot(backgroundImage, (&AAttitudeTopLine)->header.w / 2, 240);

        lv_img_set_angle(backgroundImage, -30 * 10.0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);
        lv_obj_clear_flag(backgroundImage, LV_OBJ_FLAG_CLICKABLE);
        lv_img_set_src(backgroundImage, &AAttitudeTopLine);
        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_set_pos(backgroundImage, 0, -240 + (&AAttitudeTopLine)->header.h / 2);
        lv_img_set_pivot(backgroundImage, (&AAttitudeTopLine)->header.w / 2, 240);

        lv_img_set_angle(backgroundImage, 15 * 10.0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);
        lv_obj_clear_flag(backgroundImage, LV_OBJ_FLAG_CLICKABLE);
        lv_img_set_src(backgroundImage, &AAttitudeTopLine);
        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_set_pos(backgroundImage, 0, -240 + (&AAttitudeTopLine)->header.h / 2);
        lv_img_set_pivot(backgroundImage, (&AAttitudeTopLine)->header.w / 2, 240);

        lv_img_set_angle(backgroundImage, 30 * 10.0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);
        lv_obj_clear_flag(backgroundImage, LV_OBJ_FLAG_CLICKABLE);
        lv_img_set_src(backgroundImage, &AAttitudeTopTriangle);
        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_set_pos(backgroundImage, 0, -240 + (&AAttitudeTopTriangle)->header.h / 2);
        lv_img_set_pivot(backgroundImage, (&AAttitudeTopTriangle)->header.w / 2, 240);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);
        lv_obj_clear_flag(backgroundImage, LV_OBJ_FLAG_CLICKABLE);
        lv_img_set_src(backgroundImage, &AAttitudeTopTriangle);
        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_set_pos(backgroundImage, 0, -240 + (&AAttitudeTopTriangle)->header.h / 2);
        lv_img_set_pivot(backgroundImage, (&AAttitudeTopTriangle)->header.w / 2, 240);

        lv_img_set_angle(backgroundImage, 45 * 10.0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);
        lv_obj_clear_flag(backgroundImage, LV_OBJ_FLAG_CLICKABLE);
        lv_img_set_src(backgroundImage, &AAttitudeTopTriangle);
        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_set_pos(backgroundImage, 0, -240 + (&AAttitudeTopTriangle)->header.h / 2);
        lv_img_set_pivot(backgroundImage, (&AAttitudeTopTriangle)->header.w / 2, 240);

        lv_img_set_angle(backgroundImage, -45 * 10.0);
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *backgroundImage = lv_img_create(aaStatus->lv_parent);
        lv_obj_clear_flag(backgroundImage, LV_OBJ_FLAG_CLICKABLE);
        lv_img_set_src(backgroundImage, &AAttitudeTopIndicator);
        lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(aaStatus->lv_parent, LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(aaStatus->lv_parent, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_set_pos(backgroundImage, 0, -240 + (&AAttitudeTopIndicator)->header.h / 2);
        lv_img_set_pivot(backgroundImage, (&AAttitudeTopIndicator)->header.w / 2, 240);

        aaStatus->lv_roll = backgroundImage;
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *_screenBall = lv_obj_create(aaStatus->lv_parent);
        lv_obj_clear_flag(_screenBall, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(_screenBall, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(_screenBall, Screen_TurnSlip_Obj_Ball_Size, Screen_TurnSlip_Obj_Ball_Size);

        lv_obj_align(_screenBall, LV_ALIGN_CENTER, 0, 240 - Screen_TurnSlip_Obj_Ball_Size / 2);
        lv_obj_set_style_bg_color(_screenBall, lv_color_white(), 0);
        lv_obj_set_style_border_color(_screenBall, lv_color_black(), 0);
        lv_obj_set_style_radius(_screenBall, LV_RADIUS_CIRCLE, 0);

        aaStatus->lv_ball = _screenBall;
    }

    if (aaStatus->lv_parent != NULL)
    {
        lv_obj_t *_pinkRect = lv_obj_create(aaStatus->lv_parent);
        lv_obj_clear_flag(_pinkRect, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_scrollbar_mode(_pinkRect, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_style_border_color(_pinkRect, lv_color_make(255, 0, 255), 0);
        lv_obj_set_size(_pinkRect, 0, 16);

        lv_obj_align(_pinkRect, LV_ALIGN_CENTER, 0, 165);
        lv_obj_set_style_bg_color(_pinkRect, lv_color_make(255, 0, 255), 0);

        aaStatus->lv_gyro_pink = _pinkRect;
    }

    return NULL;
}

#endif