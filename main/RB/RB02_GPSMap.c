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
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/
#include "RB02_GPSMap.h"
#ifdef RB_ENABLE_MAP
#include "lvgl.h"
#include <stdio.h>
#include <math.h>
#include "RB02_GUIHelpers.h"

#ifdef RB_02_ENABLE_INTERNALMAP
#include "images/GPSMapInternal.c"
#else
#define RB_GPS_MAP_MAX_ZOOM 4
#endif

//#define RB_ENABLE_CONSOLE_DEBUG 1
// #define RB_MAP_MULTI_TILE 1
extern lv_style_t style_title;

void RB02_GPSMap_ShowLoading(RB02_GpsMapStatus *gpsMapStatus, bool show)
{
#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("RB02_GPSMap_ShowLoading %d\n", show);
#endif
    if(show)
    {
        lv_obj_clear_flag(gpsMapStatus->labelLoading, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(gpsMapStatus->labelLoading, LV_OBJ_FLAG_HIDDEN);
    }
}

uint8_t zoomSlicerMapper(int8_t zoomLevel)
{
    uint8_t r = 100;
    switch (zoomLevel)
    {
    case 1:
        r = 50;
        break;
    case 2:
        r = 20;
        break;
    case 3:
        r = 10;
        break;
    default:
        break;
    }
    return r;
}

uint8_t zoomSlicerMapperMBTiles(int8_t zoomLevel)
{
    // TODO: Check if the array is aligned to RB_GPS_MAP_MAX_ZOOM
    uint8_t mbTilesMappping[4] = {8, 9, 10, 11};
    if (zoomLevel < 0)
        zoomLevel = 0;
    if (zoomLevel >= sizeof(mbTilesMappping))
        zoomLevel = sizeof(mbTilesMappping) - 1;
    return mbTilesMappping[zoomLevel];
}

// Support for MBTILES
#define PI 3.14159265358979323846

// Structure to hold bounding box
typedef struct
{
    double minLon;
    double minLat;
    double maxLon;
    double maxLat;
} BBox;

// Converts tile (x, y in TMS) and zoom to bounding box in WGS84

BBox tile_to_bbox_tms(int x, int y_tms, int z)
{
    int n = 1 << z; // 2^z

    // Convert TMS y to XYZ y
    int y = n - 1 - y_tms;

    // Longitude (easy)
    double minLon = x / (double)n * 360.0 - 180.0;
    double maxLon = (x + 1) / (double)n * 360.0 - 180.0;

    // Latitude (via Web Mercator inverse)
    double lat_rad1 = atan(sinh(PI - 2.0 * PI * y / (double)n));
    double lat_rad2 = atan(sinh(PI - 2.0 * PI * (y + 1) / (double)n));

    double maxLat = lat_rad1 * 180.0 / PI;
    double minLat = lat_rad2 * 180.0 / PI;
    if (maxLat < 0)
    {
        maxLat = -maxLat;
    }
    if (minLat < 0)
    {
        minLat = -minLat;
    }

    BBox bbox = {minLon, minLat, maxLon, maxLat};
    return bbox;
}

// Converts latitude, longitude, zoom → tile X/Y (TMS)
TileXY latlon_to_tile_xyz(double lat_deg, double lon_deg, int zoom)
{
    int n = 1 << zoom; // 2^zoom

    // Clamp latitude to Mercator limits
    if (lat_deg > 85.05112878)
        lat_deg = 85.05112878;
    if (lat_deg < -85.05112878)
        lat_deg = -85.05112878;

    // Convert longitude to [0, 1] range
    double x = (lon_deg + 180.0) / 360.0 * n;

    // Convert latitude to Mercator Y
    double lat_rad = lat_deg * M_PI / 180.0;
    double y = (1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI) / 2.0 * n;

    // Convert Y to TMS format (Y=0 at bottom)
    int x_tile = (int)floor(x);
    int y_xyz = (int)floor(y);
    // int y_tms = n - 1 - y_xyz;

    TileXY tile = {x_tile, y_xyz};
    return tile;
}

void RB02_GPSMap_SquareGenerator(RB02_GpsMapStatus *gpsMapStatus, float centerLatitude, float centerLongitude)
{
#ifdef RB_02_ENABLE_EXTERNALMAP
    if (gpsMapStatus->enableMercatoreLatLon == true)
    {
        // RB Standard to keep it Rect by decimals
        gpsMapStatus->tileSizeWidth = 360;
        gpsMapStatus->tileSizeHeight = 480;

        // TODO: Move this part to a function
        int16_t centerLatitudeInt = centerLatitude * 100;
        int16_t centerLongitudeInt = centerLongitude * 100;
        centerLatitudeInt = (centerLatitudeInt / zoomSlicerMapper(gpsMapStatus->zoomLevel)) * zoomSlicerMapper(gpsMapStatus->zoomLevel);
        centerLongitudeInt = (centerLongitudeInt / zoomSlicerMapper(gpsMapStatus->zoomLevel)) * zoomSlicerMapper(gpsMapStatus->zoomLevel);

        int32_t latb = centerLatitudeInt + (zoomSlicerMapper(gpsMapStatus->zoomLevel));
        int32_t late = centerLatitudeInt;

        int32_t lob = centerLongitudeInt;
        int32_t loe = centerLongitudeInt + (zoomSlicerMapper(gpsMapStatus->zoomLevel));

        if (gpsMapStatus->mapLatitudeBegin != latb || gpsMapStatus->mapLongitudeBegin != lob)
        {
            if (gpsMapStatus->mapDirty == 0)
            {
                gpsMapStatus->mapDirty = 0xff;
                RB02_GPSMap_ShowLoading(gpsMapStatus, true);
            }
        }

        gpsMapStatus->mapLatitudeBegin = latb;
        gpsMapStatus->mapLatitudeEnd = late;
        gpsMapStatus->mapLongitudeBegin = lob;
        gpsMapStatus->mapLongitudeEnd = loe;
    }
    else
    {
        // MBTiles standard
        gpsMapStatus->tileSizeWidth = 256;
        gpsMapStatus->tileSizeHeight = 256;

        TileXY tileCoordinates = latlon_to_tile_xyz(centerLatitude, centerLongitude, zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel));
        BBox box = tile_to_bbox_tms(tileCoordinates.x, tileCoordinates.y_tms, zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel));

        gpsMapStatus->mapLatitudeBegin = box.minLat * 100;
        gpsMapStatus->mapLatitudeEnd = box.maxLat * 100;
        gpsMapStatus->mapLongitudeBegin = box.minLon * 100;
        gpsMapStatus->mapLongitudeEnd = box.maxLon * 100;

        if (tileCoordinates.x != gpsMapStatus->lastTile.x || tileCoordinates.y_tms != gpsMapStatus->lastTile.y_tms)
        {
            if (gpsMapStatus->mapDirty == 0)
            {
                gpsMapStatus->mapDirty = 0xff;
                RB02_GPSMap_ShowLoading(gpsMapStatus, true);
            }
        }
    }

#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("RB02_GPSMap_SquareGenerator %f,%f, %ld %ld  %ld %ld\n", centerLatitude, centerLongitude, gpsMapStatus->mapLongitudeBegin, gpsMapStatus->mapLongitudeEnd, gpsMapStatus->mapLatitudeBegin, gpsMapStatus->mapLatitudeEnd);
#endif
#endif
}

void RB02_GPSMap_Touch_N(RB02_GpsMapStatus *gpsMapStatus)
{
#ifdef RB_02_ENABLE_EXTERNALMAP
    if (gpsMapStatus->mapDirty == 0)
    {
        if (gpsMapStatus->zoomLevel < RB_GPS_MAP_MAX_ZOOM)
        {
            gpsMapStatus->zoomLevel++;
        }
        gpsMapStatus->mapLatitudeBegin = 0;
        gpsMapStatus->latitude100 = 0;
        RB02_GPSMap_ShowLoading(gpsMapStatus, true);
    }
#endif
}
void RB02_GPSMap_Touch_S(RB02_GpsMapStatus *gpsMapStatus)
{
#ifdef RB_02_ENABLE_EXTERNALMAP
    if (gpsMapStatus->mapDirty == 0)
    {
        if (gpsMapStatus->zoomLevel > 0)
        {
            gpsMapStatus->zoomLevel--;
        }
        else
        {
        }
        gpsMapStatus->mapLatitudeBegin = 0;
        gpsMapStatus->latitude100 = 0;
        RB02_GPSMap_ShowLoading(gpsMapStatus, true);
    }
#endif
}

void RB02_GPSMap_ReloadTiles(RB02_GpsMapStatus *gpsMapStatus, gps_t *gpsStatus, lv_obj_t *parent)
{
#ifdef RB_02_ENABLE_EXTERNALMAP
    //
    char filename[41];
    lv_coord_t SCREEN_WIDTH = lv_disp_get_hor_res(NULL);
    lv_coord_t SCREEN_HEIGHT = lv_disp_get_ver_res(NULL);
    for (int y = 0; y < SCREEN_HEIGHT / gpsMapStatus->tileSizeHeight; y++)
    {
        for (int x = 0; x < SCREEN_WIDTH / gpsMapStatus->tileSizeWidth; x++)
        {

            lv_obj_t *backgroundImage = gpsMapStatus->tiles[x * (SCREEN_WIDTH / gpsMapStatus->tileSizeWidth) + y];
            if (backgroundImage == NULL)
            {
                backgroundImage = lv_img_create(parent);
                gpsMapStatus->tiles[x * (SCREEN_WIDTH / gpsMapStatus->tileSizeWidth) + y] = backgroundImage;
            }
            else
            {
            }

            // Round display optimisation
            /*
            if ((x == 0 && y == 0) ||
                (x == (SCREEN_WIDTH / gpsMapStatus->tileSizeWidth) - 1 && y == 0) ||
                (x == 0 && y == (SCREEN_HEIGHT / gpsMapStatus->tileSizeHeight) - 1) ||
                (x == (SCREEN_WIDTH / gpsMapStatus->tileSizeWidth) - 1 && y == (SCREEN_HEIGHT / gpsMapStatus->tileSizeHeight) - 1)

            )
            {
                continue;
            }
            */

            //= 43660

            // 1.1.22
            if (gpsMapStatus->enableMercatoreLatLon == true)
            {
#ifdef RB_MAP_MULTI_TILE
                int32_t currentLatitudeTile = (gpsMapStatus->mapLatitudeBegin - y * zoomSlicerMapper(gpsMapStatus->zoomLevel)) / zoomSlicerMapper(gpsMapStatus->zoomLevel);
                int32_t currentLongitudeTile = (gpsMapStatus->mapLongitudeBegin + x * zoomSlicerMapper(gpsMapStatus->zoomLevel)) / zoomSlicerMapper(gpsMapStatus->zoomLevel);
#else
                int32_t currentLatitudeTile = (gpsMapStatus->mapLatitudeBegin);
                int32_t currentLongitudeTile = (gpsMapStatus->mapLongitudeBegin);
#endif

                uint8_t folderIndex = 100 / zoomSlicerMapper(gpsMapStatus->zoomLevel);
                snprintf(filename, sizeof(filename), "S:/z%d/%04ld%04ld.bmp", folderIndex, currentLatitudeTile, currentLongitudeTile);
            }
            else
            {
                TileXY tileCoordinates = latlon_to_tile_xyz(gpsStatus->latitude, gpsStatus->longitude, zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel));

                snprintf(filename, sizeof(filename), "/sdcard/%d/%d/%d.bmp", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x, tileCoordinates.y_tms);
                // Test file exists:
                FILE *f = fopen(filename, "r");
                if (f == NULL)
                {
                    printf("Try to open: %s FAILURE\n", filename);
                }
                else
                {
                    printf("Try to open: %s SUCCESS\n", filename);
                    fclose(f);
                }
                snprintf(filename, sizeof(filename), "S:/%d/%d/%d.bmp", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x, tileCoordinates.y_tms);

                gpsMapStatus->lastTile = tileCoordinates;
            }
            lv_label_set_text(gpsMapStatus->labelTilePath, filename);
            lv_img_set_src(backgroundImage, filename);
            // lv_obj_set_size(backgroundImage, gpsMapStatus->tileSizeWidth, gpsMapStatus->tileSizeHeight);
            lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
#ifdef RB_MAP_MULTI_TILE
            int16_t lx = x * gpsMapStatus->tileSizeWidth;
            int16_t ly = y * gpsMapStatus->tileSizeHeight;
#else
            int16_t lx = 0;
            int16_t ly = 0;
#endif
            lv_obj_align(backgroundImage, LV_ALIGN_CENTER, lx, ly);
            lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
            printf("Tile:  %dx%d %s %dx%d %dx%d %dx%d\n", x, y, filename, lv_obj_get_width(backgroundImage), lv_obj_get_height(backgroundImage), lx, ly, gpsMapStatus->tileSizeWidth, gpsMapStatus->tileSizeHeight);
        }
    }

    lv_obj_move_foreground(gpsMapStatus->poiMy);
    lv_obj_move_foreground(gpsMapStatus->labelLatitude);
    lv_obj_move_foreground(gpsMapStatus->labelLongitude);
    gpsMapStatus->mapDirty = 0;
#endif
}

void RB02_GPSMap_ReloadMBTilesLoadTile(RB02_GpsMapStatus *gpsMapStatus, uint8_t index, lv_obj_t *parent, const char *filename, int16_t ox, int16_t oy, int16_t w, int16_t h, int16_t lx, int16_t ly)
{

    lv_obj_t *backgroundImage = gpsMapStatus->tiles[index];
    if (backgroundImage == NULL)
    {
        backgroundImage = lv_img_create(parent);
        gpsMapStatus->tiles[index] = backgroundImage;

        lv_obj_move_foreground(gpsMapStatus->labelLoading);
        lv_obj_move_foreground(gpsMapStatus->poiMy);
        lv_obj_move_foreground(gpsMapStatus->labelLatitude);
        lv_obj_move_foreground(gpsMapStatus->labelLongitude);
    }
    else
    {
    }

    lv_label_set_text(gpsMapStatus->labelTilePath, filename);
    lv_img_set_src(backgroundImage, filename);
    lv_obj_align(backgroundImage, LV_ALIGN_CENTER, lx, ly);
    // lv_img_set_pivot(backgroundImage, 0, 0);
    lv_img_set_offset_x(backgroundImage, -ox);
    lv_img_set_offset_y(backgroundImage, -oy);
    lv_obj_set_size(backgroundImage, w, h);
    lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
}

#define RB_GPS_MAP_TILE_N 2
#define RB_GPS_MAP_TILE_NE 4
#define RB_GPS_MAP_TILE_W 8
#define RB_GPS_MAP_TILE_C 1
#define RB_GPS_MAP_TILE_E 16
#define RB_GPS_MAP_TILE_SW 32
#define RB_GPS_MAP_TILE_S 64
#define RB_GPS_MAP_TILE_SE 128

void RB02_GPSMap_ReloadMBTiles(RB02_GpsMapStatus *gpsMapStatus, gps_t *gpsStatus, lv_obj_t *parent)
{
#ifdef RB_02_ENABLE_EXTERNALMAP
    //
    char filename[41];
    TileXY tileCoordinates = latlon_to_tile_xyz(gpsStatus->latitude, gpsStatus->longitude, zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel));

    uint16_t widthMax = 256;
    uint8_t widthMedium = 112;
    uint8_t widthSmall = 72;

    if (gpsMapStatus->mapDirty & RB_GPS_MAP_TILE_C)
    {
        snprintf(filename, sizeof(filename), "%s/%d/%d/%d.bmp", "/sdcard", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x, tileCoordinates.y_tms);
        if (RB02_CheckfileExists(filename) == 1)
        {
            snprintf(filename, sizeof(filename), "S:/%d/%d/%d.bmp", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x, tileCoordinates.y_tms);
            RB02_GPSMap_ReloadMBTilesLoadTile(gpsMapStatus, 4, parent, filename, 0, 0, widthMax, widthMax, 0, 0);
        }
        lv_label_set_text(gpsMapStatus->labelTilePath, filename);
        gpsMapStatus->mapDirty = gpsMapStatus->mapDirty & ~RB_GPS_MAP_TILE_C;
        gpsMapStatus->lastTile = tileCoordinates;
        return;
    }

    uint8_t w = widthSmall;

    if (gpsMapStatus->mapDirty & RB_GPS_MAP_TILE_N)
    {
        snprintf(filename, sizeof(filename), "%s/%d/%d/%d.bmp", "/sdcard", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x - 1, tileCoordinates.y_tms - 1);
        if (RB02_CheckfileExists(filename) == 1)
        {
            snprintf(filename, sizeof(filename), "%s/%d/%d/%d.bmp", "S:", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x - 1, tileCoordinates.y_tms - 1);
            RB02_GPSMap_ReloadMBTilesLoadTile(gpsMapStatus, 0, parent, filename, widthMax - w, widthMax - w, w, w, -widthMax / 2 - w / 2, -widthMax / 2 - w / 2);
        }
    }

    if (gpsMapStatus->mapDirty & RB_GPS_MAP_TILE_N)
    {
        snprintf(filename, sizeof(filename), "%s/%d/%d/%d.bmp", "/sdcard", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x, tileCoordinates.y_tms - 1);
        if (RB02_CheckfileExists(filename) == 1)
        {
            w = widthMedium;
            snprintf(filename, sizeof(filename), "S:/%d/%d/%d.bmp", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x, tileCoordinates.y_tms - 1);
            RB02_GPSMap_ReloadMBTilesLoadTile(gpsMapStatus, 1, parent, filename, 0, widthMax - w, widthMax, w, 0, -widthMax / 2 - w / 2);
        }
        gpsMapStatus->mapDirty = gpsMapStatus->mapDirty & ~RB_GPS_MAP_TILE_N;
        return;
    }

    if (gpsMapStatus->mapDirty & RB_GPS_MAP_TILE_NE)
    {
        snprintf(filename, sizeof(filename), "%s/%d/%d/%d.bmp", "/sdcard", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x + 1, tileCoordinates.y_tms - 1);
        if (RB02_CheckfileExists(filename) == 1)
        {
            w = widthSmall;
            snprintf(filename, sizeof(filename), "S:/%d/%d/%d.bmp", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x + 1, tileCoordinates.y_tms - 1);
            RB02_GPSMap_ReloadMBTilesLoadTile(gpsMapStatus, 2, parent, filename, 0, widthMax - w, w, w, widthMax / 2 + w / 2, -widthMax / 2 - w / 2);
        }
        gpsMapStatus->mapDirty = gpsMapStatus->mapDirty & ~RB_GPS_MAP_TILE_NE;
        return;
    }

    if (gpsMapStatus->mapDirty & RB_GPS_MAP_TILE_W)
    {
        snprintf(filename, sizeof(filename), "%s/%d/%d/%d.bmp", "/sdcard", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x - 1, tileCoordinates.y_tms);
        if (RB02_CheckfileExists(filename) == 1)
        {
            w = widthMedium;
            snprintf(filename, sizeof(filename), "S:/%d/%d/%d.bmp", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x - 1, tileCoordinates.y_tms);
            RB02_GPSMap_ReloadMBTilesLoadTile(gpsMapStatus, 3, parent, filename, widthMax - w, 0, w, widthMax, -widthMax / 2 - w / 2, 0);
        }
        gpsMapStatus->mapDirty = gpsMapStatus->mapDirty & ~RB_GPS_MAP_TILE_W;
        return;
    }

    if (gpsMapStatus->mapDirty & RB_GPS_MAP_TILE_E)
    {
        snprintf(filename, sizeof(filename), "%s/%d/%d/%d.bmp", "/sdcard", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x + 1, tileCoordinates.y_tms);
        if (RB02_CheckfileExists(filename) == 1)
        {
            w = widthMedium;
            snprintf(filename, sizeof(filename), "S:/%d/%d/%d.bmp", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x + 1, tileCoordinates.y_tms);
            RB02_GPSMap_ReloadMBTilesLoadTile(gpsMapStatus, 5, parent, filename, 0, 0, w, widthMax, widthMax / 2 + w / 2, 0);
        }
        gpsMapStatus->mapDirty = gpsMapStatus->mapDirty & ~RB_GPS_MAP_TILE_E;
        return;
    }

    if (gpsMapStatus->mapDirty & RB_GPS_MAP_TILE_SW)
    {
        snprintf(filename, sizeof(filename), "%s/%d/%d/%d.bmp", "/sdcard", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x - 1, tileCoordinates.y_tms + 1);
        if (RB02_CheckfileExists(filename) == 1)
        {
            w = widthSmall;
            snprintf(filename, sizeof(filename), "S:/%d/%d/%d.bmp", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x - 1, tileCoordinates.y_tms + 1);
            RB02_GPSMap_ReloadMBTilesLoadTile(gpsMapStatus, 6, parent, filename, widthMax - w, 0, w, w, -widthMax / 2 - w / 2, widthMax / 2 + w / 2);
        }
        gpsMapStatus->mapDirty = gpsMapStatus->mapDirty & ~RB_GPS_MAP_TILE_SW;
        return;
    }

    if (gpsMapStatus->mapDirty & RB_GPS_MAP_TILE_S)
    {
        snprintf(filename, sizeof(filename), "%s/%d/%d/%d.bmp", "/sdcard", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x, tileCoordinates.y_tms + 1);
        if (RB02_CheckfileExists(filename) == 1)
        {
            w = widthMedium;
            snprintf(filename, sizeof(filename), "S:/%d/%d/%d.bmp", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x, tileCoordinates.y_tms + 1);
            RB02_GPSMap_ReloadMBTilesLoadTile(gpsMapStatus, 7, parent, filename, 0, 0, widthMax, w, 0, widthMax / 2 + w / 2);
        }
        gpsMapStatus->mapDirty = gpsMapStatus->mapDirty & ~RB_GPS_MAP_TILE_S;
        return;
    }

    if (gpsMapStatus->mapDirty & RB_GPS_MAP_TILE_SE)
    {
        snprintf(filename, sizeof(filename), "%s/%d/%d/%d.bmp", "/sdcard", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x + 1, tileCoordinates.y_tms + 1);
        if (RB02_CheckfileExists(filename) == 1)
        {
            w = widthSmall;
            snprintf(filename, sizeof(filename), "S:/%d/%d/%d.bmp", zoomSlicerMapperMBTiles(gpsMapStatus->zoomLevel), tileCoordinates.x + 1, tileCoordinates.y_tms + 1);
            RB02_GPSMap_ReloadMBTilesLoadTile(gpsMapStatus, 8, parent, filename, 0, 0, w, w, widthMax / 2 + w / 2, widthMax / 2 + w / 2);
        }
        gpsMapStatus->mapDirty = gpsMapStatus->mapDirty & ~RB_GPS_MAP_TILE_SE;
    }

#endif
}

lv_obj_t *RB02_GPSMap_CreateScreen(RB02_GpsMapStatus *gpsMapStatus, lv_obj_t *parent)
{

    lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    gpsMapStatus->mapLatitudeBegin = 0;
    gpsMapStatus->mapLongitudeBegin = 0;
    gpsMapStatus->mapLatitudeEnd = 0;
    gpsMapStatus->mapLongitudeEnd = 0;
    // 1.1.22
    // true=Mercatore means: z%d/%d%d.bmp zoom,lat,lon
    // false=XYZ projection z%d/%d_%d.bmp zoom,x,y

#ifdef RB_02_ENABLE_EXTERNALMAP
    gpsMapStatus->enableMercatoreLatLon = false;
    for (int x = 0; x < 9; x++)
    {
        gpsMapStatus->tiles[x] = NULL;
    }
#else

#endif
    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
        lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, "GPS Map");
    }
    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_label_set_text(label, "-------");
        gpsMapStatus->labelTilePath = label;
    }
    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, -168);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, "LOADING");
        gpsMapStatus->labelLoading = label;
    }

#ifdef RB_02_ENABLE_INTERNALMAP

    if (true)
    {
        lv_obj_t *backgroundImage = lv_img_create(parent);
        lv_img_set_src(backgroundImage, &GPSMapInternal);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);
        // 1.1.9 Remove scrolling for Turbolence touch screen
        lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    }

    //
    gpsMapStatus->mapLatitudeBegin = 426380;
    gpsMapStatus->mapLongitudeBegin = 111950;
    gpsMapStatus->mapLatitudeEnd = 433430;
    gpsMapStatus->mapLongitudeEnd = 125000;

    //
    gpsMapStatus->mapLatitudeBegin = 4366;
    gpsMapStatus->mapLongitudeBegin = 1146;
    gpsMapStatus->mapLatitudeEnd = 4298;
    gpsMapStatus->mapLongitudeEnd = 1235;

    gpsMapStatus->tileSizeWidth = 480;
    gpsMapStatus->tileSizeHeight = 480;

#endif

    if (true)
    {
        lv_obj_t *poi = lv_obj_create(parent);
        lv_obj_set_scrollbar_mode(poi, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_size(poi, 16, 16);
        lv_obj_align(poi, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(poi, lv_color_make(0x0, 0x0, 0xff), 0);
        lv_obj_set_style_radius(poi, LV_RADIUS_CIRCLE, 0);
        lv_obj_clear_flag(poi, LV_OBJ_FLAG_CLICKABLE);
        gpsMapStatus->poiMy = poi;
    }

    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 196);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, "-------");
        gpsMapStatus->labelLatitude = label;
    }

    if (true)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 218);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(label, "--------");
        gpsMapStatus->labelLongitude = label;
    }

    return NULL;
}


#define RB_GPS_MAP_PRECISION 500.0

void RB02_GPSMap_Touch_W(RB02_GpsMapStatus *gpsMapStatus)
{
    RB02_GPSMap_ShowLoading(gpsMapStatus, true);
    gpsMapStatus->latitude100 = 0;
}

void RB02_GPSMap_Touch_E(RB02_GpsMapStatus *gpsMapStatus)
{
    RB02_GPSMap_ShowLoading(gpsMapStatus, true);
    gpsMapStatus->latitude100 = 0;
}

void RB02_GPSMap_Tick(RB02_GpsMapStatus *gpsMapStatus, gps_t *gpsStatus, lv_obj_t *parent)
{

#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("RB02_GPSMap_Tick\n");
#endif

    // Some example GPS Coordinates
    // Arezzo
    // gpsStatus->latitude = 43.45513012173618;
    // gpsStatus->longitude = 11.847354299755676;
    // gpsStatus->latitude= 43.11126;
    // gpsStatus->longitude = 12.08190;

    // Sassuolo
    //gpsStatus->latitude =44.56994865496443;
    //gpsStatus->longitude = 10.779384070431439;

    /*
        static gps_t gpsSimulator;
        if(gpsSimulator.latitude<0.001 && gpsSimulator.latitude>-0.001)
        {
                gpsSimulator.latitude = ;
                gpsSimulator.longitude = ;
        }
        gpsSimulator.latitude = gpsSimulator.latitude + 0.001;
        gpsSimulator.longitude = gpsSimulator.longitude + 0.001;


        gpsStatus->longitude = gpsSimulator.longitude;
        gpsStatus->latitude = gpsSimulator.latitude;
    */
    float currLon100 = (gpsStatus->longitude * RB_GPS_MAP_PRECISION);
    float currLat100 = (gpsStatus->latitude * RB_GPS_MAP_PRECISION);

    if (gpsMapStatus->latitude100 == (int32_t)currLat100 && gpsMapStatus->longitude100 == (int32_t)currLon100)
    {
        return;
    }

    RB02_GPSMap_SquareGenerator(gpsMapStatus, gpsStatus->latitude, gpsStatus->longitude);

    if (gpsMapStatus->mapLongitudeBegin - gpsMapStatus->mapLongitudeEnd == 0)
    {
        return;
    }
    if (gpsMapStatus->mapLatitudeBegin - gpsMapStatus->mapLatitudeEnd == 0)
    {
        return;
    }
#ifdef RB_02_ENABLE_EXTERNALMAP
    if (gpsMapStatus->mapDirty != 0)
    {
        if (gpsMapStatus->enableMercatoreLatLon == true)
        {
            RB02_GPSMap_ReloadTiles(gpsMapStatus, gpsStatus, parent);
        }
        else

        {
            RB02_GPSMap_ReloadMBTiles(gpsMapStatus, gpsStatus, parent);
        }
    }
#endif
    char buf[20];
    snprintf(buf, sizeof(buf), "LAT:%.5f", gpsStatus->latitude);
    lv_label_set_text(gpsMapStatus->labelLatitude, buf);
    snprintf(buf, sizeof(buf), "LON:%.5f", gpsStatus->longitude);
    lv_label_set_text(gpsMapStatus->labelLongitude, buf);

    //

    float longitudeEscursion = (gpsMapStatus->mapLongitudeEnd - gpsMapStatus->mapLongitudeBegin);
    float latitudeEscursion = (gpsMapStatus->mapLatitudeEnd - gpsMapStatus->mapLatitudeBegin);

    if (latitudeEscursion < 0)
    {
        latitudeEscursion = -latitudeEscursion;
    }
    if (longitudeEscursion < 0)
    {
        longitudeEscursion = -longitudeEscursion;
    }

    if (gpsMapStatus->mapDirty == 0)
    {
        gpsMapStatus->latitude100 = currLat100;
        gpsMapStatus->longitude100 = currLon100;
        RB02_GPSMap_ShowLoading(gpsMapStatus, false);
    }

    float diffLatitude = (gpsStatus->latitude * 100.0) - gpsMapStatus->mapLatitudeEnd;
    float diffLongitude = (gpsStatus->longitude * 100.0) - gpsMapStatus->mapLongitudeBegin;

    float percentageX = diffLongitude / longitudeEscursion;
    float percentageY = diffLatitude / latitudeEscursion;

    /*
        if (percentageX > 1.0)
        {
            percentageX = 1.0;
        }
        if (percentageX < 0)
        {
            percentageX = 0;
        }
        if (percentageY > 1.0)
        {
            percentageY = 1.0;
        }
        if (percentageY < 0)
        {
            percentageY = 0;
        }
    */

    lv_obj_align(gpsMapStatus->poiMy, LV_ALIGN_CENTER, gpsMapStatus->tileSizeWidth * (percentageX)-gpsMapStatus->tileSizeWidth / 2, gpsMapStatus->tileSizeHeight / 2 - gpsMapStatus->tileSizeHeight * (percentageY));
#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("%f %f %f %f %f %f %f %f %ld %ld %ld %ld\n", gpsStatus->latitude, gpsStatus->longitude,
           percentageX,
           percentageY,
           longitudeEscursion,
           latitudeEscursion,
           diffLatitude,
           diffLongitude,
           gpsMapStatus->mapLatitudeEnd,
           gpsMapStatus->mapLatitudeBegin,
           gpsMapStatus->mapLongitudeEnd,
           gpsMapStatus->mapLongitudeBegin);
#endif
}
#endif