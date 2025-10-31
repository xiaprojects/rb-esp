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


#include "Vendor.h"
#include "RB02_Config.h"

extern float GMeterScale;
extern uint8_t isKmh;
extern uint16_t degreeStart;
extern uint16_t degreeEnd;
extern uint16_t speedKtStart;
extern uint16_t speedKtEnd;
extern uint16_t speedWhite;
extern uint16_t speedGreen;
extern uint16_t speedYellow;
extern uint16_t speedRed;

extern IMUdata PanelAlignment;

extern uint8_t DriverLoopMilliseconds;
extern float AttitudeBalanceAlpha;
extern float FilterMoltiplier;
extern float FilterMoltiplierGyro;
extern float FilterMoltiplierOutput;

void nvsStorePCal();
void nvsStoreSpeedArc();
void nvsStoreFilters();
void nvsStoreGyroCalibration();

RB02_Status *VendorMakeDefaultsDefaultWithConfig(RB02_Status *config){
    config->settingsAutoQNH = 1;
    config->settingsCalibrateOnBoot = 0;
    config->structureVersion = RB02_STRUCTURE_CONFIG;
    config->bmp280override = 0;
    return config;
}


void VendorMakeDefaultsDefault()
{
    RB02_Status *config = singletonConfig();
    VendorMakeDefaultsDefaultWithConfig(config);
    GMeterScale = 3;

    // Example configuration for Limbach engine
    FilterMoltiplierOutput = 5.0;
    FilterMoltiplier = 2.0;
    FilterMoltiplierGyro = 4.0;
    AttitudeBalanceAlpha = 1.0/230.0;
    DriverLoopMilliseconds = 40;

    singletonConfig()->GyroHardwareCalibration.x = 0;
    singletonConfig()->GyroHardwareCalibration.y = 0;
    singletonConfig()->GyroHardwareCalibration.z = 0;
    singletonConfig()->bmp280override = 0;
    singletonConfig()->settingsAutoQNH = 0;
#ifdef RB02_ESP_BLUETOOTH
    singletonConfig()->settingsBluetoothEnabled = 0;
#endif

    PanelAlignment.x = 0;
    PanelAlignment.y = 0;
    PanelAlignment.z = 0;
}

#if ENABLE_VENDOR == RB_VENDOR_1
void VendorMakeDefaults()
{
    VendorMakeDefaultsDefault();

    isKmh = 0;
    degreeStart = 0;
    degreeEnd = 320;
    speedKtStart = 0;
    speedKtEnd = 55;
    speedWhite = 60;
    speedGreen = 105;
    speedYellow = 225;
    speedRed = 315;

    nvsStoreSpeedArc();
    nvsStorePCal();
    nvsStoreFilters();
    nvsStoreGyroCalibration();
}
#endif
#if ENABLE_VENDOR == RB_VENDOR_2
void VendorMakeDefaults()
{
    VendorMakeDefaultsDefault();

    nvsStoreSpeedArc();
    nvsStorePCal();
    nvsStoreFilters();
    nvsStoreGyroCalibration();
}
#endif
#if ENABLE_VENDOR == RB_VENDOR_3
void VendorMakeDefaults()
{
    VendorMakeDefaultsDefault();

    nvsStoreSpeedArc();
    nvsStorePCal();
    nvsStoreFilters();
    nvsStoreGyroCalibration();
}
#endif
