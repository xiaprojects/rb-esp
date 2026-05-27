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
#include "RB_License.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_mac.h"


static uint64_t compute_license_from_mac(uint64_t serial);

#if RB_LICENSE_TYPE == RB_LICENSE_TYPE_COMMERCIAL
#include "RB_LicenseKeygen.c"  // Contains compute_license_from_mac() declaration 
#else
static uint64_t compute_license_from_mac(uint64_t serial) {
    return true;
}
#endif
    
static const char *TAGLIC = "RB_LICENSE";

bool read_license_text(uint64_t *license) {
    FILE *f = fopen("/sdcard/license.txt", "r");
    if (!f || fscanf(f, "%llu", license) != 1) {
        if (f) fclose(f);
        return false;
    }
    fclose(f);
    return true;
}



bool read_license_from_sd(uint64_t *license) {
    FILE *f = fopen("/sdcard/license.bin", "rb");
    if (!f) {
        ESP_LOGE(TAGLIC, "Failed to open license.bin");
        return false;
    }
    size_t read = fread(license, sizeof(uint64_t), 1, f);
    fclose(f);
    if (read != 1) {
        ESP_LOGE(TAGLIC, "Failed to read license");
        return false;
    }
    ESP_LOGI(TAGLIC, "License read: 0x%016llX", *license);
    return true;
}


bool licenseValidated(uint64_t mac,uint64_t provided_license) {  // No param; reads from SD
    uint64_t expected = compute_license_from_mac(mac);  // Your hash func
    return (expected == provided_license);
}
