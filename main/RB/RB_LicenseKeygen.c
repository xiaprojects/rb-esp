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
#include "esp_crc.h"

// Simple 64-bit hash: CRC32 of low 32 bits XOR high 32 bits (extend as needed)
static uint64_t compute_license_from_mac(uint64_t serial) {
    uint8_t mac_le[6];
    // Serial already has low 48 bits LE (from esp_efuse_mac_get_default((uint8_t*)&serial))
    memcpy(mac_le, &serial, 6);
    uint8_t mac_be[6];  // Reverse to BE for standard CRC
    for (int i = 0; i < 6; i++) mac_be[i] = mac_le[5 - i];
    
    uint32_t crc_low = esp_crc32_le(0, mac_be, 6);
    uint64_t license = ((uint64_t)crc_low << 32) | crc_low;
    return license ^ 0x0;
}
