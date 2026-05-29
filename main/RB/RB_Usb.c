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
 * 06 -> Display with Android 6.25" 7" 8" 10" 10.2"
 * 07 -> Display with Stratux BLE Traffic composed by RB-05 + RB-03 in the same box
 * 08 -> Voice Recognition Box with LLM and Natural speaking and Voice Recorder
 * Cloud -> Cloud services for RB devices, including flight data recording, flight data analysis, flight sharing and more
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
 * More about in https://www.rbavionics.com and Discord community
 * 
 */
#if defined(RB_ENABLE_USB_FLARM) ||  defined(RB_ENABLE_USB_NMEA)
/**
 * @brief USB Console input thread function
 * 
 * Reads and processes user input from USB console (stdin).
 * Handles commands like help, status, reset, etc.
 */
#if defined(RB_ENABLE_USB_FLARM)
void RB05_FlarmParser_ParseData(const uint8_t *data, uint8_t len);
#endif
#if defined(RB_ENABLE_USB_NMEA)
void NMEA_ParseBuffer(const uint8_t *data, const int rxBytes, uint8_t SourceId);
#endif
void usb_console_input_thread(void *parameter)
{

}
#endif
