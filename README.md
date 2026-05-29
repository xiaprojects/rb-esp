# RB-Avionics
The RB is the Open-source Aviation instruments suite, the models are divided by CPU/Features. RB-02 will run in ESP32 (Arduino).
- RB-01 -> Display with Synthetic vision, Autopilot and ADSB
- RB-02 -> Display with SixPack (this repo)
- RB-03 -> Display with Autopilot, ADSB, Radio, Flight Computer
- RB-04 -> Display with EMS: Engine monitoring system
- RB-05 -> Display with Stratux BLE Traffic (this repo)
- RB-06 -> Display with Android 6.25" 7" 8" 10" 10.2"
- RB-07 -> Display with Stratux BLE Traffic composed by RB-05 + RB-03 in the same box
- RB-08 -> Voice Recognition Box with LLM and Natural speaking and Voice Recorder
- RB-Cloud -> Cloud services for RB devices, including flight data recording, flight data analysis, flight sharing and more

More about in https://www.rbavionics.com and Discord community


## Community
The RB Avionics is a complete avionics project which is supported by a huge community worldwide, read the Wiki and if you need support join the Discord channel.

## How to
We strongly suggest to install binaries, that are built from building-machine which enrich and check the output.
Anyway if you are brave enough and you are well trained, you can built it yourself by following this procedure:
- https://github.com/xiaprojects/rb-esp/wiki/Customisations


### Features
1. Customisable splash screen (image that can be displayed when ever you want) 
3. GPS Speed, with KMH or KT configuration and colored arcs
4. Attitude indicator GPS Assisted
5. Advanced Attitude Indicator
6. Altimeter with QNH settings
7. Altimeter digital with GPS Altimeter comparation
8. Slip & Turn with degree indicator
9. Gyro based on Gyroscope assisted with GPS Track
10. GPS Map based on BMP Tiles, you can load your map MapBox in the SDCard
11. Variometer
12. G-Meter with Max-Negative stored
13. Timers: up to 3 timers with GPS UTC Clock
14. Checklist loaded from SDCard
15. Engine Hour Lifetime Chrono
16. Traffic Radar display from BLE Stratux
17. Traffic List from BLE Stratux
18. Data Logger writer to SD card


### Requirements
- Supported Display Devices are 2.8 and 2.1 Touch or not touch:
- https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8C
- https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.1
- Barometer Device: BMP280 or similar
- Flashing tool: https://www.waveshare.com/wiki/Flash_Firmware_Flashing_and_Erasing
- Case: you can find the STL on wiki
- GPS: any TTL NMEA GPS with $GPRMC such as uBlox NEO M6N and up
- Micro SD Card up to 32Gb 

### Installation
Please read the installation Wiki: to make it working the aircraft profile shall be correctly loaded in the setup.
We added a "vibration" correction setup: please verify with your mechanic which is the best options.

### Setup Screens
1. Setup
2. Vibration Setup
3. GPS Status
4. Bluetooth connection

### Video
- https://www.youtube.com/watch?v=9aCub_4hdxs
- https://www.youtube.com/watch?v=8FwM3qiaDVQ
- https://www.youtube.com/watch?v=Gy3Yu2zPZRM

### Flasher setup
- chip esp32s3
- flash_mode dio
- flash_size 16MB
- 0x0 Application Binary

If you have problems check Youtube guides and try a "full clear" during flash

## License
Community edition will be free for all builders and personal use as defined by the licensing model
 
Dual licensing for commercial agreement is available

This project is licensed under DUAL license:
1. the GNU AGPLv3. See the LICENSE file for details.
2. commercial agreement