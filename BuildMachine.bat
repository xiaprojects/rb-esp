@echo off
for /f "delims=" %%i in ('git describe --tags') do set VERSION=%%i
md build\RB-%VERSION%

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-AAT"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py all
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-ALD"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-GMT"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-MAP"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-SPD"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-TRK"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-TRN"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin




set "TEMPLATE=NONTOUCH-30hz-GPSDIAG-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=TOUCH-30hz-GPDIAG-2.1"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=TOUCH-30hz-GPSDIAG-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=NONTOUCH-30hz-TRAFFICDIAG-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=TOUCH-30hz-TRAFFICDIAG-2.1"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=TOUCH-30hz-TRAFFICDIAG-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin



set "TEMPLATE=NONTOUCH-30hz-TRAFFIC-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=TOUCH-30hz-TRAFFIC-2.1"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=TOUCH-30hz-TRAFFIC-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin



set "TEMPLATE=NONTOUCH-30hz-EMS-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=TOUCH-30hz-EMS-2.1"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=TOUCH-30hz-EMS-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin



set "TEMPLATE=NONTOUCH-30hz-GPS-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin


set "TEMPLATE=TOUCH-30hz-GPS-2.1"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.binn


set "TEMPLATE=TOUCH-30hz-GPS-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
idf.py app
esptool --chip esp32s3 merge_bin -o build\RB-%VERSION%\RB-02-%VERSION%-%TEMPLATE%.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
del build\rb_project.bin