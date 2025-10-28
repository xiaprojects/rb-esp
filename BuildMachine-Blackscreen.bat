@echo off
set "PATH=%PATH%;C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts"
set "PATH=%PATH%;C:\Users\Stefano\.espressif\tools\ninja\1.12.1"
set "PATH=%PATH%;C:\Users\Stefano\.espressif\tools\ccache\4.10.2\ccache-4.10.2-windows-x86_64"
set "PATH=%PATH%;C:\Users\Stefano\.espressif\tools\cmake\3.30.2\bin"
set "PATH=%PATH%;C:\Users\Stefano\.espressif\tools\idf-python\3.11.2\Lib\venv\scripts\nt"
set "PATH=%PATH%;C:\Users\Stefano\.espressif\tools\idf-exe\1.0.3"
set "PATH=%PATH%;C:\Users\Stefano\.espressif\tools\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin"
set "IDF_PYTHON_ENV_PATH=C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env"
set "IDF_PATH=C:\Users\Stefano\esp\v5.5\esp-idf"

for /f "delims=" %%i in ('git describe --tags') do set VERSION=%%i
md build\RB-%VERSION%-WorkaroundBlackScreen

cmake -G Ninja -DPYTHON_DEPS_CHECKED=1 -DPYTHON=C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe -DESP_PLATFORM=1 -DIDF_TARGET=esp32s3 -DCCACHE_ENABLE=0 -B c:\Users\Stefano\Desktop\RB-02\build -S c:\Users\Stefano\Desktop\RB-02 -DSDKCONFIG='c:\Users\Stefano\Desktop\RB-02\sdkconfig' 

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-AAT"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-ALD"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-GMT"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-MAP"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-SPD"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-TRK"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin

set "TEMPLATE=NONTOUCH-30hz-GPS-2.8-TRN"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin




set "TEMPLATE=NONTOUCH-30hz-GPSDIAG-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=TOUCH-30hz-GPDIAG-2.1"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=TOUCH-30hz-GPSDIAG-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=NONTOUCH-30hz-TRAFFICDIAG-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-05-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=TOUCH-30hz-TRAFFICDIAG-2.1"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-05-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=TOUCH-30hz-TRAFFICDIAG-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-05-%VERSION%-%TEMPLATE%.bin



set "TEMPLATE=NONTOUCH-30hz-TRAFFIC-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-05-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=TOUCH-30hz-TRAFFIC-2.1"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-05-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=TOUCH-30hz-TRAFFIC-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-05-%VERSION%-%TEMPLATE%.bin



set "TEMPLATE=NONTOUCH-30hz-EMS-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-04-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=TOUCH-30hz-EMS-2.1"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-04-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=TOUCH-30hz-EMS-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-04-%VERSION%-%TEMPLATE%.bin



set "TEMPLATE=NONTOUCH-30hz-GPS-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=TOUCH-30hz-GPS-2.1"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin


set "TEMPLATE=TOUCH-30hz-GPS-2.8"
copy /Y main\RB\BuildMachine-Template-RB-02-%TEMPLATE%.h main\RB\BuildMachine.h
echo #define RB_VERSION "%VERSION%" >> main\RB\BuildMachine.h
echo #define RB_02_28_WORKAROUND_BLACKSCREEN 1 >> main\RB\BuildMachine.h
C:\Users\Stefano\.espressif\python_env\idf5.5_py3.11_env\Scripts\python C:\Users\Stefano\esp\v5.5\esp-idf\tools\idf.py app
move build\rb_project.bin build\RB-%VERSION%-WorkaroundBlackScreen\RB-02-%VERSION%-%TEMPLATE%.bin
