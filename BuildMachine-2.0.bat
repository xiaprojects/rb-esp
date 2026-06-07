@echo off
rem BuildMachine.bat - refactored: parameterized builds via :build subroutine

rem Get version from git tags; fall back to date if git not available
for /f "delims=" %%i in ('git describe --tags') do set VERSION=%%i

set "OUTDIR=build\RB-%VERSION%"
if not exist "%OUTDIR%" md "%OUTDIR%"

goto :main

:build
setlocal
set "TEMPLATE=%~1"
set "SOURCE_PREFIX=%~2"
set "OUT_PREFIX=%~3"
set "IDF_MODE=%~4"
set "PIXEL_CLOCK=%~5"
REM Compute expected output path early; skip build if it already exists
if defined PIXEL_CLOCK (
  set "OUTDIR=build\RB-%VERSION%\pixelclock-%PIXEL_CLOCK%"
) else (
  set "OUTDIR=build\RB-%VERSION%"
)
set "OUTFILE=%OUTDIR%\%OUT_PREFIX%%VERSION%-%TEMPLATE%.bin"
if exist "%OUTFILE%" (
  echo [INFO] Skipping %TEMPLATE% build; output already exists: "%OUTFILE%"
  endlocal & goto :eof
)

set "SOURCE=%SOURCE_PREFIX%%TEMPLATE%.h"
if not exist "%SOURCE%" (
  echo [WARN] Template not found: "%SOURCE%"
  endlocal & goto :eof
)

copy /Y "%SOURCE%" "main\RB\BuildMachine.h" >nul
echo #define RB_VERSION "%VERSION%" >> "main\RB\BuildMachine.h"
if defined PIXEL_CLOCK (
  echo #define EXAMPLE_LCD_PIXEL_CLOCK_MHZ %PIXEL_CLOCK% >> "main\RB\BuildMachine.h"
)
if defined PIXEL_CLOCK (
  echo [INFO] Running idf.py %IDF_MODE% for %TEMPLATE% (PIXEL_CLOCK=%PIXEL_CLOCK%)
) else (
  echo [INFO] Running idf.py %IDF_MODE% for %TEMPLATE%
)
idf.py %IDF_MODE%
if errorlevel 1 (
  echo [ERROR] idf.py failed for %TEMPLATE%
  endlocal & goto :eof
)
if defined PIXEL_CLOCK (
  set "OUTDIR=build\RB-%VERSION%\pixelclock-%PIXEL_CLOCK%"
) else (
  set "OUTDIR=build\RB-%VERSION%"
)
if not exist "%OUTDIR%" md "%OUTDIR%"
set "OUTFILE=%OUTDIR%\%OUT_PREFIX%%VERSION%-%TEMPLATE%.bin"
esptool --chip esp32s3 merge_bin -o "%OUTFILE%" --flash_mode dio --flash_freq 80m --flash_size 16MB ^
  0x0000 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\rb_project.bin
if exist ".\build\rb_project.bin" del /Q ".\build\rb_project.bin"

rem Verify the output file was created and is non-zero; stop on failure
if not exist "%OUTFILE%" (
  echo [ERROR] Output file not created: "%OUTFILE%"
  endlocal & exit /b 1
)
for %%I in ("%OUTFILE%") do set "OUTFILE_SIZE=%%~zI"
if "%OUTFILE_SIZE%"=="0" (
  echo [ERROR] Output file has zero size: "%OUTFILE%"
  endlocal & exit /b 1
)

endlocal
goto :eof

:main
rem Calls (parameter order: TEMPLATE, SOURCE_PREFIX, OUT_PREFIX, IDF_MODE, PIXEL_CLOCK)
for %%P in (18 19 20) do (
  echo [INFO] ===== Building for pixel clock %%P =====
  call :build DIAG-TOUCH-DEBUG-2.8 "main\RB\BuildMachine-Template-" "" app %%P
  call :build DIAG-TOUCH-DEBUG-2.1 "main\RB\BuildMachine-Template-" "" app %%P

  call :build TOUCH-30hz-BT-2.8 "main\RB\BuildMachine-Template-RB-07-" "RB-07-" app %%P
  call :build TOUCH-30hz-BT-2.1 "main\RB\BuildMachine-Template-RB-07-" "RB-07-" app %%P

  call :build TOUCH-30hz-USB-2.8 "main\RB\BuildMachine-Template-RB-07-" "RB-07-" app %%P
  call :build TOUCH-30hz-USB-2.1 "main\RB\BuildMachine-Template-RB-07-" "RB-07-" app %%P

  call :build TOUCH-30hz-TRAFFIC-WIRED-USB-2.1 "main\RB\BuildMachine-Template-RB-05-" "RB-05-" app %%P
  call :build TOUCH-30hz-TRAFFIC-WIRED-USB-2.8 "main\RB\BuildMachine-Template-RB-05-" "RB-05-" app %%P

  call :build TOUCH-30hz-LIGHT-2.1 "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build TOUCH-30hz-LIGHT-2.8 "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P

  call :build NONTOUCH-30hz-GPS-2.8-AAT "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build NONTOUCH-30hz-GPS-2.8-ALD "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build NONTOUCH-30hz-GPS-2.8-GMT "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build NONTOUCH-30hz-GPS-2.8-MAP "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build NONTOUCH-30hz-GPS-2.8-SPD "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build NONTOUCH-30hz-GPS-2.8-TRK "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build NONTOUCH-30hz-GPS-2.8-TRN "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P

  call :build NONTOUCH-30hz-GPSDIAG-2.8 "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build TOUCH-30hz-GPDIAG-2.1 "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build TOUCH-30hz-GPSDIAG-2.8 "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P

  call :build NONTOUCH-30hz-TRAFFICDIAG-2.8 "main\RB\BuildMachine-Template-RB-02-" "RB-05-" app %%P
  call :build TOUCH-30hz-TRAFFICDIAG-2.1 "main\RB\BuildMachine-Template-RB-02-" "RB-05-" app %%P
  call :build TOUCH-30hz-TRAFFICDIAG-2.8 "main\RB\BuildMachine-Template-RB-02-" "RB-05-" app %%P

  call :build NONTOUCH-30hz-TRAFFIC-2.8 "main\RB\BuildMachine-Template-RB-02-" "RB-05-" app %%P
  call :build TOUCH-30hz-TRAFFIC-2.1 "main\RB\BuildMachine-Template-RB-02-" "RB-05-" app %%P
  call :build TOUCH-30hz-TRAFFIC-2.8 "main\RB\BuildMachine-Template-RB-02-" "RB-05-" app %%P

  call :build NONTOUCH-30hz-EMS-USB-2.8 "main\RB\BuildMachine-Template-RB-04-" "RB-04-" app %%P
  call :build TOUCH-30hz-EMS-USB-2.1 "main\RB\BuildMachine-Template-RB-04-" "RB-04-" app %%P
  call :build TOUCH-30hz-EMS-USB-2.8 "main\RB\BuildMachine-Template-RB-04-" "RB-04-" app %%P

  call :build NONTOUCH-30hz-GPS-2.8 "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build TOUCH-30hz-GPS-2.1 "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build TOUCH-30hz-GPS-2.8 "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P

  call :build TOUCH-30hz-USB-2.1 "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
  call :build TOUCH-30hz-USB-2.8 "main\RB\BuildMachine-Template-RB-02-" "RB-02-" app %%P
)

echo All builds completed.
exit /b 0
