# Host-side unit tests for RB-Avionics firmware logic.
#
# Builds selected firmware sources from ../../main with the native compiler,
# replacing ESP-IDF / FreeRTOS / LVGL with the fakes in stubs/, and runs them
# under Unity via CTest. No ESP-IDF installation is required.
#
#   cmake -S test/host -B build/host-tests
#   cmake --build build/host-tests
#   ctest --test-dir build/host-tests --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(rb_host_tests C)

enable_testing()

# Optional AddressSanitizer + UndefinedBehaviorSanitizer build. Several of the
# defects pinned by test_known_issue_* are only visible this way:
#   cmake -S test/host -B build/host-asan -DRB_SANITIZE=ON
option(RB_SANITIZE "Build the host tests with ASan + UBSan" OFF)
if(RB_SANITIZE)
  add_compile_options(-g -fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

set(RB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(RB_MAIN "${RB_ROOT}/main")

if(NOT EXISTS "${RB_MAIN}/RB/BuildMachine.h")
  message(FATAL_ERROR
    "main/RB/BuildMachine.h is missing. Copy one of main/RB/BuildMachine-Template-*.h "
    "to that path (this is what the BuildMachine*.bat scripts do).")
endif()

# ---- which feature flags does the current BuildMachine.h set? ----------------
# RB02_Defines.h includes BuildMachine.h by quoted path, so a quoted include
# always resolves next to RB02_Defines.h and the host build cannot shadow it.
# The suites therefore inherit whatever feature set that generated file happens
# to carry. Parse it so a suite that cannot work is skipped with a clear
# message instead of failing to link.
file(READ "${RB_MAIN}/RB/BuildMachine.h" RB_BM_RAW)
string(REPLACE ";" "," RB_BM_RAW "${RB_BM_RAW}")
string(REPLACE "\n" ";" RB_BM_LINES "${RB_BM_RAW}")
set(RB_BM_DEFINES "")
foreach(rb_line IN LISTS RB_BM_LINES)
  if(rb_line MATCHES "^[ \t]*#define[ \t]+([A-Za-z_0-9]+)")
    list(APPEND RB_BM_DEFINES "${CMAKE_MATCH_1}")
  endif()
endforeach()

set(RB_HAS_TRAFFIC FALSE)
if("RB02_ESP_BLUETOOTH" IN_LIST RB_BM_DEFINES OR "RB_ENABLE_USB_FLARM" IN_LIST RB_BM_DEFINES)
  set(RB_HAS_TRAFFIC TRUE)
endif()
set(RB_HAS_TRK FALSE)
if("RB_ENABLE_TRK" IN_LIST RB_BM_DEFINES)
  set(RB_HAS_TRK TRUE)
endif()
message(STATUS "BuildMachine.h: traffic=${RB_HAS_TRAFFIC} RB_ENABLE_TRK=${RB_HAS_TRK}")

# ---- Unity test framework (vendored, MIT) ------------------------------------
add_library(unity STATIC unity/unity.c)
target_include_directories(unity PUBLIC unity)
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_DOUBLE UNITY_OUTPUT_COLOR)

# ---- Host fakes for ESP-IDF / FreeRTOS / LVGL / drivers -----------------------
add_library(host_stubs STATIC stubs/esp_stubs.c stubs/lvgl_stubs.c)
target_include_directories(host_stubs PUBLIC stubs stubs/include "${RB_MAIN}/sc16is750")
target_compile_options(host_stubs PRIVATE -std=gnu17 -Wall -Wextra)

# ---- Flags shared by every firmware source compiled for the host --------------
# RB_HOST_TEST lifts the compile guard in main/RB/RB02.h. The stub include dir
# must come first so that "lvgl.h", "esp_log.h", ... resolve to the fakes.
add_library(rb_host_flags INTERFACE)
target_compile_definitions(rb_host_flags INTERFACE RB_HOST_TEST=1)
target_compile_options(rb_host_flags INTERFACE -std=gnu17 -Wall)
target_include_directories(rb_host_flags INTERFACE
  stubs/include
  "${RB_MAIN}/RB"
  "${RB_MAIN}"
  "${RB_MAIN}/sc16is750"
  "${RB_MAIN}/MS4525DO")

# rb_add_test(<name> <firmware sources...>)
#   Builds test/host/<name>.c together with the given firmware sources into
#   one executable and registers it with CTest.
function(rb_add_test name)
  add_executable(${name} ${name}.c ${ARGN})
  target_link_libraries(${name} PRIVATE unity host_stubs rb_host_flags m)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

# ---- Test suites --------------------------------------------------------------
rb_add_test(test_madgwick   "${RB_MAIN}/RB/madgwick.c")
rb_add_test(test_guihelpers "${RB_MAIN}/RB/RB02_GUIHelpers.c")
rb_add_test(test_ems_parser "${RB_MAIN}/RB/RB04_NMEA_Parser.c")
target_compile_definitions(test_ems_parser PRIVATE RB_ENABLE_EMS=1)

rb_add_test(test_ec11 "${RB_MAIN}/RB/RB02_NavCore.c")
target_compile_definitions(test_ec11 PRIVATE RB_ENABLE_NavCore=1)

# Traffic integration: FLARM / ADS-B sentences -> parser -> store -> screens.
# Driven by the fixtures in fixtures/ via the transport simulator in sim/.
#
# Needs a traffic-capable BuildMachine.h: RB02_ESP_BLUETOOTH or
# RB_ENABLE_USB_FLARM is what defines RB05_Flarm_SERIAL, and without it the
# whole traffic module compiles to nothing. RB_ENABLE_TRK must be off, because
# it makes RB05_Radar.c reach for singletonConfig()->ui.Gyro, pulling the NVS
# stack into a screen test.
if(NOT RB_HAS_TRAFFIC)
  message(STATUS
    "Skipping test_traffic: main/RB/BuildMachine.h defines neither "
    "RB02_ESP_BLUETOOTH nor RB_ENABLE_USB_FLARM, so the traffic module is "
    "compiled out. Use a TRAFFIC template (see .claude/skills/firmware-build).")
elseif(RB_HAS_TRK)
  message(STATUS
    "Skipping test_traffic: main/RB/BuildMachine.h defines RB_ENABLE_TRK, which "
    "makes RB05_Radar.c depend on singletonConfig(). Use a TRAFFIC template.")
else()
  rb_add_test(test_traffic
    sim/traffic_replay.c
    "${RB_MAIN}/RB/RB05_FlarmParser.c"
    "${RB_MAIN}/RB/RB05_Traffic.c"
    "${RB_MAIN}/RB/RB05_Radar.c"
    "${RB_MAIN}/RB/RB02_GUIHelpers.c")
  target_include_directories(test_traffic PRIVATE sim "${RB_MAIN}/I2C_Driver" "${RB_MAIN}/QMI8658")
  # Absolute fixture paths, so the binary also runs standalone from anywhere.
  target_compile_definitions(test_traffic PRIVATE RB_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()
