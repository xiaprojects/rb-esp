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
rb_add_test(test_ems_parser "${RB_MAIN}/RB/RB04_NMEA_Parser.c" sim/fixture_lines.c)
target_compile_definitions(test_ems_parser PRIVATE RB_ENABLE_EMS=1 RB_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(test_ems_parser PRIVATE sim)

rb_add_test(test_ec11 "${RB_MAIN}/RB/RB02_NavCore.c")
target_compile_definitions(test_ec11 PRIVATE RB_ENABLE_NavCore=1)

# RB-09 remote display: $RBATT / $RBAIR -> RB09_SlaveData -> the instrument
# variables. RB_ENABLE_SLAVE_DEVICE is forced, so it builds on every BuildMachine.h.
rb_add_test(test_slave "${RB_MAIN}/RB/RB09_Slave.c" "${RB_MAIN}/RB/RB02_GUIHelpers.c" sim/fixture_lines.c)
target_compile_definitions(test_slave PRIVATE RB_ENABLE_SLAVE_DEVICE=1 RB_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(test_slave PRIVATE sim "${RB_MAIN}/I2C_Driver" "${RB_MAIN}/QMI8658")

# GPS: the $GPRMC / $GPGGA scanners and NMEA_ParseBuffer() live in RB02.c, the
# application core, which cannot be compiled on the host without faking most
# of LVGL. Moving them to their own file would rewrite blame on code under
# active work, so cut that section out of RB02.c at configure time and compile
# only the slice. Editing RB02.c re-runs this; the #line directive keeps
# compiler and sanitizer reports pointing at RB02.c itself.
get_filename_component(RB_RB02_C "${RB_MAIN}/RB/RB02.c" REALPATH)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${RB_RB02_C}")
file(READ "${RB_RB02_C}" rb02_src)

# From the `#ifdef RB_ENABLE_GPS` that opens nmea_GGA_UpdatedValueFor() ...
string(FIND "${rb02_src}" "void nmea_GGA_UpdatedValueFor(" rb02_fn)
if(rb02_fn EQUAL -1)
  message(FATAL_ERROR "test_gps: nmea_GGA_UpdatedValueFor() not found in RB02.c; update the slice markers")
endif()
string(SUBSTRING "${rb02_src}" 0 ${rb02_fn} rb02_head)
string(FIND "${rb02_head}" "#ifdef RB_ENABLE_GPS" rb02_begin REVERSE)
string(SUBSTRING "${rb02_head}" ${rb02_begin} -1 rb02_gap)
if(rb02_begin EQUAL -1 OR NOT rb02_gap MATCHES "^#ifdef RB_ENABLE_GPS[ \t\r\n]*$")
  message(FATAL_ERROR "test_gps: expected `#ifdef RB_ENABLE_GPS` right before nmea_GGA_UpdatedValueFor() in RB02.c")
endif()

# ... to the `#endif` that closes the NMEA_ParseBuffer() definition.
string(SUBSTRING "${rb02_src}" ${rb02_fn} -1 rb02_tail)
string(REGEX MATCH "void NMEA_ParseBuffer\\([^)]*\\)[ \t\r]*\n{" rb02_def "${rb02_tail}")
string(FIND "${rb02_tail}" "${rb02_def}" rb02_def_at)
if(NOT rb02_def OR rb02_def_at EQUAL -1)
  message(FATAL_ERROR "test_gps: NMEA_ParseBuffer() definition not found in RB02.c")
endif()
string(SUBSTRING "${rb02_tail}" ${rb02_def_at} -1 rb02_body)
string(FIND "${rb02_body}" "\n}" rb02_close)
string(SUBSTRING "${rb02_body}" ${rb02_close} -1 rb02_after)
if(rb02_close EQUAL -1 OR NOT rb02_after MATCHES "^\n}[ \t\r]*\n#endif")
  message(FATAL_ERROR "test_gps: expected `#endif` right after the NMEA_ParseBuffer() body in RB02.c")
endif()
string(FIND "${rb02_after}" "#endif" rb02_endif)
math(EXPR rb02_len "${rb02_fn} + ${rb02_def_at} + ${rb02_close} + ${rb02_endif} + 6 - ${rb02_begin}")
string(SUBSTRING "${rb02_src}" ${rb02_begin} ${rb02_len} rb02_slice)

string(SUBSTRING "${rb02_src}" 0 ${rb02_begin} rb02_before)
string(REGEX MATCHALL "\n" rb02_nl "${rb02_before}")
list(LENGTH rb02_nl rb02_line)
math(EXPR rb02_line "${rb02_line} + 1")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/gen/rb02_gps_slice.inc.tmp"
  "#line ${rb02_line} \"${RB_RB02_C}\"\n${rb02_slice}\n")
configure_file("${CMAKE_CURRENT_BINARY_DIR}/gen/rb02_gps_slice.inc.tmp"
               "${CMAKE_CURRENT_BINARY_DIR}/gen/rb02_gps_slice.inc" COPYONLY)
message(STATUS "test_gps: RB02.c GPS slice starts at line ${rb02_line}")

rb_add_test(test_gps "${RB_MAIN}/RB/RB02_GUIHelpers.c" sim/fixture_lines.c)
target_compile_definitions(test_gps PRIVATE RB_ENABLE_GPS=1 RB_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(test_gps PRIVATE sim "${CMAKE_CURRENT_BINARY_DIR}/gen")

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
    sim/fixture_lines.c
    "${RB_MAIN}/RB/RB05_FlarmParser.c"
    "${RB_MAIN}/RB/RB05_Traffic.c"
    "${RB_MAIN}/RB/RB05_Radar.c"
    "${RB_MAIN}/RB/RB02_GUIHelpers.c")
  target_include_directories(test_traffic PRIVATE sim "${RB_MAIN}/I2C_Driver" "${RB_MAIN}/QMI8658")
  # Absolute fixture paths, so the binary also runs standalone from anywhere.
  target_compile_definitions(test_traffic PRIVATE RB_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()
