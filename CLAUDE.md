# RB-Avionics firmware (rb-esp)

Open-source aviation instrument firmware for ESP32-S3 displays: attitude, altimeter,
variometer, GPS, engine monitoring and FLARM/ADS-B traffic. ESP-IDF (>= 4.4) + LVGL 8.4,
pure C. Public repo with a worldwide builder community; see `README.md` for the product
lines (RB-01..RB-09) and the wiki links.

---

## Read this before touching anything

**1. Everything under `main/` is CRLF.** Preserve it. A tool that rewrites a file with LF
produces a whole-file diff and destroys `git blame`. When scripting an edit, read and
write in binary and put `\r\n` in the replacement text:

```python
s = open(path, "rb").read()
s = s.replace(old.replace("\n", "\r\n").encode(), new.replace("\n", "\r\n").encode())
open(path, "wb").write(s)
```

Check afterwards with `file main/RB/<file>` — it must still say `CRLF line terminators`.

**2. The working tree carries the maintainer's uncommitted work.** Dozens of tracked
files are modified and whole directories that `main/CMakeLists.txt` references are
untracked. **Never `git add -A` or `git add .`** and never `git checkout --` a file you
did not modify. Stage only paths you created or deliberately changed, by name.

**3. `main/RB/RB02.h` ends with a deliberate `#error`**, guarded by `#ifndef RB_HOST_TEST`:

```c
#ifndef RB_HOST_TEST
 #error You shall not compile yourself, please use the official releases and join the discord community
#endif
```

This is upstream policy — casual cloners are meant to use the official binaries. The host
test harness defines `RB_HOST_TEST` to lift it. Do not remove the guard. The maintainer
sometimes comments the `#error` out locally to build; if you find it commented, leave it.

**4. `main/RB/BuildMachine.h` is tracked *and* generated, and is currently modified.**
Overwriting it is part of the build (see below), but restoring it with
`git checkout main/RB/BuildMachine.h` **destroys the maintainer's working config**. Always
back it up to a scratch file first and restore from that copy.

---

## Build (firmware)

There is no single build. `main/RB/BuildMachine.h` selects the product, display and feature
set, and 39 templates in `main/RB/BuildMachine-Template-*.h` are the supported combinations.
The Windows `BuildMachine*.bat` scripts loop over them; the Linux equivalent for one variant:

```sh
cp main/RB/BuildMachine.h /tmp/BuildMachine.h.mine            # 1. ALWAYS back up first
cp main/RB/BuildMachine-Template-RB-02-TOUCH-30hz-GPS-2.8.h main/RB/BuildMachine.h
printf '#define RB_VERSION "%s"\n' "$(git describe --tags)" >> main/RB/BuildMachine.h
idf.py app
cp /tmp/BuildMachine.h.mine main/RB/BuildMachine.h            # 2. restore, not via git
```

`RB_VERSION` is **required** — the templates do not define it, the build scripts append it,
and `RB02.c` uses it in string concatenation (`"Version " RB_VERSION`), so it must be a
string literal or compilation fails.

Template names read `BuildMachine-Template-RB-<line>-<TOUCH|NONTOUCH>-30hz-<features>-<display>.h`,
e.g. `RB-02-TOUCH-30hz-GPS-2.8`. Display is 1.7 / 1.8 / 2.1 / 2.8. Feature tokens include
`GPS`, `MAP`, `EMS`, `TRAFFIC`, `USB`, `BT`, `AAT`, `SPD`, `TRK`, `TRN`, `ALD`, `GMT`,
`LIGHT`, and `*DIAG` / `DEBUG` for diagnostic builds.

Pick a representative set when verifying a change, not just one — the `#ifdef` paths differ
a lot. A change touching GPS parsing needs a `GPS` template; one touching the radar needs a
`TRAFFIC` template. There is no ESP-IDF in this dev container, so **you cannot build the
firmware here** — say so rather than claiming a build passed.

## Build and run (host tests)

Native unit/integration tests for logic that does not need an ESP32. No ESP-IDF required.

```sh
cmake -S test/host -B build/host-tests
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

Sanitizers (some defects only show here):

```sh
cmake -S test/host -B build/host-asan -DRB_SANITIZE=ON
cmake --build build/host-asan
ASAN_OPTIONS=detect_leaks=0 ./build/host-asan/test_traffic
```

**`test/host/README.md` is the reference** for the stub layer, the fixtures, the transport
simulator, the dump1090 → `$PFLAA` converter, and how to add a suite. Read it before
extending the tests; do not re-derive it.

Current state: 7 suites, ~235 tests, 8 `TEST_IGNORE` markers (1 in `test_madgwick`, 4 GPS
defects in `test_gps`, 1 each in `test_ems_parser`, `test_slave` and `test_traffic` from the RB
Simulator replays). Keep it that way — a new failure or a new ignore should be deliberate and
mentioned.

**`test_gps` compiles a slice of `RB02.c`**, cut out at configure time from the
`#ifdef RB_ENABLE_GPS` before `nmea_GGA_UpdatedValueFor()` to the `#endif` after
`NMEA_ParseBuffer()`. Editing `RB02.c` re-runs the cut. If you restructure that region,
configure fails naming the marker it lost: update the markers in `test/host/CMakeLists.txt`.

**The suites inherit their feature flags from `main/RB/BuildMachine.h`.** `RB02_Defines.h`
includes it by quoted path, so the host build cannot shadow it, and whichever config happens
to be in the tree decides what compiles. The committed version has `RB_ENABLE_TRK` on and no
Bluetooth, which compiles the traffic module out entirely — so `test_traffic` is **skipped**
on it. `test/host/CMakeLists.txt` detects this and prints why, rather than failing to link:

```
-- BuildMachine.h: traffic=FALSE RB_ENABLE_TRK=TRUE
-- Skipping test_traffic: ... the traffic module is compiled out.
```

If you see 6 suites instead of 7, that is the reason. CI pins a known traffic-capable config
(safe there — the checkout is throwaway) and fails if any suite was skipped.

---

## Architecture

**One ESP-IDF component.** Everything is in `main/`, registered by `main/CMakeLists.txt`.
`main.c` brings up the drivers and calls `RB02_Main()`; `main/RB/RB02.c` (5600 lines) is the
application: screen construction, the LVGL tick dispatch, and the input parsers.

**Only 42 of the 152 `.c` files are compilation units.** The rest are `#include`d as source
text from another file — LVGL image data (`main/RB/images/*.c`), per-display LCD drivers
(`ST7701S_28.c` from `ST7701S.c`), touch controllers, and some logic
(`RB04_NMEA_Parser.c` from `RB04_EMS.c`, `RB_Usb.c` from `main.c`). Consequences:

- Adding a file to `main/CMakeLists.txt` that is already `#include`d gives duplicate symbols.
- `static` helpers in those files are not linkable from elsewhere (e.g.
  `parse_int_token()` in `RB04_NMEA_Parser.c`, the only correct numeric parser in the tree).
- A host test for such a file must compile it together with its includer.

**Feature flags are the architecture.** `main/RB/BuildMachine.h` sets a handful of flags;
`main/RB/RB02_Defines.h` derives dozens more from them (`#include "BuildMachine.h"` at the
top), and nearly every module is wrapped in `#ifdef`. Before concluding code is dead or
reachable, check which templates define the flag:

```sh
grep -l RB_ENABLE_TRK main/RB/BuildMachine-Template-*.h
```

A trap worth knowing: the screen enum in `main/RB/RB02_Navigator.h` is itself
`#ifdef`-conditional, so `RB02_TAB_*` **values shift between build variants**. Never persist
or hardcode a tab index.

**Global mutable state.** `singletonConfig()` (`main/RB/RB02_Config.c`) returns the one
`RB02_Status`, holding settings, `NMEA_DATA`, per-screen UI handles and the EMS snapshot.
Traffic lives in a separate global, `flarmStatus` in `main/RB/RB05_FlarmParser.c`. Logic and
rendering are interleaved — parsers call `lv_label_set_text()` directly — so extracting a
pure seam is usually the first step in making something testable.

**Dead directories, not in `main/CMakeLists.txt`:** `main/RB/removed/`, `main/2832/`,
`main/Wireless/`, `main/WirelessOLD/`, `main/LVGL_Driver/Fixed/`. Code there is stale and
sometimes carries copies of bugs already fixed in the live tree — do not use it as a
reference, and do not copy from it.

### Module map (`main/RB/`)

| File | Role |
|---|---|
| `RB02.c` | app core: screens, LVGL tick dispatch, GPS `$GPRMC`/`$GPGGA` parsers |
| `RB02_Navigator.c/.h` | tab model and navigation; the conditional `RB02_TAB_*` enum |
| `RB02_Config.c/.h` | `singletonConfig()`, `RB02_Status`, NVS-backed settings |
| `RB02_Defines.h` | derives every feature flag from `BuildMachine.h` |
| `RB02_GUIHelpers.c/.h` | shared pure helpers: `nmea_to_decimal`, `nmeaChecksum`, `RB02_SuggestedQNH` |
| `RB02_Setup.c` | settings screens |
| `RB02_AAttitude.c`, `RB02_Altimeter.c`, `RB02_Gyro.c` | instrument screens |
| `RB02_GPSMap.c`, `RB02_GPSDiag.c` | moving map, GPS diagnostics |
| `RB02_NavCore.c/.h` | RB02-NavCore board: SC16IS76x UART/GPIO expander, EC11 encoder state machine |
| `RB04_EMS.c`, `RB04_NMEA_Parser.c` | engine monitoring, `$RBEMS`/`$RBCYL` parser |
| `RB05_FlarmParser.c/.h` | `$PFLAA`/`$PFLAU` parser, traffic store, stream reassembly |
| `RB05_Traffic.c`, `RB05_Radar.c` | traffic list and radar screens |
| `RB09_Slave.c/.h` | RB-09 remote display: `$RBATT`/`$RBAIR` from USB in place of the local IMU and barometer; `RB09_Slave_FormatAir()`, the master's `$RBAIR`, is in every build |
| `madgwick.c/.h` | IMU attitude filter |
| `RB_License.c` | licence tier gating |

---

## Conventions and gotchas

**Hand-rolled NMEA field scanners.** Four near-identical scanners
(`nmea_RMC_mini_parser` / `nmea_GGA_mini_parser` in `RB02.c`,
`ParsePFLAU` / `ParsePFLAA` in `RB05_FlarmParser.c`) walk bytes and dispatch each field to
an `Update*ValueFor(csvCounter, finalNumber, decimalCounter)` callback, where the value is an
integer and `decimalCounter` says where the decimal point was. They share a shape:

- a `sign` flag applied when the field is delivered,
- `inChecksum` set at `'*'` so the checksum never reaches the accumulator,
- `sawEnd` set on `\0` / `\n` / `\r`, which gates delivery of the final field so a
  truncated stream read never yields a partial value,
- the return value spans the whole sentence, because `NMEA_ParseBuffer()` uses it for the
  `settingsEnableNMEAOutput` passthrough — do not make the scanner stop early at `'*'`.

**The `RB02.c` scanners deliberately feed non-digits to the accumulator.** The hemisphere
cases recover the `N`/`S`/`E`/`W` letter with `finalNumber + '0'`, which only works while a
lone letter round-trips through that branch. Adding digit validation there silently breaks
latitude and longitude signs. The FLARM scanners have no such dependency and do validate.

**Traffic units.** In `FlarmObstacle`: `x` = relative **east** metres, `y` = relative
**north** metres, `z` = relative vertical **metres** (the source comment saying feet is
wrong). Screens display `z / 100 * 3.28084`, i.e. a flight-level difference in hundreds of
feet — and that division is integer, so it quantizes to 100 m steps. All four are `int16_t`,
so large values wrap. Age comparisons must use `RB05_Traffic_Age()`
(`RB05_FlarmParser.h`), which survives the seconds-of-day rollover at midnight.

**LVGL conventions.** `lv_img_set_angle()` takes 0.1-degree units and rotates
**clockwise**; the aircraft icons (`images/TrafficAircraft*.c`) point **north (up)** at angle
0, so `track * 10` is correct on a north-up display. Screen `+y` is **down**, so a compass
bearing maps to `(sin b, -cos b)` — which is what `(east, -north)` in `RB05_Radar.c` and
`(cos(d-90), sin(d-90))` in `RB05_Radar_MoveRadar()` both compute. The radar face is
north-up; `COG = 0` with a `// TODO Rotate the whole Radar` is intentional, unlike the Gyro
screen which rotates its card.

**No absolute heading on most builds.** The QMI8658 is a 6-axis IMU, so `-AttitudeYaw` is
Madgwick yaw with an arbitrary origin at boot plus drift. `AttitudeYawCorrection` anchors it
to true GPS track, but it only exists `#ifdef RB_ENABLE_TRK` — and the traffic templates
define neither that nor `RB_ENABLE_GPS`. Code needing true heading must handle its absence.

**No stdlib parsing anywhere.** There is no `strtol`, `atoi`, `atof` or `sscanf` in `main/`.
That is deliberate for code size; match it rather than introducing them.

---

## Working practice

- **Prefer a host test over reasoning.** The suite catches real defects fast, and
  `test/host/sim/traffic_replay.c` can replay a recorded session through the real parser.
  Several bugs in this tree were only visible under `-DRB_SANITIZE=ON`.
- **Pin a defect you are not fixing** as a `test_known_issue_*` test that `TEST_IGNORE`s
  itself while the defect is present and becomes a live assertion once fixed. It never
  blocks CI but stays visible in every run.
- **Say what you did not verify.** You cannot build or flash firmware in this container.
  Host tests passing is not the same as the firmware compiling.
- **Issue-driven commits.** The history is `<Description> #<issue>` — e.g.
  `GPS Realiability #128`, `INOP screen on Radar in USB Mode #129` — with bare version
  bumps (`1.7.1`) for releases. Match that. Commit only when asked.
