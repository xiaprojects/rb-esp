#!/usr/bin/env bash
# BuildMachine-2.0.sh - bash port of BuildMachine-2.0.bat
#
# Builds every distribution firmware variant for each LCD pixel clock and
# merges bootloader, partition table and app into one flashable image:
#   build/RB-<version>/pixelclock-<MHz>/<prefix><version>-<template>.bin
#
# Usage: ./BuildMachine-2.0.sh [PATTERN...]
#   PATTERN       optional shell globs; only variants whose <prefix><template>
#                 matches one of them are built, e.g. "RB-07-*" or "*EMS*"
# Environment:
#   PIXEL_CLOCKS  pixel clocks in MHz to build for (default "18 19 20")
#   ESPTOOL       esptool command (default: esptool, else esptool.py)
#
# Run it from a shell where ESP-IDF is activated (export.sh, or EIM's
# activate_idf_*.sh); with only IDF_PATH set, $IDF_PATH/export.sh is sourced
# here. Images that already exist are skipped;
# delete them to force a rebuild. main/RB/BuildMachine.h is restored on exit.

cd "$(dirname "$0")" || exit 1

if ! command -v idf.py >/dev/null 2>&1; then
  if [ -x "${IDF_PYTHON_ENV_PATH:-}/bin/python" ] && [ -f "${IDF_PATH:-}/tools/idf.py" ]; then
    # EIM's activate_idf_*.sh defines idf.py as a shell function, which
    # scripts do not inherit
    idf.py() { "$IDF_PYTHON_ENV_PATH/bin/python" "$IDF_PATH/tools/idf.py" "$@"; }
  elif [ -f "${IDF_PATH:-}/export.sh" ]; then
    . "$IDF_PATH/export.sh" >/dev/null || exit 1
  fi
fi
if ! command -v idf.py >/dev/null 2>&1; then
  echo "[ERROR] idf.py not found; activate ESP-IDF (export.sh or activate_idf_*.sh) first" >&2
  exit 1
fi

set -uo pipefail

if [ -z "${ESPTOOL:-}" ]; then
  if command -v esptool >/dev/null 2>&1; then ESPTOOL=esptool; else ESPTOOL=esptool.py; fi
fi
PIXEL_CLOCKS=${PIXEL_CLOCKS:-18 19 20}
PATTERNS=("$@")
FAILED=()

# Get version from git tags; fall back to date if git not available
VERSION=$(git describe --tags 2>/dev/null)
[ -n "$VERSION" ] || VERSION=$(date +%Y-%m-%d)

# Every build overwrites the header; put the developer's copy back afterwards.
# Plain cp (not -p) so the restored file is newer than the last build's objects.
HEADER=main/RB/BuildMachine.h
HEADER_BACKUP=$(mktemp) || exit 1
cp "$HEADER" "$HEADER_BACKUP" || exit 1
trap 'cp "$HEADER_BACKUP" "$HEADER"; rm -f "$HEADER_BACKUP"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

selected() {
  local p
  [ ${#PATTERNS[@]} -eq 0 ] && return 0
  for p in "${PATTERNS[@]}"; do
    [[ $1 == $p ]] && return 0
  done
  return 1
}

# Parameter order: TEMPLATE, SOURCE_PREFIX, OUT_PREFIX, IDF_MODE, PIXEL_CLOCK
build() {
  local TEMPLATE=$1 SOURCE_PREFIX=$2 OUT_PREFIX=$3 IDF_MODE=$4 PIXEL_CLOCK=${5:-}
  local OUTDIR="build/RB-$VERSION" OUTFILE SOURCE

  selected "$OUT_PREFIX$TEMPLATE" || return 0

  # Compute expected output path early; skip build if it already exists
  [ -n "$PIXEL_CLOCK" ] && OUTDIR="$OUTDIR/pixelclock-$PIXEL_CLOCK"
  OUTFILE="$OUTDIR/$OUT_PREFIX$VERSION-$TEMPLATE.bin"
  if [ -e "$OUTFILE" ]; then
    echo "[INFO] Skipping $TEMPLATE build; output already exists: \"$OUTFILE\""
    return 0
  fi

  SOURCE="$SOURCE_PREFIX$TEMPLATE.h"
  if [ ! -f "$SOURCE" ]; then
    echo "[WARN] Template not found: \"$SOURCE\""
    return 0
  fi

  cp "$SOURCE" "$HEADER"
  {
    echo
    echo "#define RB_VERSION \"$VERSION\""
    if [ -n "$PIXEL_CLOCK" ]; then
      echo "#define EXAMPLE_LCD_PIXEL_CLOCK_MHZ $PIXEL_CLOCK"
    fi
  } >> "$HEADER"

  if [ -n "$PIXEL_CLOCK" ]; then
    echo "[INFO] Running idf.py $IDF_MODE for $TEMPLATE (PIXEL_CLOCK=$PIXEL_CLOCK)"
  else
    echo "[INFO] Running idf.py $IDF_MODE for $TEMPLATE"
  fi
  if ! idf.py "$IDF_MODE"; then
    echo "[ERROR] idf.py failed for $TEMPLATE"
    FAILED+=("$OUTFILE")
    return 1
  fi

  mkdir -p "$OUTDIR"
  "$ESPTOOL" --chip esp32s3 merge_bin -o "$OUTFILE" --flash_mode dio --flash_freq 80m --flash_size 16MB \
    0x0000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/rb_project.bin
  local MERGE_STATUS=$?
  rm -f build/rb_project.bin

  # Verify the output file was created and is non-zero; drop partial output
  # so the next run does not skip it
  if [ $MERGE_STATUS -ne 0 ] || [ ! -s "$OUTFILE" ]; then
    echo "[ERROR] Output file not created or empty: \"$OUTFILE\""
    rm -f "$OUTFILE"
    FAILED+=("$OUTFILE")
    return 1
  fi
}

# idf.py app only rebuilds the application; the merge also needs these
if [ ! -f build/bootloader/bootloader.bin ] || [ ! -f build/partition_table/partition-table.bin ]; then
  echo "[INFO] Building bootloader and partition table"
  idf.py bootloader partition-table || exit 1
fi

T=main/RB/BuildMachine-Template-
for P in $PIXEL_CLOCKS; do
  echo "[INFO] ===== Building for pixel clock $P ====="
  build DIAG-TOUCH-DEBUG-2.8 "$T" "" app "$P"
  build DIAG-TOUCH-DEBUG-2.1 "$T" "" app "$P"

  build TOUCH-30hz-BT-2.8 "${T}RB-07-" "RB-07-" app "$P"
  build TOUCH-30hz-BT-2.1 "${T}RB-07-" "RB-07-" app "$P"

  build TOUCH-30hz-USB-2.8 "${T}RB-07-" "RB-07-" app "$P"
  build TOUCH-30hz-USB-2.1 "${T}RB-07-" "RB-07-" app "$P"

  build TOUCH-30hz-TRAFFIC-WIRED-USB-2.1 "${T}RB-05-" "RB-05-" app "$P"
  build TOUCH-30hz-TRAFFIC-WIRED-USB-2.8 "${T}RB-05-" "RB-05-" app "$P"

  build TOUCH-30hz-LIGHT-2.1 "${T}RB-02-" "RB-02-" app "$P"
  build TOUCH-30hz-LIGHT-2.8 "${T}RB-02-" "RB-02-" app "$P"

  build NONTOUCH-30hz-GPS-2.8-AAT "${T}RB-02-" "RB-02-" app "$P"
  build NONTOUCH-30hz-GPS-2.8-ALD "${T}RB-02-" "RB-02-" app "$P"
  build NONTOUCH-30hz-GPS-2.8-GMT "${T}RB-02-" "RB-02-" app "$P"
  build NONTOUCH-30hz-GPS-2.8-MAP "${T}RB-02-" "RB-02-" app "$P"
  build NONTOUCH-30hz-GPS-2.8-SPD "${T}RB-02-" "RB-02-" app "$P"
  build NONTOUCH-30hz-GPS-2.8-TRK "${T}RB-02-" "RB-02-" app "$P"
  build NONTOUCH-30hz-GPS-2.8-TRN "${T}RB-02-" "RB-02-" app "$P"

  build NONTOUCH-30hz-GPSDIAG-2.8 "${T}RB-02-" "RB-02-" app "$P"
  build TOUCH-30hz-GPSDIAG-2.1 "${T}RB-02-" "RB-02-" app "$P"
  build TOUCH-30hz-GPSDIAG-2.8 "${T}RB-02-" "RB-02-" app "$P"

  build NONTOUCH-30hz-TRAFFICDIAG-2.8 "${T}RB-02-" "RB-05-" app "$P"
  build TOUCH-30hz-TRAFFICDIAG-2.1 "${T}RB-02-" "RB-05-" app "$P"
  build TOUCH-30hz-TRAFFICDIAG-2.8 "${T}RB-02-" "RB-05-" app "$P"

  build NONTOUCH-30hz-TRAFFIC-2.8 "${T}RB-02-" "RB-05-" app "$P"
  build TOUCH-30hz-TRAFFIC-2.1 "${T}RB-02-" "RB-05-" app "$P"
  build TOUCH-30hz-TRAFFIC-2.8 "${T}RB-02-" "RB-05-" app "$P"

  build NONTOUCH-30hz-EMS-USB-2.8 "${T}RB-04-" "RB-04-" app "$P"
  build TOUCH-30hz-EMS-USB-2.1 "${T}RB-04-" "RB-04-" app "$P"
  build TOUCH-30hz-EMS-USB-2.8 "${T}RB-04-" "RB-04-" app "$P"

  build NONTOUCH-30hz-GPS-2.8 "${T}RB-02-" "RB-02-" app "$P"
  build TOUCH-30hz-GPS-2.1 "${T}RB-02-" "RB-02-" app "$P"
  build TOUCH-30hz-GPS-2.8 "${T}RB-02-" "RB-02-" app "$P"

  build TOUCH-30hz-USB-2.1 "${T}RB-02-" "RB-02-" app "$P"
  build TOUCH-30hz-USB-2.8 "${T}RB-02-" "RB-02-" app "$P"
done

if [ ${#FAILED[@]} -gt 0 ]; then
  echo "[ERROR] ${#FAILED[@]} build(s) failed:"
  printf '  %s\n' "${FAILED[@]}"
  exit 1
fi
echo "All builds completed."
exit 0
