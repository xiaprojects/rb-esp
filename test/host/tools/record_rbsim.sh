#!/bin/sh
# Record the RB Simulator's synthetic flight as the fixtures in fixtures/rbsim/,
# one file per device profile, exactly as its serial outputs write them.
#
#   tools/record_rbsim.sh [RBSIM_DIR]     default: rbsimulator next to rb-esp
#
# The simulator is github.com/xiaprojects/rbsimulator, wrapper-cs. Its
# synthetic source needs no flight simulator and no hardware, only the .NET
# SDK (put it on PATH, or set DOTNET_ROOT). The wrapper paces its output in
# real time, so this takes RBSIM_SECONDS (default 70) of wall-clock time. Its
# web service binds 127.0.0.1:RBSIM_PORT (default 18091) and refuses control
# writes while recording.
#
# The synthetic flight starts at the current UTC time, so every recording has
# different timestamps. The tests read their expectations from the fixture
# text and never hard-code a time.
set -eu

here=$(cd "$(dirname "$0")/.." && pwd)
sim=${1:-$here/../../../rbsimulator}
sim=$(cd "$sim" && pwd)
project=$sim/wrapper-cs/src/RbSim.Msfs
out=$here/fixtures/rbsim
seconds=${RBSIM_SECONDS:-70}
rb09_fixes=${RBSIM_RB09_FIXES:-100} # $GPRMC kept from rb09: 20 s at 5 Hz

if [ ! -f "$project/RbSim.Msfs.csproj" ]; then
    echo "record_rbsim.sh: no rbsimulator wrapper at $project" >&2
    exit 1
fi

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

dotnet build "$project" -c Release -o "$tmp/bin" -nologo -v quiet >"$tmp/build.log" 2>&1 || {
    cat "$tmp/build.log" >&2
    exit 1
}
rev=$(git -C "$sim" describe --always --dirty 2>/dev/null || echo unknown)

status=0
timeout -s INT "$seconds" dotnet "$tmp/bin/rbsim-msfs.dll" \
    --source synthetic --listen 127.0.0.1 --port "${RBSIM_PORT:-18091}" --no-control \
    --serial "file:$tmp/rb04.nmea=rb04" \
    --serial "file:$tmp/rb05.nmea=rb05" \
    --serial "file:$tmp/rb09.nmea=rb09" \
    >"$tmp/wrapper.log" 2>&1 || status=$?
if [ "$status" -ne 124 ]; then
    cat "$tmp/wrapper.log" >&2
    echo "record_rbsim.sh: the wrapper exited with $status before the timeout" >&2
    exit 1
fi

# The fixtures are CRLF like the rest of fixtures/, and like the simulator.
header() {
    printf '# RB Simulator (rbsimulator %s), wrapper-cs synthetic source, device\r\n' "$rev"
    printf '# profile %s: %s\r\n' "$1" "$2"
    printf '# Recorded %s by test/host/tools/record_rbsim.sh. Do not edit: re-record.\r\n' "$(date -u +%Y-%m-%d)"
}

mkdir -p "$out"
{
    header rb04 'what RB-01 sends an RB-04 EMS display.'
    cat "$tmp/rb04.nmea"
} >"$out/rb04.nmea"
{
    header rb05 'what RB-01 sends an RB-05 traffic display.'
    cat "$tmp/rb05.nmea"
} >"$out/rb05.nmea"
{
    header rb09 "what an RB-02 sends its RB-09 remote display; the first $rb09_fixes fixes."
    awk -v max="$rb09_fixes" '/^\$GPRMC/ { if (++n > max) exit } { print }' "$tmp/rb09.nmea"
} >"$out/rb09.nmea"

wc -l "$out"/*.nmea
