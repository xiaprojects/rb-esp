#!/usr/bin/env python3
"""
Convert dump1090 ADS-B output into FLARM $PFLAA/$PFLAU sentences.

The firmware has no dump1090 parser (RB05_GDL90Parser.c is a stub and is not
even in main/CMakeLists.txt), but it does have a working FLARM parser. ADS-B
carries absolute positions while PFLAA carries positions relative to ownship,
so this script does the conversion that a firmware adapter would have to do:

    dump1090 SBS-1 (port 30003)  ->  merge per aircraft  ->  relative N/E/V
                                 ->  $PFLAA / $PFLAU

Two uses:

  1. Generate a test fixture from a capture:
       sbs_to_pflaa.py --in fixtures/dump1090_sbs.txt \
                       --out fixtures/dump1090_traffic.nmea

  2. Replay a live dump1090 into a device or a pty, in real time:
       sbs_to_pflaa.py --host 127.0.0.1 --port 30003 --follow > /dev/ttyUSB0

Ownship defaults to the position used by the other fixtures; override with
--lat/--lon/--alt (altitude in metres).

SBS-1 ("BaseStation") is a 22-field CSV. The fields used here:
   1 transmission type   4 hex ident    10 callsign   11 altitude (ft)
  12 ground speed (kt)  13 track (deg)  14 latitude   15 longitude
  16 vertical rate (ft/min)             7 time (HH:MM:SS.mmm)
Transmission type 1 carries the callsign, 3 the position, 4 the velocity, so a
usable target needs several messages merged.
"""
import argparse
import math
import socket
import sys
import time

FT_PER_M = 3.28084
M_PER_DEG_LAT = 111320.0


def checksum(body):
    c = 0
    for ch in body:
        c ^= ord(ch)
    return "$%s*%02X" % (body, c)


def nmea_latlon(lat, lon):
    """Decimal degrees -> (ddmm.mmmmm, N/S, dddmm.mmmmm, E/W)."""
    ns = "N" if lat >= 0 else "S"
    ew = "E" if lon >= 0 else "W"
    alat, alon = abs(lat), abs(lon)
    dlat, dlon = int(alat), int(alon)
    return ("%02d%08.5f" % (dlat, (alat - dlat) * 60), ns,
            "%03d%08.5f" % (dlon, (alon - dlon) * 60), ew)


class Ownship:
    def __init__(self, lat, lon, alt_m):
        self.lat, self.lon, self.alt_m = lat, lon, alt_m
        self.m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(lat))

    def relative(self, lat, lon, alt_ft):
        """-> (relative north m, relative east m, relative vertical m)."""
        return (int(round((lat - self.lat) * M_PER_DEG_LAT)),
                int(round((lon - self.lon) * self.m_per_deg_lon)),
                int(round(alt_ft / FT_PER_M - self.alt_m)))


class Target:
    __slots__ = ("hexid", "callsign", "alt_ft", "lat", "lon", "gs_kt",
                 "track", "vr_fpm", "last_seen")

    def __init__(self, hexid):
        self.hexid = hexid
        self.callsign = ""
        self.alt_ft = self.lat = self.lon = None
        self.gs_kt = self.track = self.vr_fpm = 0
        self.last_seen = None

    def positioned(self):
        return self.lat is not None and self.lon is not None and self.alt_ft is not None

    def pflaa(self, own, alarm=0):
        """FLARM traffic sentence. Field 5 is ICAOHEX!CALLSIGN, which is what
        RB05_FlarmParser_ParsePFLAA() reads (6 id chars, then the tail)."""
        n, e, v = own.relative(self.lat, self.lon, self.alt_ft)
        ident = "%s!%s" % (self.hexid.upper()[:6], (self.callsign or "")[:7])
        return checksum("PFLAA,%d,%d,%d,%d,1,%s,%d,0,%d,%.1f,9" % (
            alarm, n, e, v, ident, int(self.track) % 360,
            int(self.gs_kt * 1.852), self.vr_fpm * 0.00508))


def parse_sbs_line(line, targets):
    """Merge one SBS-1 record into `targets`. Returns the hex id, or None."""
    f = line.rstrip("\r\n").split(",")
    if len(f) < 17 or f[0] != "MSG":
        return None
    hexid = f[4].strip()
    if not hexid:
        return None
    t = targets.setdefault(hexid, Target(hexid))
    ttype = f[1].strip()

    def num(i, cast=float):
        try:
            return cast(f[i])
        except (ValueError, IndexError):
            return None

    if ttype == "1":
        cs = f[10].strip()
        if cs:
            t.callsign = cs
    elif ttype == "3":
        alt, lat, lon = num(11), num(14), num(15)
        if alt is not None:
            t.alt_ft = alt
        if lat is not None and lon is not None and (lat, lon) != (0.0, 0.0):
            t.lat, t.lon = lat, lon
    elif ttype == "4":
        gs, trk, vr = num(12), num(13), num(16)
        if gs is not None:
            t.gs_kt = gs
        if trk is not None:
            t.track = trk
        if vr is not None:
            t.vr_fpm = vr
    if f[7].strip():
        t.last_seen = f[7].strip()
    return hexid


def emit_cycle(out, own, targets, hms, nearest_bearing=""):
    """One output cycle: ownship GPRMC/GGA, a PFLAU, then one PFLAA each."""
    lat, ns, lon, ew = nmea_latlon(own.lat, own.lon)
    out.append(checksum("GPRMC,%s.00,A,%s,%s,%s,%s,0.0,0.0,250725,,,A"
                        % (hms, lat, ns, lon, ew)))
    out.append(checksum("GPGGA,%s.00,%s,%s,%s,%s,1,10,1.00,%.1f,M,44.3,M,,"
                        % (hms, lat, ns, lon, ew, own.alt_m)))
    out.append(checksum("PFLAU,2,1,%s,1,0,,0,,," % (nearest_bearing or "0")))
    for t in targets:
        out.append(t.pflaa(own))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="infile", help="SBS-1 capture file ('-' for stdin)")
    ap.add_argument("--host", help="live dump1090 host")
    ap.add_argument("--port", type=int, default=30003)
    ap.add_argument("--out", help="output file (default stdout)")
    ap.add_argument("--follow", action="store_true",
                    help="emit a cycle every --interval seconds, forever")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="seconds between output cycles (default 1)")
    ap.add_argument("--lat", type=float, default=43.1853257)
    ap.add_argument("--lon", type=float, default=12.1364383)
    ap.add_argument("--alt", type=float, default=278.0, help="ownship altitude, metres")
    ap.add_argument("--max-targets", type=int, default=10,
                    help="cap per cycle (firmware stores RB05_Flarm_SERIAL)")
    args = ap.parse_args()

    if not args.infile and not args.host:
        ap.error("need --in FILE or --host HOST")

    own = Ownship(args.lat, args.lon, args.alt)
    targets = {}
    out = []

    if args.host:
        sock = socket.create_connection((args.host, args.port))
        stream = sock.makefile("r", encoding="latin-1")
    else:
        stream = sys.stdin if args.infile == "-" else open(args.infile, encoding="latin-1")

    sink = open(args.out, "w", newline="\r\n") if args.out else sys.stdout

    def flush_cycle(hms):
        live = [t for t in targets.values() if t.positioned()][:args.max_targets]
        if not live:
            return
        out.clear()
        emit_cycle(out, own, live, hms)
        sink.write("\n".join(out) + "\n")
        sink.flush()

    try:
        if args.follow:
            next_at = time.time()
            for line in stream:
                parse_sbs_line(line, targets)
                now = time.time()
                if now >= next_at:
                    flush_cycle(time.strftime("%H%M%S", time.gmtime(now)))
                    next_at = now + args.interval
        else:
            # Batch: group the capture into cycles by the SBS timestamp's second,
            # then merge and emit one group at a time so a cycle never contains
            # positions from the following second.
            groups = []
            for line in stream:
                f = line.rstrip("\r\n").split(",")
                if len(f) < 17 or f[0] != "MSG":
                    continue
                sec = f[7].strip()[:8].replace(":", "")
                if not groups or groups[-1][0] != sec:
                    groups.append((sec, []))
                groups[-1][1].append(line)
            for sec, lines in groups:
                for line in lines:
                    parse_sbs_line(line, targets)
                flush_cycle(sec)
    finally:
        if sink is not sys.stdout:
            sink.close()


if __name__ == "__main__":
    main()
