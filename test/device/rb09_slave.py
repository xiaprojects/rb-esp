#!/usr/bin/env python3
"""
Hardware-in-the-loop test of an RB-09 remote display, over its USB port.

The script plays the master: it streams $RBATT and $RBAIR into the RB-09 at
10 Hz, as rb_increase_lvgl_tick() in RB02.c does on an RB-02, and checks what
the RB-09 made of them by reading back the RB-09's own $RBATT / $RBAIR. With
"Enable NMEA Output" on, the RB-09 prints those from the variables
RB09_Slave_Loop() writes:

  $RBATT  roll, pitch, yaw, accel, gyro  <- AttitudeRoll .. GyroFiltered
  $RBAIR  pressure, QNH, variometer      <- bmp280Pressure, QNH, Variometer,
                                            printed only while Operative_BMP280

so every echo has been through RB_Usb.c, RB09_Slave_ParseData() and
RB09_Slave_Loop() on the flashed firmware.

Setup, once per device: Settings -> "Enable NMEA Output" on (kept in NVS).

  test/device/rb09_slave.py                    # /dev/ttyACM0, every test
  test/device/rb09_slave.py --reset            # reboot first, check it is an RB-09
  test/device/rb09_slave.py air_inop_when_the_master_stops --log rb09.log

Exit status: 0 all passed, 1 a test failed, 2 the device could not be tested.

Not covered, as the RB-09 prints nothing for them: the attitude INOP after 1 s
(Operative_Attitude) and whatever is drawn. test/host/test_slave.c has both.

Opening an ESP32-S3 USB-Serial-JTAG port resets the chip when RTS is high while
DTR is low, and pyserial lowers DTR before RTS: the port is opened with DTR
high and RTS low so the test attaches to the running firmware. --reset pulses
RTS on purpose, as esptool's hard reset does.
"""
import argparse
import random
import sys
import threading
import time

import serial

STREAM_PERIOD = 0.1     # the RB-02 master sends at 10 Hz
AIR_TIMEOUT = 3.0       # RB09_SLAVE_AIR_TIMEOUT_US
ECHOES_PER_SECOND = 8   # the RB-09 prints at 10 Hz, less some slack
LATENCY_LIMIT = 0.35    # USB console, RB09_Slave_Loop() and one 100 ms print tick

# Output that means the firmware crashed, rebooted or stalled
CRASH_MARKERS = ("Guru Meditation", "abort() was called", "Backtrace:",
                 "ESP-ROM:", "task_wdt", "stack overflow")

# The example in RB09_Slave.c, and an attitude differing from it in every field
A = (-12.50, 3.25, 87.10, 0.98, -0.21, 0.05, 0.10, -1.20, 4.80)
B = (25.00, -7.50, 181.25, 0.12, 0.34, 1.02, -2.50, 3.75, -10.00)
# GPS acceleration, magnetometer, compass and filter beta, as an RB-02 sends them
TAIL = (0, 0, 0, 0, 0, 0, 0, 0.04)

# Static pressure in Pa, QNH in hPa, variometer in feet/minute
AIR1 = (95000, 1020.0, 500)
AIR2 = (89875, 1005.0, -1200)


def nmea(body):
    """'RBATT,...' -> '$RBATT,...*hh', as nmeaChecksum() computes it."""
    c = 0
    for ch in body:
        c ^= ord(ch)
    return "$%s*%02X" % (body, c)


def fields(values):
    return ",".join("%.2f" % v for v in values)


def rbatt(att, tail=TAIL):
    """$RBATT as rb_increase_lvgl_tick() sends it: att is roll .. gyro z."""
    return nmea("RBATT," + fields(tuple(att) + tuple(tail)))


def rbair(pressure, qnh, vario):
    """$RBAIR as RB09_Slave_FormatAir() writes it."""
    return nmea("RBAIR,%.4f,%.5f,,%.0f,,," % (pressure / 3386.39, qnh / 33.8639, vario))


def flip_checksum(line):
    return line[:-2] + "%02X" % (int(line[-2:], 16) ^ 0x01)


def lowercase_checksum(att):
    """rbatt(att) with its checksum in lower case. Beta is nudged until the
    checksum has a hex letter, or lower case would change nothing."""
    for beta in range(4, 100):
        line = rbatt(att, TAIL[:-1] + (beta / 100.0,))
        if not line[-2:].isdigit():
            return line[:-2] + line[-2:].lower()


def parse_echo(text):
    """'$RBATT,...*hh' -> ('RBATT', [float or None, ...]), None unless intact.
    Only a line that starts with the sentence: firmware that logs its input
    ("Slave Data: $RBATT,..." in RB_Usb.c) must not read as the RB-09's state."""
    star = text.rfind("*")
    if not text.startswith("$RB") or star < 0:
        return None
    body = text[1:star]
    if text[star + 1:star + 3].upper() != nmea(body)[-2:]:
        return None
    kind, *values = body.split(",")
    try:
        return kind, [float(v) if v else None for v in values]
    except ValueError:
        return None


def same_attitude(values, att):
    """The 9 mandatory $RBATT fields, as printed with 2 decimals."""
    return len(values) >= 9 and all(v is not None and abs(v - a) < 0.006
                                    for v, a in zip(values, att))


def same_air(values, air):
    """An $RBAIR echo against (Pa, hPa, feet/minute). The RB-09 rounds the
    pressure to 1 Pa, snaps QNH to 0.001 hPa and keeps the variometer in
    0.01 feet/second."""
    if len(values) < 4 or None in (values[0], values[1], values[3]):
        return False
    pressure, qnh, vario = air
    return (abs(values[0] * 3386.39 - pressure) < 1.0 and
            abs(values[1] * 33.8639 - qnh) < 0.002 and
            abs(values[3] - vario) <= 1)


def air_text(air):
    return "%.0f Pa, QNH %.3f hPa, %+.0f ft/min" % air


def describe_air(values):
    """An $RBAIR echo in the units of AIR1."""
    if len(values) < 4 or None in (values[0], values[1], values[3]):
        return str(values)
    return air_text((values[0] * 3386.39, values[1] * 33.8639, values[3]))


class Failure(Exception):
    pass


class SetupError(Exception):
    pass


def check(condition, message):
    if not condition:
        raise Failure(message)


class Device:
    """The RB-09 on a serial port. A reader thread timestamps every line it
    prints, a streamer thread repeats what the master currently sends."""

    def __init__(self, port, log=None):
        self.port = serial.Serial()
        self.port.port = port
        self.port.baudrate = 115200  # USB-Serial-JTAG ignores it
        self.port.timeout = 0.05
        self.port.write_timeout = 5
        self.port.dtr = True
        self.port.rts = False
        self.port.open()
        self.log = log
        self.started = time.monotonic()
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.log_lock = threading.Lock()
        self.lines = []          # (time, text, parse_echo(text))
        self.streaming = ()
        self.next_send = 0.0
        self.last_sent = {}      # sentence name -> time it was last written
        self.error = None
        self.closed = False
        self.threads = [threading.Thread(target=self._read, daemon=True),
                        threading.Thread(target=self._stream, daemon=True)]
        for thread in self.threads:
            thread.start()

    def close(self):
        self.closed = True
        for thread in self.threads:
            thread.join(1)
        self.port.close()

    def now(self):
        return time.monotonic()

    def _log(self, t, direction, text):
        if self.log:
            with self.log_lock:
                self.log.write("%9.3f %s %s\n" % (t - self.started, direction, text))

    def _read(self):
        pending = b""
        while not self.closed:
            try:
                data = self.port.read(self.port.in_waiting or 1)
            except serial.SerialException as e:
                self.error = str(e)
                return
            if not data:
                continue
            pending += data
            *complete, pending = pending.split(b"\n")
            t = time.monotonic()
            for raw in complete:
                text = raw.decode("ascii", "replace").strip()
                with self.lock:
                    self.lines.append((t, text, parse_echo(text)))
                self._log(t, "<", text)

    def _stream(self):
        while not self.closed:
            with self.lock:
                due = self.streaming and time.monotonic() >= self.next_send
                lines = self.streaming if due else ()
                if due:
                    self.next_send = max(self.next_send + STREAM_PERIOD, time.monotonic())
            for line in lines:
                try:
                    self.send(line)
                except serial.SerialException as e:
                    self.error = str(e)
                    return
            time.sleep(0.005)

    def stream(self, *lines):
        """Sends lines now and then every 100 ms, as the master does. No lines
        silences the master."""
        with self.lock:
            self.streaming = tuple(line for line in lines if line)
            self.next_send = time.monotonic()

    def send(self, line):
        """Writes one line, as the master prints it."""
        self.send_raw((line + "\n").encode("latin-1"))
        if line.startswith("$"):
            self.last_sent[line[1:6]] = time.monotonic()

    def send_raw(self, data):
        try:
            with self.write_lock:
                self.port.write(data)
        except serial.SerialTimeoutException:
            raise Failure("the RB-09 stopped reading its USB input")
        except serial.SerialException as e:
            # Unplugged: main() sees self.error after this test and stops the run
            self.error = str(e)
            raise Failure("lost the port: %s" % e)
        t = time.monotonic()
        if self.log:
            text = data.decode("latin-1").rstrip("\r\n")
            if len(data) > 200 or not text.isprintable():
                text = "<%d bytes>" % len(data)
            self._log(t, ">", text)

    def reset(self):
        """EN pulse through RTS, DTR low so it boots the firmware, not the ROM loader."""
        self.port.dtr = False
        time.sleep(0.1)
        self.port.rts = True
        time.sleep(0.2)
        self.port.rts = False
        time.sleep(0.1)
        self.port.dtr = True

    def lines_since(self, since):
        with self.lock:
            return [line for line in self.lines if line[0] >= since]

    def echoes(self, kind, since):
        """[(time, values)] of the intact $<kind> the RB-09 printed since then."""
        return [(t, echo[1]) for t, _, echo in self.lines_since(since)
                if echo is not None and echo[0] == kind]

    def last_echo(self, kind):
        echoes = self.echoes(kind, 0)
        return echoes[-1][1] if echoes else None

    def wait_line(self, match, since, timeout):
        """The first line since then that match() accepts, None after timeout."""
        deadline = time.monotonic() + timeout
        while True:
            for _, text, _ in self.lines_since(since):
                if match(text):
                    return text
            if time.monotonic() > deadline or self.error:
                return None
            time.sleep(0.01)

    def wait_echo(self, kind, match, since, timeout):
        """Time of the first $<kind> since then that match() accepts, None
        after timeout."""
        deadline = time.monotonic() + timeout
        while True:
            for t, values in self.echoes(kind, since):
                if match(values):
                    return t
            if time.monotonic() > deadline or self.error:
                return None
            time.sleep(0.01)

    def wait_attitude(self, att, timeout=1.0, since=None):
        """Seconds until the RB-09 echoes att."""
        since = self.now() if since is None else since
        t = self.wait_echo("RBATT", lambda v: same_attitude(v, att), since, timeout)
        if t is None:
            last = self.last_echo("RBATT")
            raise Failure("sent %s, after %.1f s the RB-09 still echoes %s" %
                          (fields(att), timeout, fields(last[:9]) if last else "nothing"))
        return t - since

    def wait_air(self, air, timeout=1.5, since=None):
        """Seconds until the RB-09 echoes air."""
        since = self.now() if since is None else since
        t = self.wait_echo("RBAIR", lambda v: same_air(v, air), since, timeout)
        if t is None:
            last = self.last_echo("RBAIR")
            raise Failure("sent %s, after %.1f s the RB-09 still echoes %s" %
                          (air_text(air), timeout, describe_air(last) if last else "no $RBAIR"))
        return t - since

    def crash_since(self, since):
        for _, text, _ in self.lines_since(since):
            if any(marker in text for marker in CRASH_MARKERS):
                return text
        return None


def inject(dev, line, att, air=None):
    """Brings the RB-09 to att (and air), silences the master and sends line
    five times at the master's rate. Returns the time of the first send.
    The window stays under both timeouts, so the baseline is still current."""
    dev.stream(rbatt(att), air and rbair(*air))
    dev.wait_attitude(att)
    if air:
        dev.wait_air(air)
        time.sleep(0.3)  # past the tick where RB02_AltimeterQNHUpdated() zeroes Variometer
    dev.stream()
    t = dev.now()
    for _ in range(5):
        dev.send(line)
        time.sleep(STREAM_PERIOD)
    time.sleep(0.2)  # the last one through RB09_Slave_Loop() and one print tick
    return t


def settled(echoes, match):
    """The last three echoes all match: the RB-09 took the injected value."""
    return len(echoes) >= 3 and all(match(values) for _, values in echoes[-3:])


# Field texts for sentences no RB-02 would send
B_TEXT = ["%.2f" % v for v in B + TAIL]
AIR2_TEXT = ["%.4f" % (AIR2[0] / 3386.39), "%.5f" % (AIR2[1] / 33.8639), "", "%.0f" % AIR2[2], "", "", ""]

# Every field to a fixed width, blank padded, as a printf("%9.2f") master writes it
B_PADDED = ["%9.2f" % v for v in B + TAIL]

RBATT_REJECTED = [
    ("wrong checksum", flip_checksum(rbatt(B))),
    ("no checksum", rbatt(B)[:-3]),
    ("cut mid-sentence", rbatt(B)[:40]),
    ("every field empty", nmea("RBATT," + "," * 16)),
    ("every field blank", nmea("RBATT," + ",".join(["   "] * 17))),
    ("every field invalid", nmea("RBATT," + ",".join(["400", "abc", "-400", "2.5e1", "", "99"]))),
]

RBATT_ACCEPTED = [
    ("as an RB-02 sends it", rbatt(B)),
    ("roll to gyro only", rbatt(B, tail=())),
    ("fields after the gyro empty", nmea("RBATT," + ",".join(B_TEXT[:9] + [""] * 8))),
    ("fields after the gyro blank", nmea("RBATT," + ",".join(B_TEXT[:9] + ["  "] * 8))),
    ("padded to a fixed width", nmea("RBATT," + ",".join(B_PADDED))),
    ("extra fields appended", rbatt(B, TAIL + (1, 2))),
    ("lowercase checksum", lowercase_checksum(B)),
    ("leading garbage", "xx" + rbatt(B)),
    ("CR LF line end", rbatt(B) + "\r"),
]

# Used with the invalid fields at 0: (name, sentence, roll .. gyro z the RB-09
# should end up echoing). Each keeps the acceleration, as a 0 g there would
# reach the G-meter extremes the RB-09 stores.
RBATT_DEFAULTED = [
    ("roll beyond 360 deg", rbatt((400.0,) + B[1:]), (0,) + B[1:]),
    ("gyro X beyond 2048 deg/s", rbatt(B[:6] + (2100.0,) + B[7:]), B[:6] + (0,) + B[7:]),
    ("gyro Z empty", nmea("RBATT," + ",".join(B_TEXT[:8] + [""] + B_TEXT[9:])), B[:8] + (0,)),
    ("exponent notation in the roll", nmea("RBATT," + ",".join(["2.5e1"] + B_TEXT[1:])), (0,) + B[1:]),
    ("no gyro: the sentence ends after the acceleration", nmea("RBATT," + ",".join(B_TEXT[:6])),
     B[:6] + (0, 0, 0)),
]

RBAIR_REJECTED = [
    ("wrong checksum", flip_checksum(rbair(*AIR2))),
    ("no checksum", rbair(*AIR2)[:-3]),
    ("every field empty", nmea("RBAIR," + "," * 6)),
    ("every field blank", nmea("RBAIR," + ",".join([" "] * 7))),
]

# (name, sentence, what the RB-09 should end up echoing)
RBAIR_ACCEPTED = [
    ("as RB09_Slave_FormatAir() writes it", rbair(*AIR2), AIR2),
    ("every field, QNH in inHg: the example in RB09_Slave.c",
     nmea("RBAIR,29.921,29.92,15.0,500,120,20,270"), (29.921 * 3386.39, 29.92 * 33.8639, 500)),
    ("extra fields appended", nmea("RBAIR," + ",".join(AIR2_TEXT + ["1", "2"])), AIR2),
    ("padded to a fixed width",
     nmea("RBAIR,%10.4f,%10.5f,%6s,%6.0f,%6s,%6s,%6s" % (AIR2[0] / 3386.39, AIR2[1] / 33.8639, "", AIR2[2], "", "", "")),
     AIR2),
    ("empty variometer reads 0", nmea("RBAIR," + ",".join(AIR2_TEXT[:3] + [""] + AIR2_TEXT[4:])),
     AIR2[:2] + (0,)),
    ("pressure and QNH only", nmea("RBAIR," + ",".join(AIR2_TEXT[:2])), AIR2[:2] + (0,)),
]

# Used, but with no altitude in it: the pressure or QNH missing, so read as 0,
# or out of the range Get_BMP280() accepts
AIR_WITHOUT_ALTITUDE = [
    ("QNH 800 hPa", rbair(78000, 800.0, 0)),
    ("QNH 1150 hPa", rbair(112000, 1150.0, 0)),
    ("1400 ft below the QNH level", rbair(106000, 1013.25, 0)),
    ("empty pressure", nmea("RBAIR," + ",".join([""] + AIR2_TEXT[1:]))),
    ("empty QNH", nmea("RBAIR," + ",".join(AIR2_TEXT[:1] + [""] + AIR2_TEXT[2:]))),
    ("letter in the pressure", nmea("RBAIR," + ",".join(["28.1O"] + AIR2_TEXT[1:]))),
]

TESTS = []


def test(fn):
    TESTS.append(fn)
    return fn


@test
def attitude_round_trip(dev):
    """The 9 mandatory $RBATT fields come back as sent, at 10 Hz."""
    dev.stream(rbatt(A))
    latency = dev.wait_attitude(A)
    t = dev.now()
    time.sleep(1.0)
    echoes = dev.echoes("RBATT", t)
    check(len(echoes) >= ECHOES_PER_SECOND, "%d $RBATT echoes in 1 s" % len(echoes))
    wrong = [values for _, values in echoes if not same_attitude(values, A)]
    if wrong:
        raise Failure("sent %s, echoed %s" % (fields(A), fields(wrong[0][:9])))
    return "%d echoes in 1 s, all as sent; first after %d ms" % (len(echoes), latency * 1000)


@test
def attitude_follows_the_master(dev):
    """A bank from -60 to +60 deg in 30 deg steps reaches the RB-09 step by step."""
    latencies = []
    for roll in (-60, -30, 0, 30, 60):
        att = (roll, roll / 4, 180 + 2 * roll, roll / 100, -roll / 100, 1.0, roll / 10, -roll / 10, roll / 5)
        dev.stream(rbatt(att))
        latencies.append(dev.wait_attitude(att))
        time.sleep(0.4)
    check(max(latencies) <= LATENCY_LIMIT,
          "a step took %d ms, limit %d ms" % (max(latencies) * 1000, LATENCY_LIMIT * 1000))
    return "5 steps, each on the RB-09 after %d-%d ms" % (min(latencies) * 1000, max(latencies) * 1000)


@test
def rbatt_rejects_damaged_sentences(dev):
    """A $RBATT that is cut, fails its checksum or has no valid field is dropped:
    the RB-09 keeps the last good attitude."""
    used = []
    for name, line in RBATT_REJECTED:
        t = inject(dev, line, A)
        echoes = dev.echoes("RBATT", t)
        check(len(echoes) >= 5, "%s: %d $RBATT echoes in 0.7 s" % (name, len(echoes)))
        if not all(same_attitude(values, A) for _, values in echoes):
            used.append(name)
    check(not used, "used anyway: " + ", ".join(used))
    return "%d kinds of damage dropped" % len(RBATT_REJECTED)


@test
def rbatt_accepts_valid_forms(dev):
    """Every $RBATT form RB09_Slave.c allows reaches the instruments."""
    dropped = []
    for name, line in RBATT_ACCEPTED:
        t = inject(dev, line, A)
        if not settled(dev.echoes("RBATT", t), lambda values: same_attitude(values, B)):
            dropped.append(name)
    check(not dropped, "not used: " + ", ".join(dropped))
    return "%d forms used" % len(RBATT_ACCEPTED)


@test
def rbatt_invalid_fields_read_zero(dev):
    """A $RBATT with some fields invalid is used, those fields at 0."""
    wrong = []
    for name, line, expected in RBATT_DEFAULTED:
        t = inject(dev, line, A)
        if not settled(dev.echoes("RBATT", t), lambda values: same_attitude(values, expected)):
            wrong.append(name)
    check(not wrong, "not as expected: " + ", ".join(wrong))
    return "%d sentences used with their invalid fields at 0" % len(RBATT_DEFAULTED)


@test
def air_round_trip(dev):
    """Pressure, QNH and variometer come back as sent, with the master's QNH adopted."""
    for air in (AIR1, AIR2):
        dev.stream(rbatt(A), rbair(*air))
        dev.wait_air(air)
        time.sleep(0.3)  # past the tick where RB02_AltimeterQNHUpdated() zeroes Variometer
        t = dev.now()
        time.sleep(1.0)
        echoes = dev.echoes("RBAIR", t)
        check(len(echoes) >= ECHOES_PER_SECOND, "%d $RBAIR echoes in 1 s" % len(echoes))
        wrong = [values for _, values in echoes if not same_air(values, air)]
        if wrong:
            raise Failure("sent %s, echoed %s" % (air_text(air), describe_air(wrong[0])))
    return "QNH %.0f then %.0f hPa adopted, every echo as sent" % (AIR1[1], AIR2[1])


@test
def rbair_rejects_damaged_sentences(dev):
    """A $RBAIR that fails its checksum or has no valid field is dropped: the
    RB-09 keeps the last good air data."""
    used = []
    for name, line in RBAIR_REJECTED:
        t = inject(dev, line, A, AIR1)
        echoes = dev.echoes("RBAIR", t)
        check(len(echoes) >= 5, "%s: %d $RBAIR echoes in 0.7 s" % (name, len(echoes)))
        if not all(same_air(values, AIR1) for _, values in echoes):
            used.append(name)
    check(not used, "used anyway: " + ", ".join(used))
    return "%d kinds of damage dropped" % len(RBAIR_REJECTED)


@test
def rbair_accepts_valid_forms(dev):
    """Every $RBAIR form RB09_Slave.c allows reaches the instruments."""
    dropped = []
    for name, line, expected in RBAIR_ACCEPTED:
        t = inject(dev, line, A, AIR1)
        if not settled(dev.echoes("RBAIR", t), lambda values: same_air(values, expected)):
            dropped.append(name)
    check(not dropped, "not used: " + ", ".join(dropped))
    return "%d forms used" % len(RBAIR_ACCEPTED)


@test
def both_sentences_on_one_line(dev):
    """$RBATT and $RBAIR with no line end between them are both used."""
    t = inject(dev, rbatt(B) + rbair(*AIR2), A, AIR1)
    check(settled(dev.echoes("RBATT", t), lambda values: same_attitude(values, B)), "$RBATT not used")
    check(settled(dev.echoes("RBAIR", t), lambda values: same_air(values, AIR2)), "$RBAIR not used")
    return "both used"


@test
def air_out_of_range_is_inop(dev):
    """Air data with no valid pressure or QNH, or out of the Get_BMP280() range,
    takes the barometer INOP, fresh as it is."""
    used = []
    for name, line in AIR_WITHOUT_ALTITUDE:
        dev.stream(rbatt(A), rbair(*AIR1))
        dev.wait_air(AIR1)
        dev.stream(rbatt(A), line)
        t = dev.now()
        time.sleep(1.5)
        late = dev.echoes("RBAIR", t + 0.3)
        if late:
            used.append("%s, echoed %s" % (name, describe_air(late[-1][1])))
    check(not used, "still operative: " + "; ".join(used))
    return "%d cases INOP within 0.3 s" % len(AIR_WITHOUT_ALTITUDE)


@test
def air_inop_when_the_master_stops(dev):
    """The barometer goes INOP 3 s after the last $RBAIR and recovers with the next."""
    dev.stream(rbatt(A), rbair(*AIR1))
    dev.wait_air(AIR1)
    time.sleep(0.5)
    dev.stream(rbatt(A))  # the master's barometer fails, its attitude keeps coming
    time.sleep(AIR_TIMEOUT + 1.0)
    last_sent = dev.last_sent["RBAIR"]
    echoes = dev.echoes("RBAIR", last_sent)
    check(echoes, "no $RBAIR echo after the last one sent")
    held = echoes[-1][0] - last_sent
    check(AIR_TIMEOUT - 0.3 < held < AIR_TIMEOUT + 0.3,
          "$RBAIR echoed for %.2f s after the last one sent, the timeout is %.1f s" % (held, AIR_TIMEOUT))
    dev.stream(rbatt(A), rbair(*AIR1))
    back = dev.wait_air(AIR1, timeout=1.0)
    return "INOP %.2f s after the last $RBAIR, back %d ms after it resumed" % (held, back * 1000)


@test
def survives_garbage(dev):
    """Overlong lines and binary noise neither crash the RB-09 nor reach an instrument."""
    dev.stream(rbatt(A))
    dev.wait_attitude(A)
    dev.stream()
    t = dev.now()
    garbage = [b"X" * 1000 + b"\n",                                   # 4x the RB_Usb.c line buffer
               (rbatt(A, TAIL + (0,) * 60) + "\n").encode(),          # $RBATT past 256 bytes
               bytes(random.Random(9).randrange(256) for _ in range(4096)) + b"\n",
               b"$" * 300 + b"\n"]
    for data in garbage:
        dev.send_raw(data)
    time.sleep(0.5)
    crash = dev.crash_since(t)
    check(crash is None, "the RB-09 crashed: " + str(crash))
    wrong = [values for _, values in dev.echoes("RBATT", t) if not same_attitude(values, A)]
    if wrong:
        raise Failure("the attitude changed to %s" % fields(wrong[0][:9]))
    dev.stream(rbatt(B))
    latency = dev.wait_attitude(B)
    return "%d bytes ignored, next attitude on after %d ms" % (sum(map(len, garbage)), latency * 1000)


def preflight(dev, port, reset):
    if reset:
        t = dev.now()
        dev.reset()
        hello = dev.wait_line(lambda text: "$RB," in text, t, 15)
        if dev.wait_line(lambda text: "waiting for download" in text, t, 0):
            raise SetupError("the reset left the RB-09 in its ROM loader: power cycle it")
        if hello is None:
            raise SetupError("no $RB identification within 15 s of the reset")
        product = hello[hello.find("$RB,"):].split(",")
        if product[1] != "9":
            raise SetupError("product line %s, not an RB-09: %s" % (product[1], hello))
        print("%s: %s" % (port, hello))
    if dev.wait_echo("RBATT", lambda values: True, dev.now(), 15 if reset else 3) is None:
        if dev.error:
            raise SetupError("lost %s: %s" % (port, dev.error))
        raise SetupError(
            "the RB-09 prints no $RBATT, so there is nothing to read back.\n"
            "On the RB-09: Settings -> \"Enable NMEA Output\" on (stored, once per device), then run again.")


def main():
    ap = argparse.ArgumentParser(description="Hardware-in-the-loop test of an RB-09 remote display over USB.")
    ap.add_argument("tests", nargs="*", metavar="TEST", help="run only these (default: all, see --list)")
    ap.add_argument("--port", default="/dev/ttyACM0", help="the RB-09's USB serial port (default %(default)s)")
    ap.add_argument("--reset", action="store_true", help="reboot the RB-09 first and check it identifies as one")
    ap.add_argument("--log", metavar="FILE", help="write every line sent (>) and received (<), timestamped")
    ap.add_argument("--list", action="store_true", help="list the tests and exit")
    args = ap.parse_args()

    if args.list:
        for fn in TESTS:
            print("%-34s %s" % (fn.__name__, fn.__doc__))
        return 0
    by_name = {fn.__name__: fn for fn in TESTS}
    unknown = [name for name in args.tests if name not in by_name]
    if unknown:
        ap.error("no such test: %s (see --list)" % ", ".join(unknown))
    selected = [by_name[name] for name in args.tests] or TESTS

    log = open(args.log, "w") if args.log else None
    try:
        dev = Device(args.port, log)
    except serial.SerialException as e:
        print("cannot open %s: %s" % (args.port, e), file=sys.stderr)
        return 2
    try:
        preflight(dev, args.port, args.reset)
        failed = 0
        for fn in selected:
            start = dev.now()
            try:
                note = fn(dev)
                crash = dev.crash_since(start)
                check(crash is None, "the RB-09 crashed or rebooted: %s" % crash)
                print("PASS %-34s %s" % (fn.__name__, note))
            except Failure as e:
                failed += 1
                print("FAIL %-34s %s" % (fn.__name__, e))
            sys.stdout.flush()
            if dev.error:
                raise SetupError("lost %s: %s" % (args.port, dev.error))
        dev.stream()
        print("%d passed, %d failed" % (len(selected) - failed, failed))
        return 1 if failed else 0
    except SetupError as e:
        print("cannot test: %s" % e, file=sys.stderr)
        return 2
    finally:
        dev.close()
        if log:
            log.close()


if __name__ == "__main__":
    sys.exit(main())
