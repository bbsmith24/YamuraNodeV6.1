#!/usr/bin/env python3
"""
ntrip2serial - NTRIP caster -> serial port, for RTCM3 corrections on the bench.

Does the str2str job (RTKLIB-EX's Windows build has no str2str.exe):
  - logs in to an NTRIP caster (NTRIP v1, HTTP basic auth)
  - sends an NMEA GGA position line (--lat/--lon, required) on connect and every
    --gga-interval seconds (network and "nearest" mount points need it; MDOT asks
    for it on every product). Use the base's position - at the track, the track's.
  - writes the RTCM3 stream to a serial port (the base radio, or the Teensy's USB port)
  - reports each epoch's size and message types, and flags epochs over --limit bytes
    (the DX-SMART LoRa modules drop anything past ~455 bytes per burst)

The password is read from the NTRIP_PASSWORD environment variable, never the
command line, so it doesn't end up in shell history or logs. The user name can come
from --user or NTRIP_USER.

Examples (PowerShell):
  $env:NTRIP_USER = 'name'; $env:NTRIP_PASSWORD = 'secret'
  python ntrip2serial.py --mount NEAREST_RTCM3-GG --lat 42.4467 --lon -83.4557 --height 250 --serial COM20
  python ntrip2serial.py --mount MILI_RTCM3-GG --lat 42.4467 --lon -83.4557 --height 250   # no serial: report only

Needs pyserial (pip install pyserial) when --serial is given.
"""
import argparse
import base64
import os
import socket
import sys
import time
from collections import Counter
from datetime import datetime, timezone

MDOT_HOST = "148.149.0.87"
MDOT_PORT = 10001
USER_AGENT = "NTRIP ntrip2serial/1.0"


# ------------------------------------------------------------------ NMEA GGA
def gga_sentence(lat, lon, height_m):
    """GGA for a fixed position (fix quality 1, 12 satellites, HDOP 1.0)."""
    now = datetime.now(timezone.utc)
    hhmmss = now.strftime("%H%M%S") + ".00"
    lat_h = "N" if lat >= 0 else "S"
    lon_h = "E" if lon >= 0 else "W"
    lat_a, lon_a = abs(lat), abs(lon)
    lat_s = "%02d%08.5f" % (int(lat_a), (lat_a - int(lat_a)) * 60.0)
    lon_s = "%03d%08.5f" % (int(lon_a), (lon_a - int(lon_a)) * 60.0)
    # height is ellipsoidal; GGA wants MSL + geoid separation, so put it all in MSL
    # with separation 0 - casters only use the position to pick a station.
    body = "GPGGA,%s,%s,%s,%s,%s,1,12,1.0,%.1f,M,0.0,M,," % (hhmmss, lat_s, lat_h, lon_s, lon_h, height_m)
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return "$%s*%02X\r\n" % (body, checksum)


# ------------------------------------------------------------------ RTCM3 framing
def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class RtcmScanner:
    """Finds RTCM3 frames in a byte stream (preamble 0xD3, 10-bit length, CRC-24Q)."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        """Returns a list of (message type, frame length in bytes) for complete frames."""
        self.buf += data
        frames = []
        while True:
            start = self.buf.find(b"\xd3")
            if start < 0:
                self.buf.clear()
                break
            if start > 0:
                del self.buf[:start]
            if len(self.buf) < 3:
                break
            length = ((self.buf[1] & 0x03) << 8) | self.buf[2]
            total = 3 + length + 3
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big") and length >= 2:
                msg_type = (frame[3] << 4) | (frame[4] >> 4)
                frames.append((msg_type, total))
                del self.buf[:total]
            else:
                self.bad_crc += 1
                del self.buf[:1]  # not a real frame start; resync
        return frames


class EpochTracker:
    """Groups frames into epochs: a new epoch starts after a gap in the stream."""

    def __init__(self, gap_s, limit, verbose=True):
        self.gap_s = gap_s
        self.limit = limit
        self.verbose = verbose
        self.last_time = None
        self.bytes = 0
        self.types = Counter()
        self.count = 0
        self.over = 0
        self.max_bytes = 0
        self.all_types = Counter()

    def add(self, now, frames):
        if not frames:
            return
        if self.last_time is not None and now - self.last_time > self.gap_s and self.bytes:
            self.finish()
        for msg_type, size in frames:
            self.bytes += size
            self.types[msg_type] += 1
            self.all_types[msg_type] += 1
        self.last_time = now

    def finish(self):
        if not self.bytes:
            return
        self.count += 1
        self.max_bytes = max(self.max_bytes, self.bytes)
        flag = ""
        if self.bytes > self.limit:
            self.over += 1
            flag = "   OVER %d" % self.limit
        if self.verbose or flag:
            types = " ".join("%d%s" % (t, "x%d" % n if n > 1 else "") for t, n in sorted(self.types.items()))
            print("%s epoch %5d  %4d B  %s%s" % (time.strftime("%H:%M:%S"), self.count, self.bytes, types, flag), flush=True)
        self.bytes = 0
        self.types.clear()

    def summary(self):
        types = ", ".join("%d: %d" % (t, n) for t, n in sorted(self.all_types.items()))
        return "%d epochs, largest %d B, %d over %d B; messages %s" % (
            self.count, self.max_bytes, self.over, self.limit, types or "none")


# ------------------------------------------------------------------ NTRIP
def connect(args, user, password):
    sock = socket.create_connection((args.host, args.port), timeout=15)
    auth = base64.b64encode(("%s:%s" % (user, password)).encode()).decode()
    request = ("GET /%s HTTP/1.0\r\n"
               "User-Agent: %s\r\n"
               "Accept: */*\r\n"
               "Authorization: Basic %s\r\n"
               "Connection: close\r\n\r\n") % (args.mount, USER_AGENT, auth)
    sock.sendall(request.encode())
    # the reply header ends at a blank line; ICY 200 OK (v1) or HTTP/1.x 200
    header = b""
    while b"\r\n\r\n" not in header and len(header) < 8192:
        chunk = sock.recv(1024)
        if not chunk:
            break
        header += chunk
    head, _, rest = header.partition(b"\r\n\r\n")
    status = head.split(b"\r\n", 1)[0].decode(errors="replace")
    if " 200" not in status:
        sock.close()
        raise ConnectionError("caster refused: %s" % status)
    return sock, status, rest


def main():
    p = argparse.ArgumentParser(description="NTRIP caster -> serial port (RTCM3), with epoch size checks")
    p.add_argument("--host", default=MDOT_HOST, help="caster address (default MDOT CORS %s)" % MDOT_HOST)
    p.add_argument("--port", type=int, default=MDOT_PORT, help="caster port (default %d)" % MDOT_PORT)
    p.add_argument("--mount", required=True, help="mount point, e.g. NEAREST_RTCM3-GG or MILI_RTCM3-GG")
    p.add_argument("--user", default=os.environ.get("NTRIP_USER"), help="user name (default: NTRIP_USER)")
    p.add_argument("--lat", type=float, required=True,
                   help="base position for the GGA sent to the caster, degrees (+N); use the track's, not home's")
    p.add_argument("--lon", type=float, required=True, help="base position for GGA, degrees (+E)")
    p.add_argument("--height", type=float, default=0.0, help="ellipsoidal height for GGA, m")
    p.add_argument("--gga-interval", type=float, default=10.0, help="seconds between GGA lines (0 = once)")
    p.add_argument("--serial", help="serial port for the RTCM stream (omit to only report)")
    p.add_argument("--baud", type=int, default=57600, help="serial baud (default 57600, the radio's)")
    p.add_argument("--limit", type=int, default=450, help="flag epochs bigger than this many bytes (default 450)")
    p.add_argument("--gap", type=float, default=0.3, help="seconds of quiet that end an epoch (default 0.3)")
    p.add_argument("--quiet", action="store_true", help="only print epochs over --limit, plus a summary every minute")
    p.add_argument("--status", action="store_true",
                   help="also print the 'Status' lines read back from --serial (the GPSRTK Teensy's USB port), "
                        "since this program holds the port and nothing else can read it")
    p.add_argument("--log", help="also append everything printed to this file")
    args = p.parse_args()
    if args.log:
        log_file = open(args.log, "a", buffering=1, encoding="utf-8")
        class Tee:
            def write(self, text):
                sys.__stdout__.write(text)
                log_file.write(text)
            def flush(self):
                sys.__stdout__.flush()
                log_file.flush()
        sys.stdout = Tee()
    status_buf = bytearray()

    password = os.environ.get("NTRIP_PASSWORD")
    if not args.user or password is None:
        sys.exit("set the user name (--user or NTRIP_USER) and the NTRIP_PASSWORD environment variable")
    if not (-90.0 <= args.lat <= 90.0 and -180.0 <= args.lon <= 180.0):
        sys.exit("--lat must be -90..90 and --lon -180..180 (decimal degrees)")

    out = None
    if args.serial:
        import serial  # pyserial
        out = serial.Serial(args.serial, args.baud, timeout=0)
        print("writing RTCM to %s at %d baud" % (args.serial, args.baud))
    else:
        print("no --serial: reporting only")

    scanner = RtcmScanner()
    epochs = EpochTracker(args.gap, args.limit, verbose=not args.quiet)

    total_bytes = 0
    last_summary = time.monotonic()
    try:
        while True:
            try:
                sock, status, rest = connect(args, args.user, password)
            except (OSError, ConnectionError) as e:
                print("connect failed: %s - retry in 10 s" % e, flush=True)
                time.sleep(10)
                continue
            print("connected to %s:%d/%s (%s)" % (args.host, args.port, args.mount, status), flush=True)
            sock.settimeout(1.0)
            last_gga = 0.0
            last_data = time.monotonic()
            pending = rest
            try:
                while True:
                    now = time.monotonic()
                    if last_gga == 0.0 or (args.gga_interval > 0 and now - last_gga >= args.gga_interval):
                        sock.sendall(gga_sentence(args.lat, args.lon, args.height).encode())
                        last_gga = now
                    if pending:
                        data, pending = pending, b""
                    else:
                        try:
                            data = sock.recv(4096)
                        except socket.timeout:
                            data = None
                        if data == b"":
                            raise ConnectionError("caster closed the connection")
                    now = time.monotonic()
                    if data:
                        last_data = now
                        total_bytes += len(data)
                        if out is not None:
                            out.write(data)
                        epochs.add(now, scanner.feed(data))
                    elif epochs.last_time is not None and now - epochs.last_time > args.gap:
                        epochs.finish()
                    if args.status and out is not None and out.in_waiting:
                        status_buf += out.read(out.in_waiting)
                        while b"\n" in status_buf:
                            line, _, rest_buf = status_buf.partition(b"\n")
                            status_buf[:] = rest_buf
                            text = line.decode(errors="replace").strip()
                            if text.startswith("Status") or text.startswith("ZED-F9P"):
                                print("%s rover  %s" % (time.strftime("%H:%M:%S"), text.replace("\t", " ")), flush=True)
                        if len(status_buf) > 4096:
                            status_buf.clear()
                    if now - last_data > 30:
                        raise ConnectionError("no data for 30 s")
                    if now - last_summary >= 60:
                        print("-- %s; %d bytes total, %d bad frames" % (epochs.summary(), total_bytes, scanner.bad_crc), flush=True)
                        last_summary = now
            except (OSError, ConnectionError) as e:
                print("stream ended: %s - reconnecting in 5 s" % e, flush=True)
                try:
                    sock.close()
                except OSError:
                    pass
                time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        epochs.finish()
        print("-- %s; %d bytes total, %d bad frames" % (epochs.summary(), total_bytes, scanner.bad_crc))
        if out is not None:
            out.close()


if __name__ == "__main__":
    main()
