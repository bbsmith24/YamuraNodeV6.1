# YamuraNode_GPSRTK — Teensy 4.0 + ZED-F9P

PlatformIO project for an RTK GPS node: a Teensy 4.0 with a SparkFun
GPS-RTK-SMA Breakout (u-blox ZED-F9P).

Stage 1 is the GPS-RTK module only — **no CAN yet**. Each fix is packed into
the same 38-byte extended GPS record that YamuraNode_GPS sends on CAN id 0x50
(record 0x51 in the hub log) and printed to USB serial. Adding CAN later means
sending `gpsExData.dataBuffer` on 0x50, as YamuraNode_GPS does.

Build and upload:

```bash
platformio run -e teensy40 --target upload
```

## Wiring

| Teensy 4.0 | ZED-F9P breakout |
|---|---|
| 3.3V | 3V3 |
| GND | GND |
| 18 (SDA) | SDA |
| 19 (SCL) | SCL |

A Qwiic cable to the breakout's Qwiic connector does the same. The correction
radio's TX goes to the breakout's **RX2** (UART2), and its ground to GND.

LEDs: pin 22 is on with a 3D fix. Pin 13 is off with no RTK, blinks with RTK
float, and stays on with RTK fixed.

## USB serial output (115200)

With `DEBUG_VERBOSE` defined, each fix prints one tab-separated line in the same
format YamuraNode_Analyze uses for an extended GPS message, so GPS_Reader can read
either one:

```
00123456	0x50	GPS	22/11/2026 14:05:09	LAT	451234567	LONG	-931234567	COURSE	27012345	SPEED	15	SIV	22	TOW	345600100	HACC	14	FIX	3	RTK	2
```

The fields are millis, the CAN id, the date as DD/MM/YYYY and the UTC time, then:
LAT/LONG in degrees x 1e7, COURSE in degrees x 1e5, SPEED in mm/s, TOW in ms and HACC in mm.
FIX is 0 none, 2 2D, 3 3D, 4 GNSS+DR, 5 time only. RTK is 0 none, 1 float, 2 fixed.
Once a second a status line follows:

```
Status	rate	10 Hz	fix	3	RTK	fixed	SIV	22	hAcc	14 mm	USB RTCM	512 B/s
```

## F9P settings

Set at every boot (RAM and battery-backed RAM):

- I2C: UBX out; UBX + RTCM3 in
- UART2: RTCM3 in at 57,600 baud (`RTCM_UART2_BAUD`), nothing out
- GPS + Galileo + BeiDou; GLONASS, SBAS and QZSS off
- Automotive dynamic model, 10 Hz (`NAV_RATE_HZ`), auto NAV-PVT

The once-a-second status line shows the measured solution rate. If it stays
under 10 Hz in RTK, disable BeiDou as well.

## Corrections

- **In the car:** a radio on UART2. The F9P reads it directly; the Teensy is not involved.
- **On the bench:** send RTCM3 to the Teensy's USB serial port and it is pushed to
  the F9P over I2C. For example, with RTKLIB from an NTRIP caster (fill in your
  own caster, mount point and login, and the Teensy's COM port):

```bash
str2str -in ntrip://user:password@caster:2101/MOUNT -out serial://COM5:115200
```

Close the serial monitor first — only one program can hold the COM port.

## Correction radio (DX-SMART LoRa UART module, 915 MHz)

A pair of DX-SMART LoRa serial modules (SX1262, AT-command firmware V1.2.4)
carry the corrections: one at the base on a Raspberry Pi, one in the car on
the F9P's UART2. Each module sits on a small USB-serial adapter board wired
VCC, GND, TX, RX. M0, M1 and AUX are left unconnected.

The module is a transparent serial link: bytes into its RX go out over the air,
and received packets come out of its TX unchanged, so binary RTCM is fine. RTCM
has its own CRC-24 and the radio drops packets that fail its own CRC.

### Settings (both modules must match)

| Setting | Value | Command |
|---|---|---|
| Mode | 0, transparent | `AT+MODE0` (default) |
| Air rate | level 5, 18,230 bps | `AT+LEVEL5` |
| Serial | 57,600 baud, 8N1 (matches `RTCM_UART2_BAUD`) | `AT+BAUD6` |
| Frequency | 915.15 MHz (channel 41) | default |
| Power | 22 dBm | default |
| Address | ff,ff (broadcast) | default |

The first module is already set. To set the second, plug its adapter into a PC
and open the port at its current baud (factory default 9600) in a serial
terminal that sends CR+LF:

1. `+++` → `Entry AT` (command mode)
2. `AT+LEVEL5` → `+LEVEL=5 OK`
3. `AT+BAUD6` → `+BAUD=6 OK`
4. `AT+HELP` lists the settings; check `LEVEL:5 >> 18230bps`
5. `+++` → `Exit AT`, `Power on`. The module restarts at the new baud and keeps
   its settings.

`+++` toggles command mode at any time, so the base software must never send
`+++` followed by CR+LF (RTCM won't).

Air-rate levels (`AT+LEVELn`; a lower level reaches farther):

| Level | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| bps | 336 | 1,075 | 2,149 | 6,250 | 10,417 | 18,230 | 37,500 | 62,500 |

Baud codes (`AT+BAUDn`), as this firmware reports them: 3 = 9600, 6 = 57600,
7 = 115200.

### The 455-byte limit

Tested with both modules on a PC (2026-11-24): the module passes at most
**455 bytes per burst** and silently drops the rest, at every air rate
(levels 5, 6 and 7 gave identical results). Bursts of up to 450 bytes once a
second arrived 100% intact; 600-byte bursts lost 24% and 900-byte bursts 49%.
A base sends each epoch's RTCM as one burst, so **the base must send under
about 450 bytes per epoch**.

Splitting bursts also works, but needs a pacing script on the Pi (not written).
The module sends about 225 bytes per radio packet and drops anything that
arrives before it's ready for the next one. 900-byte epochs as four 225-byte
pieces were 100% intact with pieces at least 250 ms apart at level 5, 150 ms at
level 6 and 120 ms at level 7, and lost data at 200 / 120 / 100 ms. So the
full set with BeiDou (600-900 bytes) would need level 6 with about 200 ms
spacing; level 5 has no margin. The trimmed set below avoids that.

The correction budget (an MSM4 message is about 27 bytes with framing, plus
2.25 per satellite and 6 per signal, so about 15 bytes per two-signal
satellite):

| Message | Rate | Bytes |
|---|---|---|
| 1074 GPS MSM4 | 1 Hz | about 27 + 15 per satellite (12 satellites: about 200) |
| 1094 Galileo MSM4 | 1 Hz | about 27 + 15 per satellite (10 satellites: about 175) |
| 1005 base position | every 10 s | 25 |
| 1124 BeiDou MSM4 | **off** | would add about 150-200 |
| 1084 GLONASS, 1230 GLONASS biases | **off** | (GLONASS is off on the rover) |

That is about 300-400 bytes per epoch, up to about 450 with an unusually full
sky: it fits, but without much margin. The rover still tracks BeiDou; it just has no
corrections for it, so RTK uses GPS and Galileo. If fixes drop out under a
full sky, raise the base's elevation mask so it sends fewer low satellites.

Level 5 (18,230 bps) moves 450 bytes in about 0.2 s, well inside a second, and
keeps more range than levels 6 and 7, which don't help with the limit. If the
link drops out at the far end of the track, try level 4.

### Rover (in the car)

| Module | Connects to |
|---|---|
| TX | ZED-F9P breakout **RX2** |
| RX | not connected (the F9P sends nothing on UART2) |
| GND | GND (common with the Teensy and breakout) |
| VCC | 5 V supply / Teensy **VIN** (check the voltage marked on the board) |
| M0, M1, AUX | not connected |

Use the 5 V supply, not the Teensy's 3.3 V pin: that pin already feeds the
Teensy and the F9P, and the radio draws about 100 mA in bursts at 22 dBm. The
module's serial pins are 3.3 V logic, matching the F9P.

### Base (Raspberry Pi)

Plug the USB-serial adapter into a Pi USB port. It shows up as `/dev/ttyUSB0`
(Linux includes the CH340 driver). Send corrections with RTKLIB's `str2str`.
Either way, keep each epoch under 450 bytes (see *The 455-byte limit*).

**From a base F9P** plugged into the Pi by USB, set the base's RTCM output
(u-center: Configuration View → Generation 9 Advanced Configuration, or
`ubxtool -z KEY,VALUE`), saved to RAM and flash:

| Key | Value |
|---|---|
| CFG-RATE-MEAS | 1000 (1 Hz; message rates below count epochs) |
| CFG-MSGOUT-RTCM_3X_TYPE1074_USB | 1 |
| CFG-MSGOUT-RTCM_3X_TYPE1094_USB | 1 |
| CFG-MSGOUT-RTCM_3X_TYPE1005_USB | 10 |
| CFG-MSGOUT-RTCM_3X_TYPE1124_USB | 0 |
| CFG-MSGOUT-RTCM_3X_TYPE1084_USB | 0 |
| CFG-MSGOUT-RTCM_3X_TYPE1230_USB | 0 |
| CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB | 0 |
| CFG-MSGOUT-UBX_NAV_PVT_USB (and other UBX/NMEA on USB) | 0, so only RTCM reaches the radio |

The base also needs its position, from a survey-in or a fixed position
(CFG-TMODE-*). Then:

```bash
str2str -in serial://ttyACM0:115200 -out serial://ttyUSB0:57600
```

**From an NTRIP caster** the mount point decides what is sent, and most send
more than 450 bytes per epoch. Let `str2str` decode and re-encode just the
messages above (both ends need the `#rtcm3` format for it to convert):

```bash
str2str -in ntrip://user:password@caster:2101/MOUNT#rtcm3 \
        -out serial://ttyUSB0:57600#rtcm3 -msg "1074,1094,1005(10)"
```

This conversion has not been tested yet. Check that the rover gets RTK float or
fixed, and pick a mount point near the track (within about 10-20 km).

Or wire the module to the Pi's header UART instead of the adapter:

| Pi header pin | Pi signal | Module |
|---|---|---|
| 8 | GPIO14 / TXD | RX |
| 10 | GPIO15 / RXD | TX |
| 6 | GND | GND |
| 2 or 4 (5 V) | 5 V | VCC |

TX and RX cross. For the header UART: in `sudo raspi-config` → Interface Options
→ Serial Port, set login shell **No** and serial hardware **Yes**. On a Pi 3 / 4 /
Zero W add `dtoverlay=disable-bt` to `/boot/firmware/config.txt` (Pi 5:
`dtparam=uart0=on`), reboot, and use `serial0` (Pi 5: `ttyAMA0`) in place of
`ttyUSB0` above.

### Bench test

With the rover module on the F9P and the base module on a PC, run the same
`str2str` command with `serial://COMn:57600` as the output. The once-a-second
status line should go to RTK float, then fixed. GPS_Reader can compare this
stream with the uncorrected one.
