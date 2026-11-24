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

If a setting isn't acknowledged at boot, the F9P line names it, for example
`ZED-F9P setting failed: navigation rate`. The first boot after a constellation
change can report a few: the change restarts the GNSS and settings sent during
the restart time out. They are kept in battery-backed RAM, so the next boot is clean.

### Satellite diagnostics build

Uncomment `#define SAT_DIAGNOSTICS` in `src/main.cpp` (or build with
`PLATFORMIO_BUILD_FLAGS="-D SAT_DIAGNOSTICS"`) for a bench build that adds
satellites per constellation, used in the solution / tracked, to the status line:

```
Status	rate	1 Hz	fix	3	RTK	none	SIV	16	hAcc	4572 mm	USB RTCM	0 B/s	GPS	7/8	GLO	4/5	GAL	5/6	BDS	0/0	L1	16/18	L1 C/N0	20 dBHz	L2	12/14	L2 C/N0	19 dBHz	CORR	0
```

- `GPS` / `GLO` / `GAL` / `BDS`: satellites used in the solution / tracked (NAV-SAT).
- `L1`, `L2`: signals used / tracked (code locked) on each band (NAV-SIG). L2 is
  the second frequency: GPS L2C, GLONASS L2OF, Galileo E5b, BeiDou B2I. No L2
  signals means a single-band antenna or one that isn't receiving L2.
- `L1 C/N0`, `L2 C/N0`: mean signal strength of the tracked signals. An open sky
  with a good antenna gives about 40-50 dBHz; RTK needs mid-30s or better.
  Around 20 dBHz means the antenna is indoors or blocked, has no ground plane,
  isn't powered, or has a bad cable or connector.
- `CORR`: signals using RTCM corrections. It stays 0 until corrections arrive.

It runs at **1 Hz**, because the F9P only sends NAV-SAT at a 1 Hz solution rate
(tested: nothing at 5 or 10 Hz), so don't use it for logging laps. The extra
fields are name/value pairs like the rest, so GPS_Reader ignores them. Flashing
the normal build afterwards turns NAV-SAT off again and goes back to 10 Hz.

## F9P settings

Set at every boot (RAM and battery-backed RAM):

- I2C: UBX out; UBX + RTCM3 in
- UART2: RTCM3 in at 57,600 baud (`RTCM_UART2_BAUD`), nothing out
- GPS + GLONASS + Galileo; BeiDou, SBAS and QZSS off
- Automotive dynamic model, 10 Hz (`NAV_RATE_HZ`), auto NAV-PVT

GPS and GLONASS are the constellations with corrections (the MDOT CORS stream,
below); Galileo only adds satellites to the standalone solution. BeiDou is off
because nothing sends corrections for it, and three constellations keep the
solution rate up. The once-a-second status line shows the measured rate. If it
stays under 10 Hz in RTK, disable Galileo as well.

## Corrections

- **In the car:** a radio on UART2. The F9P reads it directly; the Teensy is not involved.
- **On the bench:** send RTCM3 to the Teensy's USB serial port and it is pushed to
  the F9P over I2C, or to the base radio's COM port to test the whole radio link.

### Bench corrections from Windows: `tools/ntrip2serial.py`

RTKLIB-EX's Windows build has no `str2str.exe` (only the `strsvr.exe` GUI), so
`tools/ntrip2serial.py` does the same job. It needs Python 3 and pyserial:
- it logs in to the caster (MDOT by default) and sends a GGA position;
- it writes the RTCM to a serial port;
- it prints each epoch's size and message types, and flags epochs over the
  radio's 450-byte limit.

`--lat` and `--lon` are required: they are the base's position, sent to the
caster as a GGA line on connect and every 10 s. `NEAREST_RTCM3-GG` picks the
station from it, so at the track use the track's position, not home's.

The password comes only from the `NTRIP_PASSWORD` environment variable:

```powershell
$env:NTRIP_USER = 'name'; $env:NTRIP_PASSWORD = 'secret'
# report only - check the stream and its epoch sizes
python tools/ntrip2serial.py --mount NEAREST_RTCM3-GG --lat LAT --lon LON --height HEIGHT
# through the base radio (rover radio on the F9P's RX2)
python tools/ntrip2serial.py --mount NEAREST_RTCM3-GG --lat LAT --lon LON --height HEIGHT --serial COM20
# straight into the Teensy's USB port (no radio)
python tools/ntrip2serial.py --mount NEAREST_RTCM3-GG --lat LAT --lon LON --height HEIGHT --serial COM17 --baud 115200
```

Other options: `--host`, `--port`, `--gga-interval` (default 10 s), `--limit`
(default 450), `--quiet` (only over-limit epochs and a summary each minute),
`--status` (print the rover's `Status` lines read back from `--serial`, for the
Teensy's USB port: the script holds the port, so nothing else can read it), and
`--log FILE` (also append all output to a file). It
reconnects if the stream drops. Ctrl+C prints a summary: epochs, largest
epoch, how many were over the limit, and the count of each message type.

On the Pi, `sudo apt install rtklib` gives the real `str2str` (commands under
*Base*). Only one program can hold a COM port, so close the serial monitor or
GPS_Reader on that port first.

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
level 6 and 120 ms at level 7, and lost data at 200 / 120 / 100 ms. So an
epoch of 600-900 bytes would need level 6 with about 200 ms spacing; level 5
has no margin. Keeping each epoch under 450 bytes avoids that.

The correction set is **GPS + GLONASS**, which is what the MDOT CORS
subscription (RTCM 3.1) provides. Approximate sizes per epoch:

| Message | Rate | Bytes |
|---|---|---|
| 1004 GPS L1/L2 (RTCM 3.1) | 1 Hz | about 14 + 16 per satellite (12 satellites: about 200) |
| 1012 GLONASS L1/L2 (RTCM 3.1) | 1 Hz | about 14 + 16 per satellite (8 satellites: about 145) |
| 1005 or 1006 base position | every 10 s or so | 25-27 |
| 1007 / 1008 antenna, 1033 receiver and antenna | if the caster sends them | 20-80 each |
| Galileo, BeiDou | none | (not in the MDOT subscription) |

That is about 350 bytes per epoch, so it fits, but extra descriptor messages
sent in the same epoch can push it toward the limit. The F9P doesn't need
1007 / 1008 / 1033, so `str2str` can drop them (see *Base*). The MSM
equivalents from a base F9P (1074 GPS, 1084 GLONASS, 1230 GLONASS biases) are
about the same size. If fixes drop out under a full sky, send fewer satellites
(a higher elevation mask on a base F9P) or the position message less often.

Level 5 (18,230 bps) moves 450 bytes in about 0.2 s, well inside a second, and
keeps more range than levels 6 and 7, which don't help with the limit. If the
link drops out at the far end of the track, try level 4.

### Rover (in the car)

The radio feeds the SparkFun GPS-RTK-SMA breakout's UART2, which the firmware
sets to RTCM3 in at 57,600 baud on every boot. The Teensy-to-breakout wiring
(I2C or Qwiic, see *Wiring*) doesn't change.

| Radio module | Connects to | Notes |
|---|---|---|
| TX | breakout **RX2** | corrections into the F9P's UART2 |
| RX | not connected | the F9P's TX2 sends nothing |
| GND | breakout **GND** | common ground with the Teensy and breakout |
| VCC | Teensy **VIN** (5 V) | not the Teensy's 3.3 V pin |
| M0, M1, AUX | not connected | as on the bench, where the link was tested |

- **Power:** the Teensy's 3.3 V pin already feeds the Teensy and the F9P, and the
  radio draws about 100 mA in bursts at 22 dBm. The breakout's own 5V pin is
  only live when its USB-C is plugged in, so don't use it. On the bench the
  module ran from its USB adapter's VCC; check the voltage marked on the
  module. If it's 3.3 V only, give it its own 3.3 V regulator.
- **Logic levels:** the radio's TX is 3.3 V logic, matching RX2. The F9P's
  inputs are not 5 V tolerant.
- **Adapter:** take the module off its USB adapter in the car, or at least
  leave the adapter's USB unplugged, so the CH340 doesn't drive the TX line too.
- **Antennas:** keep the LoRa antenna at least 30 cm from the GNSS antenna if
  you can. 22 dBm at 915 MHz that close can desensitize the GNSS receiver.

First power-up check: with the base sending, the rover radio's TX shows
activity once a second, and the status line goes from `RTK none` to `float`,
then `fixed`. The pin 13 LED goes from off to blinking (float) to on (fixed).

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
| CFG-MSGOUT-RTCM_3X_TYPE1084_USB | 1 |
| CFG-MSGOUT-RTCM_3X_TYPE1230_USB | 10 |
| CFG-MSGOUT-RTCM_3X_TYPE1005_USB | 10 |
| CFG-MSGOUT-RTCM_3X_TYPE1094_USB | 0 |
| CFG-MSGOUT-RTCM_3X_TYPE1124_USB | 0 |
| CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB | 0 |
| CFG-MSGOUT-UBX_NAV_PVT_USB (and other UBX/NMEA on USB) | 0, so only RTCM reaches the radio |

The base also needs its position, from a survey-in or a fixed position
(CFG-TMODE-*). Then:

```bash
str2str -in serial://ttyACM0:115200 -out serial://ttyUSB0:57600
```

**From the MDOT CORS network (MSRN)**, the planned source: the subscription
(2026-11-24) is "GPS and GLONASS Corrections", RTCM 3.1, unlimited. Details from
MDOT's port list (`Documents/MDOT connection info.pdf`, effective 04/15/2021) and
the caster's sourcetable (checked 2026-11-24):

- **Caster:** `148.149.0.87` (Leica GNSS Spider). This is not the website's
  host, which only answers on port 443.
- **Login:** the MDOT CORS portal username and password; both are case sensitive.
- **Coordinates:** RTK positions come out in **NAD83(2011)**, not WGS84, about
  1-1.5 m apart in Michigan. Lap timing against a track map made from the same
  corrections is unaffected; the satellite imagery (WGS84) will look shifted by
  about that much.

| Port | Mount point | What it is | Needs GGA |
|---|---|---|---|
| **10001** | **`NEAREST_RTCM3-GG`** | nearest single station, GPS + GLONASS, RTCM 3 | yes |
| 10001 | `<SITE>_RTCM3-GG` (121 stations, e.g. `MILI_RTCM3-GG` Livonia, `ANNARBOR_RTCM3-GG`) | one named station, GPS + GLONASS | no |
| 10000 | `NETWORK_SOLUTION_RTCM3-GG` | network solution (AMC), GPS + GLONASS | yes |
| 10011 | `NEAREST_MSM4`, `<CODE>-MSM4` | MSM4, GPS + GLONASS + Galileo + BeiDou; may not be in this subscription | nearest: yes |

**Use port 10001.** A single-station stream sends only that station's
observations, so it stays under the radio's 450-byte limit. A network
solution adds network correction messages and would likely go over it.
`NEAREST_RTCM3-GG` picks the station for you from the GGA position. Or name
the station closest to the track, ideally within about 20 km (the sourcetable
lists each station's position: `curl http://148.149.0.87:10001/`). A named
station doesn't need GGA, but MDOT's notes say all products expect it, so send
it anyway.

`str2str` sends the GGA from a fixed position with `-p` (latitude, longitude,
ellipsoidal height) every `-n` seconds:

```bash
str2str -in ntrip://USER:PASSWORD@148.149.0.87:10001/NEAREST_RTCM3-GG \
        -out serial://ttyUSB0:57600 -p LAT LON HEIGHT -n 10
```

If the epochs
come out over 450 bytes (for example because of 1007 / 1008 / 1033 descriptor
messages), let `str2str` decode and re-encode just what the F9P needs; both
ends need the `#rtcm3` format for it to convert:

```bash
str2str -in ntrip://USER:PASSWORD@148.149.0.87:10001/NEAREST_RTCM3-GG#rtcm3 \
        -out serial://ttyUSB0:57600#rtcm3 -msg "1004,1012,1006(10)" -p LAT LON HEIGHT -n 10
```

None of this has been tested yet. Check that the rover gets RTK float or fixed.

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
