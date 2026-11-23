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

## Correction radio (DX-LR32 / DX-LR30-900 LoRa kit)

The plan is a pair of DX-SMART LoRa dev boards: one at the base on a Raspberry
Pi, one in the car on the F9P's UART2. Vendor documents are in `Documents/`
(DX-LR30-900M22SP board manual, DX-LR30-900M22S module manual, demo source).

Each board is a **DX-LR30-900M22S module** (SX1262, SPI only, 850-930 MHz,
3.3 V only, max 3.7 V) driven by an **STM32F103C8T6**. The board's serial port
is the STM32's USART1, also brought out to a CH340 USB-serial chip on the
Type-C connector. Whatever the STM32 firmware does decides how serial data
goes over the air. See *Radio firmware* below: the stock demo can't carry RTCM.

Board header pins used (numbers from the board manual, table 2):

| Board pin | Signal |
|---|---|
| 29 | PA9 = USART1 **TX** (data out of the board) |
| 30 | PA10 = USART1 **RX** (data into the board) |
| 18 | GND |
| 21 | +5 V in (on-board regulator makes the module's 3.3 V) |
| 1, 19, 20, 22 | 3.3 V |

The serial pins are 3.3 V logic; the module manual warns 5 V TTL can damage it.
The vendor suggests a 200 ohm series resistor on TX/RX from an external MCU.

### Rover (in the car)

| Board pin | Connects to |
|---|---|
| 29 (PA9, TX) | ZED-F9P breakout **RX2** |
| 30 (PA10, RX) | not connected (the F9P sends nothing on UART2) |
| 18 (GND) | GND (common with the Teensy and breakout) |
| 21 (+5 V) | Teensy **VIN** / the 5 V supply |

Power it from 5 V, not the Teensy's 3.3 V pin: that pin already feeds the
Teensy and the F9P, and the radio draws about 100 mA in bursts at 22 dBm.
Leave the Type-C port unplugged in the car (the CH340 shares the RX/TX lines).

### Base (Raspberry Pi)

Simplest: plug the board's **Type-C port into a Pi USB port**. It powers the
board and the CH340 shows up as `/dev/ttyUSB0` (install nothing; Linux has the
CH340 driver).

Or wire it to the Pi's header UART:

| Pi header pin | Pi signal | Board pin |
|---|---|---|
| 8 | GPIO14 / TXD | 30 (PA10, RX) |
| 10 | GPIO15 / RXD | 29 (PA9, TX) |
| 6 | GND | 18 (GND) |
| 2 or 4 (5 V) | 5 V | 21 (+5 V) |

TX and RX cross. Both sides are 3.3 V logic, so no level shifter is needed.

Pi setup for the header UART:

1. `sudo raspi-config` → Interface Options → Serial Port: login shell over
   serial **No**, serial port hardware **Yes**.
2. Pi 3 / 4 / Zero W: add `dtoverlay=disable-bt` to `/boot/firmware/config.txt`
   so GPIO14/15 get the full hardware UART, not the less reliable mini UART.
   Pi 5: add `dtparam=uart0=on` instead.
3. Reboot. The radio is on `/dev/serial0` (Pi 5: `/dev/ttyAMA0`).

Send corrections with RTKLIB's `str2str`, either from an NTRIP caster or from a
base F9P plugged into the Pi by USB (use `ttyUSB0` instead of `serial0` when the
board is on USB):

```bash
str2str -in ntrip://user:password@caster:2101/MOUNT -out serial://serial0:57600
str2str -in serial://ttyACM0:115200 -out serial://serial0:57600
```

### Radio firmware

The stock demo firmware on the STM32 (`Documents/.../07 Programming code
demonstration`, `LR_driver/UserConfig.c`) is a serial-to-LoRa test, not a
transparent modem, and **won't carry RTCM as shipped**:

- The receiving board prints each packet with `printf("%s", payload)`, which
  stops at the first zero byte. RTCM is binary and full of zeros.
- LoRa is set to SF9 / BW125 / CR4-6, about 1.5 kbps over the air, with packet
  CRC off.
- USART1 is fixed at 9600 baud (960 bytes/s).
- 915 MHz, 22 dBm, up to 255 bytes per packet (sent when 255 bytes arrive or the
  serial line goes idle).

Corrections for GPS + Galileo + BeiDou (MSM4 at 1 Hz plus the base position)
are roughly 600–900 bytes/s, about 5–7 kbps. So both boards need the
replacement firmware in `../LoRa_RTCM_Bridge`: binary-safe, CRC on,
SF7 / BW500 (about 21.9 kbps), serial at 57,600 to match `RTCM_UART2_BAUD`.
Its README covers building and flashing with mcuisp over the Type-C port.
If the link still can't keep up, send fewer messages from the base: MSM4 at
1 Hz and the base position (1005) every 10 s.
