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
