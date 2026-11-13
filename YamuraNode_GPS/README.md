# YamuraNode_GPS — Teensy 4.0 port

PlatformIO project for the YamuraLog GPS node, converted from the
Arduino sketch in YamuraLogV6.

Quick start (PlatformIO):

1. Install PlatformIO (VS Code extension) or use `platformio` CLI.
2. From this folder open the project in VS Code.
3. Build and upload to a connected Teensy 4.0:

```bash
platformio run -e teensy40 --target upload
```

Notes:
- Libraries are declared in `platformio.ini` (`FlexCAN_T4` by tonton81, plus any sensor libraries).
- Shared message structures come from `../YamuraLog_Common/src/YamuraLog_Structures.h`.
- Uses a u-blox GNSS receiver on I2C (SparkFun u-blox GNSS Arduino Library v2).
- Sends position, course, ground speed and SIV on CAN id 0x50 at 10 Hz.
