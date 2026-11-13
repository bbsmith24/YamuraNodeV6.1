# YamuraNode_Speed — Teensy 4.0 port

PlatformIO project for the YamuraLog wheel speed node, converted from the
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
- Wheel speed sensors on pins 5-8 (rising edge interrupts); no add-on devices.
- CAN ids 0x80-0x8F, selected in bands of 4 by jumpers on pins 1-3.
