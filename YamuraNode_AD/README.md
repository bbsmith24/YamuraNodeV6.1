# YamuraNode_AD — Teensy 4.0 port

This workspace contains a Teensy 4.0-ready PlatformIO project for the YamuraLog A2D+Digital node.

Quick start (PlatformIO):

1. Install PlatformIO (VS Code extension) or use `platformio` CLI.
2. From this folder open the project in VS Code.
3. Build and upload to a connected Teensy 4.0:

```bash
platformio run -e teensy40 --target upload
```

Notes:
- The project depends on the `FlexCAN_T4` library by tonton81 (declared in `platformio.ini`).
- Alternatively, you can use the Arduino + Teensyduino toolchain; copy `src/main.cpp` into an Arduino sketch and install `FlexCAN_T4` in the Arduino IDE.
