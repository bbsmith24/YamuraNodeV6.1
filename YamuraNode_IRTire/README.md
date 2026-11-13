# YamuraNode_IRTire — Teensy 4.0 port

PlatformIO project for the YamuraLog IR tire temperature node, converted from the
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
- Uses an MLX90621 IR array on I2C; the driver (`MLX90621.cpp/.h`) is in `src/`.
- Sends 60 temperature cells on CAN id 0x60 at 2 Hz.
