# YamuraNode_Analyze — Teensy 4.0 port

PlatformIO project for the YamuraLog CAN bus analyzer, converted from the CAN_Sniffer Arduino
sketch in YamuraLogV6. It listens to every YamuraLog CAN id and prints the message types you
pick from a serial menu.

Quick start (PlatformIO):

1. Install PlatformIO (VS Code extension) or use `platformio` CLI.
2. From this folder open the project in VS Code.
3. Build and upload to a connected Teensy 4.0, then open the serial monitor (115200):

```bash
platformio run -e teensy40 --target upload
```

```bash
platformio device monitor
```

Serial menu: H hub, C control, D A/D, I IMU, G GPS, T IR tire, W wheel speed, S shock, R RPM,
X CAN, A all, B send a heartbeat, M menu (all off).

Notes:
- `FlexCAN_T4` is declared in `platformio.ini`; `TimeLib` and `USBHost_t36` come with the Teensy platform.
- Shared message structures come from `../YamuraLog_Common/src/YamuraLog_Structures.h`.
- IMU messages show gyro rates too when the node sends the 28-byte accel + gyro message.
