# YamuraNode_Accel — Teensy 4.0 port

PlatformIO project for the YamuraLog IMU (accel + gyro) node, converted from the
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
- Uses an MPU6050 IMU on I2C (Adafruit MPU6050 library). The LSM9DS1 code path is kept
  behind `#define LSM9DS1`; to use it, add `sparkfun/SparkFun LSM9DS1 IMU` to `lib_deps`.
- Sends accel (g) + gyro (deg/s) as one 28-byte message on CAN id 0x40 at 100 Hz.
