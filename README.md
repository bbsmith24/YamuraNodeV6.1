# YamuraNodeV6.1
Visual Studio Code projects for YamuraLog nodes

Repository: https://github.com/bbsmith24/YamuraNodeV6.1

All nodes are PlatformIO projects for the Teensy 4.0:

| Project | Node | CAN ids |
|---|---|---|
| YamuraNode_AD | A2D + digital inputs | 0x30-0x3F |
| YamuraNode_Accel | IMU (accel + gyro) | 0x40 |
| YamuraNode_GPS | u-blox GPS | 0x50 |
| YamuraNode_GPSRTK | u-blox ZED-F9P RTK GPS (no CAN yet) | 0x50 (planned) |
| YamuraNode_IRTire | MLX90621 IR tire temperature | 0x60-0x6F |
| YamuraNode_Speed | Wheel speed | 0x80-0x8F |
| YamuraNode_RPM | Engine RPM (tach via optocoupler) + shift light | 0x90 |
| YamuraNode_Analyze | CAN bus analyzer (serial monitor of all node messages) | listens to all |

YamuraLog_Common holds the message structures shared by all nodes.
