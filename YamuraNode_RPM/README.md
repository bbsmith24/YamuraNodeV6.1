# YamuraNode_RPM — Teensy 4.0 engine RPM node

PlatformIO project for the YamuraLog engine RPM node. It reads the engine's
tach signal through an optocoupler, sends RPM on CAN id **0x90**, and drives
an LED shift/redline light.

Build and upload:

```bash
platformio run -e teensy40 --target upload
```

## Wiring

| Teensy 4.0 | Connection |
|---|---|
| 5 | H11L1 output (pin 4), 4.7 kΩ pull-up to 3.3 V |
| 9 | Shift light, through a transistor (logic-level MOSFET or NPN) |
| 13 | Status LED (on while tach pulses arrive) |
| CAN3 | CAN bus, same as the other nodes |

Tach input side of the H11L1:

- Tach wire → 4.7 kΩ → H11L1 pin 1 (anode); pin 2 (cathode) → car ground.
- 1N4148 across pins 1–2, reversed, to protect the LED.
- H11L1 pin 6 (Vcc) to 3.3 V, pin 5 to ground.

The H11L1 output goes low while the tach signal is high, so the node counts
falling edges.

## Settings (`src/main.cpp`)

| Define | Default | Meaning |
|---|---|---|
| `PULSES_PER_REV` | 2 | Tach pulses per crank revolution (4 cylinder) |
| `SHIFT_RPM` | 6300 | Shift light on |
| `SHIFT_HYST` | 150 | Light off below `SHIFT_RPM - SHIFT_HYST` |
| `REDLINE_RPM` | 6800 | Shift light flashes |
| `MAX_RPM` | 12000 | Faster pulses are rejected as noise |
| `STOPPED_MS` | 500 | No pulse for this long reads 0 rpm |
| `AVG_PULSES` | 4 | Periods averaged per reading |
| `SAMPLE_RATE` | `SAMPLE_50Hz` | CAN send rate |

Check `PULSES_PER_REV` against the dash tach at a steady 3,000 rpm. Uncomment
`VERBOSE` to print each reading to USB serial.

The shift light lights for half a second at power-up as a lamp test.

## CAN record (0x90, 12 bytes)

`RPM_Data` in `YamuraLog_Common/src/YamuraLog_Structures.h`:

| Field | Type | Meaning |
|---|---|---|
| timestamp | uint32 | Hub time (ms) of the latest tach pulse |
| periodUs | uint32 | Averaged time between pulses (µs), 0 when stopped |
| rpm | uint16 | Engine rpm |
| shiftState | uint8 | 0 off, 1 shift, 2 redline |
| reserved | uint8 | 0 |
