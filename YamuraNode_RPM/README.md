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

## Optocoupler hardware

The tach wire swings 0–12 V (up to about 14.5 V with the engine running) and
carries ignition noise. The Teensy 4.0 is a 3.3 V part and its pins are **not**
5 V tolerant, so the tach signal never goes to a Teensy pin directly. An
optocoupler turns it into a clean 3.3 V logic signal and keeps spikes on the
tach wire away from the Teensy.

### Part: H11L1 (6-pin DIP)

The H11L1 has a Schmitt-trigger output, so slow or noisy tach edges still give
one clean edge per pulse. It runs from 3.3 V (Vcc 3–15 V) and switches with
1.6 mA or less through its LED.

| Pin | Function | Connection |
|---|---|---|
| 1 | LED anode | Tach wire through R1 |
| 2 | LED cathode | Car ground |
| 3 | — | No connection |
| 4 | Output (open collector) | Teensy pin 5, R2 pull-up to 3.3 V |
| 5 | Ground | Teensy ground |
| 6 | Vcc | 3.3 V, C1 to ground |

```
 tach ──[F1 0.5A]──[R1 4.7k]──┬────── 1 ┌───────┐ 6 ──┬── 3.3V
                              │         │ H11L1 │     C1 0.1uF
                         D1 ▲ │         │       │ 5 ──┴── GND
                    1N4148  │ │         │       │
 car GND ───────────────────┴─┴────── 2 └───────┘ 4 ──┬── Teensy pin 5
                                                      R2 4.7k
                                                      └── 3.3V
```

### Parts list

| Ref | Part | Notes |
|---|---|---|
| U1 | H11L1 | Schmitt-trigger optocoupler, 6-pin DIP or SMD |
| R1 | 4.7 kΩ, 1/4 W | LED current limit (see below) |
| D1 | 1N4148 | Across the LED, reversed; protects it from negative spikes |
| R2 | 4.7 kΩ | Output pull-up to 3.3 V (stronger than the Teensy's internal pull-up) |
| C1 | 0.1 µF ceramic | Vcc bypass, close to pin 6 |
| F1 | 0.5–1 A inline fuse | At the splice into the tach wire |
| D2 (optional) | SMBJ15A TVS | Tach input to car ground, if the tap picks up large spikes |

### R1 value

R1 = (V_tach − V_LED) / I_LED. With V_LED about 1.2 V and 2–3 mA wanted
(comfortably above the H11L1's 1.6 mA turn-on):

- 12.0 V: (12 − 1.2) / 4.7 kΩ = 2.3 mA
- 14.5 V: (14.5 − 1.2) / 4.7 kΩ = 2.8 mA, about 40 mW in R1

### Don't load down the tach wire

The tach output is usually an open-collector transistor in the ECU, with the
pull-up to 12 V inside the gauge cluster. The opto input draws its 2–3 mA from
that pull-up. If the pull-up is weak, the high level sags: the dash tach may
misread and the opto may not switch.

Check before trusting it:

1. With the engine idling, measure the tach wire on a scope (or DC volts)
   before connecting the opto, then again with it connected.
2. If the high level drops by more than a volt or two, or the dash tach acts
   up, buffer the signal instead of driving the opto LED from the tach wire.
   A 2N7000 with its gate on the tach wire through 100 kΩ (with a 12 V zener
   gate to ground) can switch the opto LED from switched 12 V through R1, so
   the tach wire sees almost no load. That buffer inverts the signal, so
   change the interrupt in `setup()` to `RISING`.

### Signal polarity

The H11L1 output goes low while its LED is on, which is while the tach wire is
high. The node counts falling edges on pin 5. Polarity doesn't change the RPM
reading, only which edge of the pulse is timed.

### Isolation

The node is powered from the car through the CAN bus, so its ground is the car
ground anyway. The opto isn't there for safety isolation; it keeps tach-wire
voltage, spikes and ringing out of the Teensy, and gives a clean edge.

### Alternatives

- **PC817** (4-pin, no Schmitt trigger): works at these frequencies (under
  300 Hz) but its output edges are slow. Use R2 of 10 kΩ and rely on the
  firmware's noise rejection (`MAX_RPM`).
- **6N137** (high-speed digital): needs 5 V, and its output has to be divided
  or level shifted for the Teensy. Not needed at tach frequencies.
- **NPN transistor + divider + clamp diode:** cheapest, but no isolation from
  spikes and less noise immunity.

### Checking it on the bench

1. Power the node from USB, feed pin 1 from a 12 V supply through R1, and
   touch the wire to 12 V and off: pin 5 should go low and high.
2. In the car, with `VERBOSE` on, idle should read about 700–900 rpm and the
   periods should be steady (about 37 ms at 800 rpm with 2 pulses/rev).
3. Hold 3,000 rpm on the dash tach and confirm the node agrees, which checks
   `PULSES_PER_REV`.

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
