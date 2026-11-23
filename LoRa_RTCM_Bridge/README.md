# LoRa_RTCM_Bridge — DX-LR30 / DX-LR32 LoRa dev board

PlatformIO firmware for the DX-SMART LoRa dev board (STM32F103C8T6 driving a
DX-LR30-900M22S SX1262 module). It turns a pair of boards into a transparent
serial link for RTCM3 corrections:

- **base:** Raspberry Pi (`str2str`) → board serial → LoRa
- **rover:** LoRa → board serial TX → ZED-F9P **RX2** (YamuraNode_GPSRTK)

Both boards run the same firmware. Bytes in on the serial port go out over the
air, and received packets go out on the serial port byte for byte. It replaces
the vendor demo, which treats received data as text and can't carry RTCM.
Wiring and the Pi setup are in `../YamuraNode_GPSRTK/README.md`.

## Settings (top of `src/main.cpp`; both boards must match)

| Setting | Value |
|---|---|
| Serial (USART1, PA9 TX / PA10 RX, also the USB-C CH340) | 57,600 baud, 8N1 |
| Frequency | 915.0 MHz (US 902–928 MHz band) |
| LoRa | SF7, BW 500 kHz, CR 4/5, sync word 0x12, preamble 8, CRC on |
| Power | 22 dBm |
| Packets | up to 250 data bytes + 2-byte header (magic 0xB5, sequence number) |

SF7 / BW500 is about 21.9 kbps on air; a full packet takes about 100 ms. A packet
is sent when 250 bytes are buffered or the serial line has been idle for 2 ms.
Packets with a bad CRC or the wrong header are dropped.

LED (PC13): three blinks at power-up = radio OK; a short blink per packet
sent or received; fast blinking forever = the radio didn't start.

## Build

```bash
platformio run
```

This writes `.pio/build/dxlr30/firmware.hex` (and `.bin`).

## Flash

The board is programmed through its USB-C port (CH340 + the STM32's serial
boot loader) with the vendor's **mcuisp** tool, in
`YamuraNode_GPSRTK/Documents/.../05 Testing Tools/STM32F103 burning tool/mcuisp.exe`:

1. Install the CH340 driver if Windows doesn't show a COM port for the board.
2. In mcuisp: **STM ISP** tab, pick the board's COM port, load `firmware.hex`,
   tick **Verify** and **Run After ISP complete**.
3. At the bottom, choose **Reset@DTR Low(<-3V), ISP@RTS High**.
4. **Start ISP**. Repeat for the second board.

## Not tested yet

Built with PlatformIO (STM32duino + RadioLib 7); not yet run on the boards.
The vendor demo sets the module's crystal trim (XTA 0x04, XTB 0x2F); this
firmware leaves the default, which BW 500 kHz should tolerate. If the boards
can't hear each other, that and the RF switch pins (RXEN PA1, TXEN PA0) are the
first things to check.
