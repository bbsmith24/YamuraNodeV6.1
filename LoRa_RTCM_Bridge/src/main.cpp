// YamuraLog LoRa RTCM bridge
// DX-SMART DX-LR30-900M22SP / DX-LR32 dev board: STM32F103C8T6 + DX-LR30-900M22S (SX1262)
//
// Brian B. Smith
// brianbsmith.com
// November 2026
// Part of the YamuraLog project
//
// A transparent serial <-> LoRa bridge for RTCM3 corrections:
//   base:  Raspberry Pi (str2str) -> board serial -> LoRa
//   rover: LoRa -> board serial TX -> ZED-F9P UART2 (RX2)
// Both boards run this same firmware. Bytes in on the serial port go out over
// the air; packets received over the air go out on the serial port, byte for
// byte (RTCM is binary, so nothing here treats it as text).
//
// The serial port is the only data path, so there is no debug output: the
// board's PC13 LED shows activity instead (see UpdateLed).
//
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// ---------------------------------------------------------------- settings
// Both boards must use the same radio settings.
#define LINK_BAUD    57600   // serial; must match the rover F9P's UART2 (RTCM_UART2_BAUD)
#define FREQ_MHZ     915.0   // US 902-928 MHz ISM band
#define BW_KHZ       500.0   // 500 kHz: fastest; the module needs >= 125 kHz (no TCXO)
#define SPREAD       7       // SF7
#define CODING       5       // 4/5
#define SYNC_WORD    0x12    // private LoRa network
#define POWER_DBM    22      // module maximum
#define PREAMBLE     8
// SF7 / BW500 / CR4-5 is about 21.9 kbps on air; a 252-byte packet takes about 100 ms.
// GPS + Galileo + BeiDou MSM4 at 1 Hz is roughly 600-900 bytes/s.

#define PAYLOAD_MAX  250     // data bytes per packet (plus a 2-byte header)
#define IDLE_US      2000    // send what is buffered once the serial line is idle this long
#define TX_TIMEOUT_MS 1000   // give up on a transmit that never signals done

// ---------------------------------------------------------------- board pins
// from the vendor demo (LR_driver/UserConfig.h)
#define PIN_NSS   PA4
#define PIN_NRST  PA3
#define PIN_BUSY  PA2
#define PIN_DIO1  PC15
#define PIN_RXEN  PA1   // RF switch: receive
#define PIN_TXEN  PA0   // RF switch: transmit
#define PIN_LED   PC13  // user LED
// SPI1 on PA5 (SCK), PA6 (MISO), PA7 (MOSI); serial USART1 on PA9 (TX), PA10 (RX),
// also wired to the CH340 USB-serial chip on the Type-C port

// packet header: magic byte, then a sequence number (lets a receiver count lost packets)
#define PKT_MAGIC 0xB5

// USART1 (PA9 TX, PA10 RX) - this core's Serial1
#define Link Serial1
SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_NRST, PIN_BUSY);

// serial -> air buffer
static uint8_t ring[4096];
static uint16_t ringHead = 0;   // next write
static uint16_t ringTail = 0;   // next read
static uint32_t lastByteUs = 0;
static uint32_t droppedBytes = 0;

// radio state
static volatile bool dio1Fired = false;
static bool transmitting = false;
static uint32_t txStartMs = 0;
static uint8_t txSeq = 0;
static uint8_t packet[PAYLOAD_MAX + 2];

// counters (visible with a debugger)
static uint32_t packetsSent = 0;
static uint32_t packetsReceived = 0;
static uint32_t packetsBad = 0;
static uint32_t packetsLost = 0;
static int16_t lastRxSeq = -1;

static uint32_t ledOffMs = 0;

// forward declarations
void Dio1Isr();
void StartReceive();
void SendPacket();
void HandleReceived();
void UpdateLed();
void Blink(uint32_t ms);
void FailForever();
uint16_t RingCount();

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);   // PC13 LED is active low: off

  Link.begin(LINK_BAUD);

  radio.setRfSwitchPins(PIN_RXEN, PIN_TXEN);
  // tcxoVoltage 0: the module has a plain crystal, no TCXO; DC-DC regulator
  int state = radio.begin(FREQ_MHZ, BW_KHZ, SPREAD, CODING, SYNC_WORD, POWER_DBM, PREAMBLE, 0.0, false);
  if (state != RADIOLIB_ERR_NONE)
  {
    FailForever();
  }
  radio.setCRC(2);               // LoRa payload CRC on; bad packets are dropped
  radio.setDio1Action(Dio1Isr);
  StartReceive();

  // three short blinks: radio up
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(PIN_LED, LOW);
    delay(80);
    digitalWrite(PIN_LED, HIGH);
    delay(120);
  }
}

void loop()
{
  // serial in -> ring buffer
  while (Link.available() > 0)
  {
    uint8_t b = (uint8_t)Link.read();
    uint16_t next = (ringHead + 1) % sizeof(ring);
    if (next == ringTail)
    {
      droppedBytes++;            // full: the air link can't keep up
    }
    else
    {
      ring[ringHead] = b;
      ringHead = next;
    }
    lastByteUs = micros();
  }

  // radio finished sending, or a packet arrived
  if (dio1Fired)
  {
    dio1Fired = false;
    if (transmitting)
    {
      radio.finishTransmit();
      transmitting = false;
      StartReceive();
    }
    else
    {
      HandleReceived();
    }
  }
  if (transmitting && millis() - txStartMs > TX_TIMEOUT_MS)
  {
    radio.finishTransmit();
    transmitting = false;
    StartReceive();
  }

  // send a full packet, or whatever is buffered once the line goes quiet
  uint16_t count = RingCount();
  if (!transmitting && count > 0 &&
      (count >= PAYLOAD_MAX || micros() - lastByteUs > IDLE_US))
  {
    SendPacket();
  }

  UpdateLed();
}

void Dio1Isr()
{
  dio1Fired = true;
}

void StartReceive()
{
  radio.startReceive();
}

uint16_t RingCount()
{
  return (uint16_t)((ringHead + sizeof(ring) - ringTail) % sizeof(ring));
}

void SendPacket()
{
  uint16_t count = RingCount();
  if (count > PAYLOAD_MAX)
  {
    count = PAYLOAD_MAX;
  }
  packet[0] = PKT_MAGIC;
  packet[1] = txSeq++;
  for (uint16_t i = 0; i < count; i++)
  {
    packet[2 + i] = ring[ringTail];
    ringTail = (ringTail + 1) % sizeof(ring);
  }
  if (radio.startTransmit(packet, count + 2) == RADIOLIB_ERR_NONE)
  {
    transmitting = true;
    txStartMs = millis();
    packetsSent++;
    Blink(20);
  }
  else
  {
    StartReceive();
  }
}

void HandleReceived()
{
  size_t len = radio.getPacketLength();
  int state = RADIOLIB_ERR_UNKNOWN;
  if (len > 0 && len <= sizeof(packet))
  {
    state = radio.readData(packet, len);
  }
  if (state == RADIOLIB_ERR_NONE && len > 2 && packet[0] == PKT_MAGIC)
  {
    if (lastRxSeq >= 0)
    {
      packetsLost += (uint8_t)(packet[1] - (uint8_t)lastRxSeq - 1);
    }
    lastRxSeq = packet[1];
    Link.write(&packet[2], len - 2);
    packetsReceived++;
    Blink(20);
  }
  else
  {
    packetsBad++;                // CRC error, someone else's packet, or junk
  }
  StartReceive();
}

// LED: a short blink per packet sent or received; off when idle
void Blink(uint32_t ms)
{
  digitalWrite(PIN_LED, LOW);
  ledOffMs = millis() + ms;
}

void UpdateLed()
{
  if (ledOffMs != 0 && (int32_t)(millis() - ledOffMs) >= 0)
  {
    digitalWrite(PIN_LED, HIGH);
    ledOffMs = 0;
  }
}

// radio didn't start: fast blink forever
void FailForever()
{
  for (;;)
  {
    digitalWrite(PIN_LED, LOW);
    delay(100);
    digitalWrite(PIN_LED, HIGH);
    delay(100);
  }
}
