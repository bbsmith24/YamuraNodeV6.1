// YamuraLog V6.1 (Teensy 4.0 CAN FD engine RPM node)
// Teensy 4.0/CAN FD
//
// Brian B. Smith
// brianbsmith.com
// November 2026
// Part of the YamuraLog project
//
// Reads the engine tach signal through an optocoupler (H11L1) and sends
// RPM on CAN id 0x90. Drives an LED shift/redline light.
//
#include <FlexCAN_T4.h>
#include "../../YamuraLog_Common/src/YamuraLog_Structures.h"

#define CANID  0x90
// using 11 bit Identifiers
//          1
// 01234567890
// -------      7 bit node type   (0x00 to 0x3F)
//        ----  4 bit node number (0x00 to 0x0F)
// hub gets lowest possible number so it has bus priority
  // YamuraLog device IDs (all use standard 11 bit IDs)
  // HUB       0000001 0000      (only one allowed) 0x10
  // (0x20 was the Control box - obsolete, removed)
  // A2D       0000011 0000-1111                    0x30-0x3F
  // Accel     0000100 0000      (only one allowed) 0x40
  // GPS       0000101 0000      (only one allowed) 0x50
  // Tire      0000110 0000-1111                    0x60-0x6F
  // Shock     0000111 0000-1111                    0x70-0x7F
  // WheelSpd  0001000 0000-1111                    0x80-0x8F
  // RPM       0001001 0000      (only one allowed) 0x90 (this node)
  // CAN       0001010 0000      (only one allowed) 0xA0
//#define VERBOSE // uncomment for output to serial

//
// pins
//
#define TACH_PIN    5    // H11L1 output (open collector, pulled up to 3.3V)
#define SHIFT_PIN   9    // shift/redline light (drive the LED through a transistor)
#define STATUS_LED 13    // on while tach pulses are arriving
//
// engine settings - 1ZZ-FE (MR2 Spyder)
//
#define PULSES_PER_REV   2     // 4 cylinder tach signal, one pulse per spark
#define SHIFT_RPM     6300     // shift light on
#define SHIFT_HYST     150     // shift light off below SHIFT_RPM - SHIFT_HYST
#define REDLINE_RPM   6800     // shift light flashes
#define FLASH_MS        50     // redline flash half period
#define MAX_RPM      12000     // pulses closer together than this are noise
#define STOPPED_MS     500     // no pulse for this long = engine stopped
#define AVG_PULSES       4     // periods averaged for each reading
#define SAMPLE_RATE  SAMPLE_50Hz

// shortest believable time between pulses
const uint32_t MIN_PERIOD_US = 60000000UL / ((uint32_t)MAX_RPM * PULSES_PER_REV);
//
// tach pulse capture (written in the interrupt handler)
//
volatile uint32_t lastPulseUs = 0;
volatile uint32_t lastPulseMs = 0;
volatile uint32_t periods[AVG_PULSES];
volatile uint8_t  periodIdx = 0;
volatile uint8_t  periodCount = 0;
volatile uint32_t pulseCount = 0;
//
// flexcan FD port
//
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> canFD;  // can3 port

char buffer512[512];
// adjust for local to hub time
uint32_t deltaTime = 0;
uint32_t lastSend = 0;
uint8_t shiftState = 0;
// forward declarations
void TachInt();
void ReadRPM(uint32_t &periodUs, uint16_t &rpm, uint32_t &pulseMs);
void UpdateShiftLight(uint16_t rpm);
void SendSensorValues();
void ReadHub();

//
//
//
void setup(void)
{
  #ifdef VERBOSE
  Serial.begin(115200);
  Serial.println("YamuraLog RPM Node");
  Serial.printf("CAN id 0x%02X, shift %d, redline %d\n", CANID, SHIFT_RPM, REDLINE_RPM);
  #endif
  pinMode(STATUS_LED, OUTPUT);
  pinMode(SHIFT_PIN, OUTPUT);
  // lamp test - shift light on for half a second at power up
  digitalWrite(SHIFT_PIN, HIGH);
  digitalWrite(STATUS_LED, HIGH);
  delay(500);
  digitalWrite(SHIFT_PIN, LOW);
  digitalWrite(STATUS_LED, LOW);
  //
  // tach input - H11L1 output goes low while the tach signal is high
  //
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(TACH_PIN, TachInt, FALLING);
  //
  // CAN-FD setup
  //
  canFD.begin();
  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate =   ARB_BAUD;
  config.baudrateFD = FD_BAUD;
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 75;
  canFD.setRegions(64);
  canFD.setBaudRateAdvanced(config, 1, 1);
  canFD.setMBFilter(REJECT_ALL);
  // mailbox 0 - messages from the hub, ignore all others
  canFD.setMBFilterRange(MB0, 0x10, 0x10);
  canFD.enhanceFilter(MB0);
}
//
// loop - check for hub messages, update the shift light, send rpm at the sample rate
//
void loop()
{
  // needed for sequential frame transmit and callback queues
  canFD.events();
  ReadHub();

  uint32_t periodUs;
  uint16_t rpm;
  uint32_t pulseMs;
  ReadRPM(periodUs, rpm, pulseMs);
  UpdateShiftLight(rpm);
  digitalWrite(STATUS_LED, rpm > 0 ? HIGH : LOW);

  if(millis() - lastSend >= SAMPLE_RATE)
  {
    lastSend = millis();
    rpmData.dataStructure.timestamp = (rpm > 0 ? pulseMs : lastSend) + deltaTime;
    rpmData.dataStructure.periodUs = periodUs;
    rpmData.dataStructure.rpm = rpm;
    rpmData.dataStructure.shiftState = shiftState;
    rpmData.dataStructure.reserved = 0;
    SendSensorValues();
  }
}
//
// tach pulse - keep the period, reject pulses too close together (ignition noise)
//
void TachInt()
{
  uint32_t nowUs = micros();
  uint32_t period = nowUs - lastPulseUs;
  if(pulseCount > 0 && period < MIN_PERIOD_US)
  {
    return;
  }
  lastPulseUs = nowUs;
  lastPulseMs = millis();
  if(pulseCount > 0)
  {
    periods[periodIdx] = period;
    periodIdx = (periodIdx + 1) % AVG_PULSES;
    if(periodCount < AVG_PULSES)
    {
      periodCount++;
    }
  }
  pulseCount++;
}
//
// averaged period and rpm from the latest pulses; 0 if the engine is stopped
//
void ReadRPM(uint32_t &periodUs, uint16_t &rpm, uint32_t &pulseMs)
{
  uint32_t sum = 0;
  uint8_t count;
  uint32_t lastUs;
  noInterrupts();
  count = periodCount;
  lastUs = lastPulseUs;
  pulseMs = lastPulseMs;
  for(int idx = 0; idx < count; idx++)
  {
    sum += periods[idx];
  }
  interrupts();

  if(count > 0 && (micros() - lastUs) > (uint32_t)STOPPED_MS * 1000UL)
  {
    // stopped - restart the average so the first reading after a start is clean
    noInterrupts();
    periodCount = 0;
    pulseCount = 0;
    interrupts();
    count = 0;
  }
  if(count == 0)
  {
    periodUs = 0;
    rpm = 0;
    return;
  }
  periodUs = sum / count;
  rpm = (uint16_t)(60000000UL / (periodUs * PULSES_PER_REV));
}
//
// shift light: off, on at SHIFT_RPM, flashing at REDLINE_RPM
//
void UpdateShiftLight(uint16_t rpm)
{
  if(rpm >= REDLINE_RPM)
  {
    shiftState = 2;
  }
  else if(rpm >= SHIFT_RPM)
  {
    shiftState = 1;
  }
  else if(rpm < SHIFT_RPM - SHIFT_HYST)
  {
    shiftState = 0;
  }
  // between SHIFT_RPM - SHIFT_HYST and SHIFT_RPM: keep the current state
  else if(shiftState == 2)
  {
    shiftState = 1;
  }

  switch(shiftState)
  {
    case 2:
      digitalWrite(SHIFT_PIN, ((millis() / FLASH_MS) & 1) ? HIGH : LOW);
      break;
    case 1:
      digitalWrite(SHIFT_PIN, HIGH);
      break;
    default:
      digitalWrite(SHIFT_PIN, LOW);
      break;
  }
}
//
// send rpm data
//
void SendSensorValues()
{
  CANFD_message_t msg;
  msg.len = RPM_SIZE;
  msg.id = CANID;
  msg.seq = 1;
  for(int idx = 0; idx < RPM_SIZE; idx++)
  {
    msg.buf[idx] = rpmData.dataBuffer[idx];
  }
  canFD.write(msg);
  #ifdef VERBOSE
  sprintf(buffer512, "%lu\t0x%X\tRPM\t%u\tperiod\t%lu us\tshift\t%u",
          (unsigned long)rpmData.dataStructure.timestamp,
          (unsigned int)msg.id,
          rpmData.dataStructure.rpm,
          (unsigned long)rpmData.dataStructure.periodUs,
          rpmData.dataStructure.shiftState);
  Serial.println(buffer512);
  #endif
}
//
// hub messages - heartbeat sets the local to hub time offset
//
void ReadHub()
{
  CANFD_message_t msg;
  if(!canFD.readMB(msg))
  {
    return;
  }
  if(msg.id == 0x10)
  {
    if(msg.buf[0] == 1)
    {
      uint32_t localTime = millis();
      timeStampData.byteBuffer[0] = msg.buf[1];
      timeStampData.byteBuffer[1] = msg.buf[2];
      timeStampData.byteBuffer[2] = msg.buf[3];
      timeStampData.byteBuffer[3] = msg.buf[4];
      deltaTime = timeStampData.timestamp - localTime;
      #ifdef VERBOSE
      sprintf(buffer512, "%lu\tHeartbeat\tmsgid=0x%X", millis(), (unsigned int)msg.id);
      Serial.println(buffer512);
      #endif
    }
    #ifdef VERBOSE
    else
    {
      sprintf(buffer512, "%lu\tUnknown submessage %X from hub", millis(), msg.buf[0]);
      Serial.println(buffer512);
    }
    #endif
  }
  #ifdef VERBOSE
  else
  {
    Serial.print("Unknown MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
    Serial.print("  TS: "); Serial.println(msg.timestamp);
  }
  #endif
}
