// YamuraLog V6.1 (Teensy 4.0 CAN FD A2D+Digital node)
// Teensy 4.0/CAN FD
//
// Brian B. Smith
// brianbsmith.com
// February 2022 updated for Visual Code and PlatformIO July 2026
// Part of the YamuraLog project
//#define VERBOSE // uncomment for debug output to serial
//#define EXTRA_VERBOSE // uncomment for more debug output to serial
#include <FlexCAN_T4.h>
#include "..\..\YamuraLog_Common\src\YamuraLog_Structures.h"

#define STATUS_LED 13
#define CANID 0x30
// using 11 bit Identifiers
// 7 bit node type   (0x00 to 0x3F)
// 4 bit node number (0x00 to 0x0F)
// hub gets lowest possible number so it has bus priority
// YamuraLog device IDs (all use standard 11 bit IDs)
// HUB       0000001 0000      (one allowed) 0x10
// Control   0000010 0000      (one allowed) 0x20
// A2D       0000011 0000-1111               0x30-0x3F
// Accel     0000100 0000      (one allowed) 0x40
// GPS       0000101 0000      (one allowed) 0x50
// Tire      0000110 0000-1111               0x60-0x6F
// Shock     0000111 0000-1111               0x70-0x7F
// WheelSpd  0001000 0000-1111               0x80-0x8F
// RPM       0001001 0000      (one allowed) 0x90
// CAN       0001010 0000      (one allowed) 0xA0
//
// A/D pin definitions
// max of 8 pins available, don't double up (no digital and analog reads...)
int digitalPins[8];
int analogPins[8];
int digitalPinCount = 0;
int analogPinCount = 0;
//
// flexcan FD port
//
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> canFD;  // can3 port
// timer for samples
uint32_t sampleDelay = SAMPLE_100Hz;
IntervalTimer timer;
uint8_t d = 0;
char buffer512[512];
// adjust for local to hub time
uint32_t deltaTime = 0;
//
//
void SetupChannels();
void SendSensorValues();

void setup(void)
{
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  Serial.begin(115200);
  delay(1000);
  #ifdef VERBOSE
  Serial.println("YamuraLog A2D and Digital Node/Teensy 4.0");
  #endif
  digitalWrite(STATUS_LED, LOW);
  // setup A/D pins
  SetupChannels();
  //
  // CAN-FD setup
  //
  canFD.begin();
  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate = ARB_BAUD;
  config.baudrateFD = FD_BAUD;
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 75;
  canFD.setRegions(64);
  canFD.setBaudRateAdvanced(config, 1, 1);
  canFD.setMBFilter(REJECT_ALL);
  // mailbox 0 - messages from control and hub box, ignore all others
  canFD.setMBFilterRange(MB0, 010, 0x20);
  canFD.enhanceFilter(MB0);
  // sensor value send timer
  timer.begin(SendSensorValues, (sampleDelay * 1000));
}
//
// send sensor data
//
void SendSensorValues()
{
  adData.dataStructure.timestamp = millis() + deltaTime;
  adData.dataStructure.digitalVal = 0;
  // read the digital pins
  for (int idx = 0; idx < digitalPinCount; idx++)
  {
    adData.dataStructure.digitalVal += (uint8_t)(digitalRead(digitalPins[idx]) << idx);
  }
  // read the analog pins
  for (int idx = 0; idx < analogPinCount; idx++)
  {
    adData.dataStructure.analogVal[idx] = analogRead(analogPins[idx]);
  }


  CANFD_message_t msg;
  msg.len = AD_SIZE;
  msg.id = CANID;
  msg.seq = 1;
  // load the message payload (copy from dataBuffer)
  for (int idx = 0; idx < AD_SIZE; idx++)
  {
    msg.buf[idx] = adData.dataBuffer[idx];
  }
  #ifdef EXTRA_VERBOSE
  sprintf(buffer512, "Send\t%ld\tid\t0x%02X\tAnalog\t%d\t%d\t%d\t%d Digital 0x%X",
          adData.dataStructure.timestamp,
          (int)msg.id,
          adData.dataStructure.analogVal[0],
          adData.dataStructure.analogVal[1],
          adData.dataStructure.analogVal[2],
          adData.dataStructure.analogVal[3],
          adData.dataStructure.digitalVal);
  Serial.println(buffer512);
  #endif
  // write to can3
  canFD.write(msg);
}
//
// loop - check for received messages
// send handled by timer
//
void loop()
{
  // needed for sequential frame transmit and callback queues
  canFD.events();
  CANFD_message_t msg;
  if (canFD.readMB(msg))
  {
    // hub messages
    if(msg.id == 0x10)
    {
      #ifdef VERBOSE
      Serial.print(millis());
      Serial.print("\tHub");
      Serial.print("\tID\t");
      Serial.print(msg.id, HEX);
      Serial.print("\tMB\t");
      Serial.println(msg.mb);
      #endif
      if(msg.buf[0] == 1)
      {
        uint32_t localTime = millis();
        timeStampData.byteBuffer[0] = msg.buf[1];
        timeStampData.byteBuffer[1] = msg.buf[2];
        timeStampData.byteBuffer[2] = msg.buf[3];
        timeStampData.byteBuffer[3] = msg.buf[4];
        #ifdef VERBOSE
        sprintf(buffer512, "%ld\tHeartbeat\tmsgid=02x%X", millis(), (int)msg.id);
        #endif
        deltaTime = timeStampData.timestamp - localTime;
      }
      #ifdef VERBOSE
      else
      {
        sprintf(buffer512, "%ld\tUnknown submessage %02X from hub", millis(), (int)msg.buf[0]);
      }
      Serial.println(buffer512);
      #endif
    }
    // control box messages
    else if (msg.id == 0x20)
    {
      #ifdef VERBOSE
      Serial.print(millis());
      Serial.print("\tControl");
      Serial.print("\tID\t");
      Serial.print(msg.id, HEX);
      Serial.print("\tMB\t");
      Serial.println(msg.mb);
      #endif
    }
    else
    {
      #ifdef VERBOSE
      Serial.print("Unknown MB: ");
      Serial.print(msg.mb);
      Serial.print("  ID: 0x");
      Serial.print(msg.id, HEX);
      Serial.print("  TS: ");
      Serial.println(msg.timestamp);
      #endif
    }
  }
}
//
//
void SetupChannels() {
  // digitial channels - input with internal pullup
  digitalPinCount = 4;
  digitalPins[0] = 14;
  digitalPins[1] = 15;
  digitalPins[2] = 16;
  digitalPins[3] = 17;
  // define digital pins as INPUT_PULLUP
  for (int digitalPinIdx = 0; digitalPinIdx < digitalPinCount; digitalPinIdx++) {
    pinMode(digitalPins[digitalPinIdx], INPUT_PULLUP);
  }
  // analog channels
  analogPinCount = 4;
  analogPins[0] = 18;
  analogPins[1] = 19;
  analogPins[2] = 20;
  analogPins[3] = 21;
  // no setup required for analog pins
}
