// YamuraLog V6.1 (Teensy 4.0 CAN FD IR tire temperature node)
// Teensy 4.0/CAN FD
//
// Brian B. Smith
// brianbsmith.com
// February 2022 updated for Visual Code and PlatformIO November 2026
// Part of the YamuraLog project
//
#include <FlexCAN_T4.h>
#include "MLX90621.h"
#include "../../YamuraLog_Common/src/YamuraLog_Structures.h"

#define STATUS_LED 13
#define CANID  0x60
// using 11 bit Identifiers
//          1
// 01234567890
// -------      7 bit node type   (0x00 to 0x3F)
//        ----  4 bit node number (0x00 to 0x0F)
// hub gets lowest possible number so it has bus priority
// YamuraLog device IDs (all use standard 11 bit IDs)
// HUB       0000001 0000      (only one allowed) 0x10  
// (0x20 was the Control box - obsolete, removed)
// A2D       0000011 0000-1111                    0x30-0x3F (this node)
// Accel     0000100 0000      (only one allowed) 0x40
// GPS       0000101 0000      (only one allowed) 0x50
// TireTemp  0000110 0000-1111                    0x60-0x6F
// Shock     0000111 0000-1111                    0x70-0x7F
// WheelSpd  0001000 0000-1111                    0x80-0x8F
// RPM       0001001 0000      (only one allowed) 0x90
// CAN       0001010 0000      (only one allowed) 0xA0
#define VERBOSE // uncomment for output to serial

MLX90621 sensor; // create an instance of the Sensor class
//
// flexcan FD port
//
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> canFD;  // can3 port
// time between samples in milliseconds
//uint32_t sampleDelay =    1;     // 0.001 interval 1000 samples/second
//uint32_t sampleDelay =   10;     // 0.010 interval  100 samples/second
//uint32_t sampleDelay =   20;     // 0.020 interval   50 samples/second
//uint32_t sampleDelay =  100;     // 0.100 interval   10 samples/second
//uint32_t sampleDelay =  200;     // 0.200 interval    5 samples/second
//uint32_t sampleDelay =  250;     // 0.250 interval    4 samples/second
  uint32_t sampleDelay =  500;     // 0.500 interval    2 samples/second
//uint32_t sampleDelay = 1000;     // 1.000 interval    1 samples/second
IntervalTimer timer;
uint8_t d = 0;
char buffer512[512];
// adjust for local to hub time
uint32_t deltaTime = 0;
// forward declarations
void SendSensorValues();

//
//
//
void setup()
{ 
  pinMode(STATUS_LED, OUTPUT);   
  digitalWrite(STATUS_LED,HIGH);
  Serial.begin(115200); 
  #ifdef VERBOSE
  Serial.println("YamuraLog IR tire temp Node/Teensy 4.0");
  #endif
  digitalWrite(STATUS_LED,LOW);

  #ifdef VERBOSE
  Serial.println(F("Initializing thermo sensor on I2C..."));
  #endif
  sensor.setRefreshRate(RATE_8HZ);
  sensor.setResolution(RES_17bit);
  sensor.setEmissivity(1.0);
  // start the thermo cam  
  sensor.initialize ();
  #ifdef VERBOSE
  Serial.println(F("Thermo sensor ready on I2C"));
  #endif
  //
  // CAN-FD setup
  //
  canFD.begin();
  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  //config.clock = CLK_60MHz;
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
  // sensor value send timer
  timer.begin(SendSensorValues, (sampleDelay * 1000));
}
//
// send sensor data
//
void SendSensorValues()
{
  // read sensor. true returns converted values in 
  //              false returns raw values in sensor.irData array
  sensor.measure(true);                             
  CANFD_message_t msg;
  msg.len = IR_SIZE;
  msg.id = CANID;
  msg.seq = 1;
  irData.dataStructure.timestamp = millis()+deltaTime;
  int cellIdx = 0;
  //int idxMax = 60;
  //for(int seqIdx = 0; seqIdx < 3; seqIdx++)
  //{
    // 22 for messages 0 and 1, 20 for message 2
    //idxMax = seqIdx < 2 ? 22 : 20;
    //irData.dataStructure.sequence = seqIdx;
    for(int idx = 0; idx < 64; idx++)
    {
      if(idx % 16 == 0) // skip 0, 16, 32, 48 - 1st entry in each row
      {
        continue;
      }
      irData.dataStructure.tempVal[cellIdx] = (uint8_t)(((sensor.temperatures[idx] * 10) - 50) > 0 ? ((sensor.temperatures[idx] * 10) - 50) : 0);
      cellIdx++;
    }
    // load the message buffer
    for(int bufIdx = 0; bufIdx < IR_SIZE; bufIdx++)
    {
      msg.buf[bufIdx] = irData.dataBuffer[bufIdx];
    }
    // write to can3
    #ifdef VERBOSE 
    sprintf(buffer512, "%d\t0x%02X\t", irData.dataStructure.timestamp,
                                       msg.id);
    Serial.print(buffer512);
    for(int bufIdx = 0; bufIdx < 60; bufIdx++)
    {
      Serial.print(irData.dataStructure.tempVal[bufIdx] + 50);
      Serial.print(" ");
    }
    Serial.println();
    #endif
    canFD.write(msg);
  //}
  Serial.println();
}
//
//
//
void loop()
{  // needed for sequential frame transmit and callback queues
  canFD.events();
  CANFD_message_t msg;
  if (  canFD.readMB(msg) ) 
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
        sprintf(buffer512, "%d\tHeartbeat\tmsgid=0x%X", millis(), msg.id);// - Local time\t%d\tAdjusted\t%d", localTime, (localTime + deltaTime)); 
        Serial.println(buffer512);      
        #endif
        deltaTime = timeStampData.timestamp - localTime;
      }
      #ifdef VERBOSE
      else
      {
        sprintf(buffer512, "%d\tUnknown submessage %X from hub", millis(), msg.buf[0]); 
        Serial.println(buffer512);      
      }
      #endif
    }
    else
    {
      #ifdef VERBOSE
      Serial.print("Unknown MB: "); Serial.print(msg.mb);
      Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
      Serial.print("  TS: "); Serial.println(msg.timestamp);
      #endif
    }
   }
}
