// YamuraLog V6.1 (Teensy 4.0 CAN FD wheel speed node)
// Teensy 4.0/CAN FD
//
// Brian B. Smith
// brianbsmith.com
// February 2022 updated for Visual Code and PlatformIO November 2026
// Part of the YamuraLog project
//
#include <FlexCAN_T4.h>
#include "../../YamuraLog_Common/src/YamuraLog_Structures.h"

#define STATUS_LED 13
#define CANID  0x80
int canIDnode = 0;
// using 11 bit Identifiers
//          1
// 01234567890
// -------      7 bit node type   (0x00 to 0x3F)
//        ----  4 bit node number (0x00 to 0x0F)
// hub gets lowest possible number so it has bus priority
  // YamuraLog device IDs (all use standard 11 bit IDs)
  // HUB       0000001 0000      (only one allowed) 0x10  
  // Control   0000010 0000      (only one allowed) 0x20
  // A2D       0000011 0000-1111                    0x30-0x3F (this node)
  // Accel     0000100 0000      (only one allowed) 0x40
  // GPS       0000101 0000      (only one allowed) 0x50
  // Tire      0000110 0000-1111                    0x60-0x6F
  // Shock     0000111 0000-1111                    0x70-0x7F
  // WheelSpd  0001000 0000-1111                    0x80-0x8F
  // RPM       0001001 0000      (only one allowed) 0x90
  // CAN       0001010 0000      (only one allowed) 0xA0
//#define VERBOSE // uncomment for output to serial

// pins 5-8 falling edge interrupts for wheel speed sensors
// 'speed' value is the milliseconds since last falling edge
// convert to speed on analysis side
unsigned long lastSample[4];
unsigned long curSpeed[4];
bool sendSpeed[4];
unsigned long millisNow;
//
// flexcan FD port
//
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> canFD;  // can3 port
//
// sample delay doesn't apply for wheelspeed node
// data is sent as soon as a sensor is triggered
// depends on wheel size, number of magnets, and speed
//
char buffer512[512];
// adjust for local to hub time
uint32_t deltaTime = 0;
// forward declarations
void SendSensorValues(int wheelNum);
void Speed0Int();
void Speed1Int();
void Speed2Int();
void Speed3Int();

//
//
//
void setup(void) 
{
  #ifdef VERBOSE
  Serial.begin(115200); 
  Serial.println("YamuraLog WheelSpeed Node");
  #endif
  // 
  // jumpers for CANID modifier
  // in bands of 4 
  // 0x80-0x83 (all jumpers closed)
  // 0x84-0x87 (jumper 1 open)
  // 0x88-0x8B (jumper 2 open)
  // 0x8C-0x8F (jumper 3 open)
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  canIDnode = CANID;
  if(digitalRead(3) == HIGH)
  {
    canIDnode += 0x0C;
  }
  else if(digitalRead(2) == HIGH)
  {
    canIDnode += 0x08;
  }
  else if(digitalRead(1) == HIGH)
  {
    canIDnode += 0x04;
  }
  #ifdef VERBOSE
  Serial.printf("full CAN ids 0x%02X - 0x%02X\n", canIDnode, canIDnode + 3);
  #endif
  // 
  // digital pin configuration
  // set to INPUT_PULLUP for all digital pins
  // zero counters
  // attach falling edge interrupts
  for(int idx = 0; idx < 4; idx++)
  {
    pinMode(idx+5, INPUT_PULLUP);
    curSpeed[idx] = 0;
    lastSample[idx] = 0;
    sendSpeed[idx] = false;
  }
  attachInterrupt(5, Speed0Int, RISING);
  attachInterrupt(6, Speed1Int, RISING);
  attachInterrupt(7, Speed2Int, RISING);
  attachInterrupt(8, Speed3Int, RISING);
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
  // mailbox 0 - messages from control and hub box, ignore all others
  canFD.setMBFilterRange(MB0,010,0x20); 
  canFD.enhanceFilter(MB0);
}
//
// send sensor data
//
void SendSensorValues(int wheelNum)
{
  // latest speed readings already in structure
  CANFD_message_t msg;
  msg.len = SPEED_SIZE;
  msg.id = canIDnode + wheelNum;
  msg.seq = 1;
  speedData.dataStructure.timestamp = lastSample[wheelNum] + deltaTime;
  speedData.dataStructure.speedVal = curSpeed[wheelNum];
  // load the message payload (copy from speedData.dataBuffer
  for(int idx = 0; idx < SPEED_SIZE; idx++)
  {
    msg.buf[idx] = speedData.dataBuffer[idx];
  }
  // write to can3
  canFD.write(msg);         
  #ifdef VERBOSE 
  sprintf(buffer512, "%d\t0x%X\tSpeed\t%05d", speedData.dataStructure.timestamp,
                                              msg.id,
                                              speedData.dataStructure.speedVal);
  Serial.println(buffer512);
  #endif
}
//
// loop - check for received messages, check for new sensor values to send
//
void loop() 
{
  // needed for sequential frame transmit and callback queues
  canFD.events();
  // sendSpeed set in pin change handlers, call SendSensorValues(idx) here to minimize time in handlers
  for(int idx = 0; idx < 4; idx++)
  {
    if(sendSpeed[idx] == true)
    {
      SendSensorValues(idx);
      sendSpeed[idx] = false;
    }
  }
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
      Serial.print("Unknown MB: "); Serial.print(msg.mb);
      Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
      Serial.print("  TS: "); Serial.println(msg.timestamp);
      #endif
    }
  }
}
void Speed0Int()
{
  millisNow = millis();
  curSpeed[0] = millisNow - lastSample[0]; 
  lastSample[0] = millisNow;
  sendSpeed[0] = true;
}
void Speed1Int()
{
  millisNow = millis();
  curSpeed[1] = millisNow - lastSample[1]; 
  lastSample[1] = millisNow;
  sendSpeed[1] = true;
}
void Speed2Int()
{
  millisNow = millis();
  curSpeed[2] = millisNow - lastSample[2]; 
  lastSample[2] = millisNow;
  sendSpeed[2] = true;
}
void Speed3Int()
{
  millisNow = millis();
  curSpeed[3] = millisNow - lastSample[3];
  lastSample[3] = millisNow;
  sendSpeed[3] = true;
}
