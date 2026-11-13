// YamuraLog V6.1 (Teensy 4.0 CAN FD GPS node)
// Teensy 4.0/CAN FD
//
// Brian B. Smith
// brianbsmith.com
// February 2022 updated for Visual Code and PlatformIO November 2026
// Part of the YamuraLog project
//
#define VERBOSE // uncomment for output to serial
#include <FlexCAN_T4.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // ZOE u-blox GPS
#include <Wire.h>                                 //Needed for I2C to GNSS
#include "../../YamuraLog_Common/src/YamuraLog_Structures.h"
#define STATUS_LED 13
#define GPS_AQ_LED 22
#define GNSS_CS 10
#define CANID  0x050
// using 11 bit Identifiers
// 7 bit node type   (0x00 to 0x3F)
// 4 bit node number (0x00 to 0x0F)
// hub gets lowest possible number so it has bus priority
// YamuraLog device IDs (all use standard 11 bit IDs)
// HUB       0000001 0000      (one allowed) 0x10  
// Control   0000010 0000      (one allowed) 0x20
// A2D       0000011 0000-1111               0x30-0x3F (this node)
// Accel     0000100 0000      (one allowed) 0x40
// GPS       0000101 0000      (one allowed) 0x50
// Tire      0000110 0000-1111               0x60-0x6F
// Shock     0000111 0000-1111               0x70-0x7F
// WheelSpd  0001000 0000-1111               0x80-0x8F
// RPM       0001001 0000      (one allowed) 0x90
// CAN       0001010 0000      (one allowed) 0xA0
//
// flexcan FD port
//
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> canFD;  // can3 port
//
// sample delay doesn't apply for GPS node
// data is sent as soon as a new position is received
// typically around every 100ms, but can be shorter or longer
//
uint8_t d = 0;
char buffer512[512];
// adjust for local to hub time
uint32_t deltaTime = 0;
uint32_t lastMessage = 0;

UBX_NAV_PVT_data_t* globalUBXStruct;  // pointer to position structure
SFE_UBLOX_GNSS gnssDevice;
bool positionUpdated = false;
// forward declarations
void SendSensorValues();

//
//
//
void setup(void) 
{
  pinMode(STATUS_LED, OUTPUT);   
  digitalWrite(STATUS_LED,HIGH);
  pinMode(GPS_AQ_LED, OUTPUT);   
  digitalWrite(GPS_AQ_LED,HIGH);
  Serial.begin(115200); 
  delay(1000);
  #ifdef VERBOSE
  Serial.println("YamuraLog GPS Node/Teensy 4.0");
  #endif
  digitalWrite(STATUS_LED,LOW);
  digitalWrite(GPS_AQ_LED,LOW);
  //
  // GPS initialization
  //
  #ifdef VERBOSE
  sprintf(buffer512,"Initializing GNSS on I2C");
  Serial.println(buffer512);
  #endif
  Wire.begin();
  Wire.setClock(400000);  
  while (gnssDevice.begin() == false) 
  {
    #ifdef VERBOSE
    sprintf(buffer512,"GNSS not detected on I2C. Retry in 5 sec");
    Serial.println(buffer512);
    #endif
    delay (5000);
  }
  #ifdef VERBOSE
  Serial.println("u-blox GNSS detected on I2C bus");
  Serial.println("Begin u-blox reset to factory default...");
  #endif
  /* start reset to defaults */
  gnssDevice.factoryDefault(); 
  delay(5000);
  #ifdef VERBOSE
  Serial.println("Reset complete");
  #endif
  /* end reset to defaults */
  // GPS is enabled and then disable Galileo, GLONASS, BeiDou, SBAS and QZSS to achieve 25Hz.
  gnssDevice.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS); // Make sure GPS is enabled (we must leave at least one major GNSS enabled!)
  gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS); // Disable SBAS
  gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO); // Disable Galileo
  gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU); // Disable BeiDou
  gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_IMES); // Disable IMES
  gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_QZSS); // Disable QZSS
  gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS); // Disable GLONASS
  delay(2000); // Give the module some extra time to get ready
  gnssDevice.setI2COutput(COM_TYPE_UBX);      // Set the I2C port to output UBX only (turn off NMEA noise)
  gnssDevice.setNavigationFrequency(10);      //Produce 'n' solutions per second
  gnssDevice.setAutoPVT(true);                //Tell the GNSS to send each solution
  #ifdef VERBOSE
  sprintf(buffer512,"u-blox GNSS initialized on SPI - update at %dHz", gnssDevice.getNavigationFrequency());
  Serial.println(buffer512);  
  #endif
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
  // last message received
  lastMessage = millis();
}
//
// send sensor data
//
void SendSensorValues()
{
  CANFD_message_t msg;
  msg.len = GPS_SIZE;
  msg.id = CANID;
  msg.seq = 1;

  #ifdef VERBOSE
  sprintf(buffer512, "Send\t%ld\tid\t0x%02X\tGPS\t%02d/%02d/%04d %02d:%02d:%02d\tLAT\t%ld\tLONG\t%ld\tCOURSE\t%ld\tSIV\t%d", gpsData.dataStructure.timestamp,
                                                                                                                (int)msg.id,
                                                                                                                gpsData.dataStructure.gpsDay,
                                                                                                                gpsData.dataStructure.gpsMonth,
                                                                                                                gpsData.dataStructure.gpsYear,
                                                                                                                gpsData.dataStructure.gpsHour,
                                                                                                                gpsData.dataStructure.gpsMinute,
                                                                                                                gpsData.dataStructure.gpsSecond,
                                                                                                                gpsData.dataStructure.latitude,
                                                                                                                gpsData.dataStructure.longitude,
                                                                                                                gpsData.dataStructure.course,
                                                                                                                gpsData.dataStructure.SIV);
  Serial.println(buffer512);
  #endif
  // load the message payload (copy from dataBuffer)
  for(int idx = 0; idx < GPS_SIZE; idx++)
  {
  	msg.buf[idx] = gpsData.dataBuffer[idx];
  }
  // write to can3
  canFD.write(msg);
}
//
// loop - check for received messages
// send handled by timer
//
void loop() 
{
  // check GPS for update
  // Calling getPVT returns true if there actually is a fresh navigation solution available.
  // Start the reading only when valid LLH is available
  if ((gnssDevice.getPVT() && 
      (gnssDevice.getInvalidLlh() == false)) ||
      (millis() - lastMessage > 10000))
  {
    lastMessage = millis();
    gpsData.dataStructure.timestamp = millis()+deltaTime;
    gpsData.dataStructure.gpsYear = gnssDevice.getYear();
    gpsData.dataStructure.gpsMonth = gnssDevice.getMonth();
    gpsData.dataStructure.gpsDay = gnssDevice.getDay();
    gpsData.dataStructure.gpsHour = gnssDevice.getHour();
    gpsData.dataStructure.gpsMinute = gnssDevice.getMinute();
    gpsData.dataStructure.gpsSecond = gnssDevice.getSecond();
    gpsData.dataStructure.latitude =  gnssDevice.getLatitude();
    gpsData.dataStructure.longitude = gnssDevice.getLongitude();
    gpsData.dataStructure.course =    gnssDevice.getHeading();
    gpsData.dataStructure.speed =     gnssDevice.getGroundSpeed();
    gpsData.dataStructure.SIV =       gnssDevice.getSIV();
    if(gnssDevice.getSIV() > 0)
    {
      digitalWrite(GPS_AQ_LED,HIGH);
    }
    else
    {
      digitalWrite(GPS_AQ_LED,LOW);
    }
    SendSensorValues();
  }  

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
