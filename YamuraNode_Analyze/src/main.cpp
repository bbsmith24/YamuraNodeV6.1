// YamuraLog V6.1 (Teensy 4.0 CAN FD bus analyzer - formerly CAN_Sniffer)
// Teensy 4.0/CAN FD
//
// Brian B. Smith
// brianbsmith.com
// February 2022 updated for Visual Code and PlatformIO November 2026
// Part of the YamuraLog project
//
// Listens to every YamuraLog CAN id and prints the selected message types to serial.
// Pick message types from the serial menu (M shows it).
//
#include <FlexCAN_T4.h>
#include <TimeLib.h>
#include "../../YamuraLog_Common/src/YamuraLog_Structures.h"
#include "USBHost_t36.h"

USBHost myusb;
USBHub hub1(myusb);

FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> canFD;  // can3 port
// using 11 bit Identifiers
//          1
// 01234567890
// -------      7 bit node type   (0x00 to 0x3F)
//        ----  4 bit node number (0x00 to 0x0F)
// hub gets lowest possible number so it has bus priority
  // YamuraLog device IDs (all use standard 11 bit IDs)
  // HUB       0000001 0000      (only one allowed) 0x10  (this node)
  // Control   0000010 0000      (only one allowed) 0x20
  // A2D       0000011 0000-1111                    0x30-0x3F
  // Accel     0000100 0000      (only one allowed) 0x40
  // GPS       0000101 0000      (only one allowed) 0x50
  // Tire      0000110 0000-1111                    0x60-0x6F
  // Shock     0000111 0000-1111                    0x70-0x7F
  // WheelSpd  0001000 0000-1111                    0x80-0x8F
  // RPM       0001001 0000      (only one allowed) 0x90
  // CAN       0001010 0000      (only one allowed) 0xA0
#define CANID  0x010
CANFD_timings_t config;

// index of a full buffer
#define SEND_BUFFER 256
#define LOCAL 0
#define REMOTE 1
//
int led = 13;

char buffer512[512];

bool timeValid = true;
// 0 control,1 A/D,2 IMU,3 GPS,4 tire,5 wheel,6 shock,7 RPM,8 CAN,9 hub,10 IMU ext,11 GPS ext
#define DISPLAY_COUNT 12
#define DISPLAY_IMU_EX 10
#define DISPLAY_GPS_EX 11
bool displayMessages[DISPLAY_COUNT] = {false};
// forward declarations
void DisplayMenu();
void CAN_ControlMsg(const CANFD_message_t &msg);
void CAN_A2DMessage(const CANFD_message_t &msg);
void CAN_AccelMessage(const CANFD_message_t &msg);
void CAN_GPSMessage(const CANFD_message_t &msg);
void CAN_TireIRMessage(const CANFD_message_t &msg);
void CAN_WheelSpeedMessage(const CANFD_message_t &msg);
void CAN_ShockMessage(const CANFD_message_t &msg);
void CAN_RPMMessage(const CANFD_message_t &msg);
void CAN_CANMessage(const CANFD_message_t &msg);
void CAN_HubMessage(const CANFD_message_t &msg);
void sendHeartbeat();

//
//
//
void setup(void) 
{
  pinMode(led, OUTPUT);   
  digitalWrite(led,HIGH);
  Serial.begin(115200); 
  Serial.println("YamuraLog CAN Sniffer");
  digitalWrite(led,LOW);
  delay(1000);

  canFD.begin();
  config.clock = CLK_24MHz;
  //config.clock = CLK_60MHz;
  config.baudrate =   ARB_BAUD;
  config.baudrateFD = FD_BAUD;
  //config.baudrateFD = 1500000;       // 1500kbps data rate
  //config.baudrateFD = 1000000;       // 1000kbps data rate
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 75;
  canFD.setRegions(64);
  canFD.setBaudRateAdvanced(config, 1, 1);
  // receive MBs start at 0
  canFD.setMB(MB0,RX,STD);
  canFD.setMB(MB1,RX,STD);
  canFD.setMB(MB2,RX,STD);
  canFD.setMB(MB3,RX,STD);
  canFD.setMB(MB4,RX,STD);
  canFD.setMB(MB5,RX,STD);
  canFD.setMB(MB6,RX,STD);
  canFD.setMB(MB7,RX,STD);
  canFD.setMB(MB8,RX,STD);
  canFD.setMB(MB9,RX,STD);
  canFD.enableMBInterrupts();
  
  // mailbox 0 - messages from control box
  canFD.setMBFilterRange(MB0,0x20,0x20); 
  canFD.enhanceFilter(MB0);
  canFD.onReceive(MB0,CAN_ControlMsg);
  
  // mailbox 1 - messages from A2D nodes
  canFD.setMBFilterRange(MB1,0x30,0x3F); 
  canFD.enhanceFilter(MB1);
  canFD.onReceive(MB1,CAN_A2DMessage);
  
  // mailbox 2 - messages from accelerometer node
  canFD.setMBFilterRange(MB2,0x40,0x4F); 
  canFD.enhanceFilter(MB2);
  canFD.onReceive(MB2,CAN_AccelMessage);
  
  // mailbox 3 - messages from gps node
  canFD.setMBFilterRange(MB3,0x50,0x5F); 
  canFD.enhanceFilter(MB3);
  canFD.onReceive(MB3,CAN_GPSMessage);
  
  // mailbox 4 - messages from IR tire temp nodes
  canFD.setMBFilterRange(MB4,0x60,0x6F); 
  canFD.enhanceFilter(MB4);
  canFD.onReceive(MB4,CAN_TireIRMessage);
  
  // mailbox 5 - messages from shock position nodes
  canFD.setMBFilterRange(MB5,0x70,0x7F); 
  canFD.enhanceFilter(MB5);
  canFD.onReceive(MB5,CAN_ShockMessage);
  
  // mailbox 6 - messages from wheel speed nodes
  canFD.setMBFilterRange(MB6,0x80,0x8F); 
  canFD.enhanceFilter(MB6);
  canFD.onReceive(MB6,CAN_WheelSpeedMessage);
  
  // mailbox 7 - messages from wheel speed nodes
  canFD.setMBFilterRange(MB7,0x90,0x9F); 
  canFD.enhanceFilter(MB7);
  canFD.onReceive(MB7,CAN_RPMMessage);

  // mailbox 8 - messages from wheel speed nodes
  canFD.setMBFilterRange(MB8,0xA0,0xAF); 
  canFD.enhanceFilter(MB8);
  canFD.onReceive(MB8,CAN_CANMessage);

  // mailbox 9 - messages from hub node
  canFD.setMBFilterRange(MB9,0x00,0x1F); 
  canFD.enhanceFilter(MB9);
  canFD.onReceive(MB9,CAN_HubMessage);

  canFD.mailboxStatus();
  DisplayMenu();
}
//
//
//
void loop() 
{
  // check CAN messages
  canFD.events(); 
  char menuChoice;
  if (Serial.available()) 
  {
    for(int idx = 0; idx < DISPLAY_COUNT; idx++)
    {
      displayMessages[idx] = false;
    }
   
    menuChoice = Serial.read();
    switch (menuChoice) 
    {
      case 'm': 
      case 'M': 
        Serial.println("All messages OFF");
        DisplayMenu(); 
        break;
      case 'a':
      case 'A':
        Serial.println("All messages ON");
        for(int idx = 0; idx < DISPLAY_COUNT; idx++)
        {
          displayMessages[idx] = true;
        }
        break;
      case 'h': 
      case 'H':
        Serial.println("HUB messages (id 0x10) ON");
        displayMessages[9] = true;
        break;
      case 'c': 
      case 'C':
        Serial.println("CONTROL messages (id 0x20) ON");
        displayMessages[0] = true;
        break;
      case 'd': 
      case 'D':
        Serial.println("A/D messages (ids 0x30-0x3F) ON");
        displayMessages[1] = true;
        break;
      case 'i': 
        Serial.println("IMU accel-only messages (id 0x40, 16 bytes) ON");
        displayMessages[2] = true;
        break;
      case 'I':
        Serial.println("IMU accel+gyro messages (id 0x40 28 bytes, or id 0x41) ON");
        displayMessages[DISPLAY_IMU_EX] = true;
        break;
      case 'g': 
        Serial.println("GPS basic messages (id 0x50, 28 bytes) ON");
        displayMessages[3] = true;
        break;
      case 'G':
        Serial.println("GPS extended messages (id 0x50 38 bytes, or id 0x51) ON");
        displayMessages[DISPLAY_GPS_EX] = true;
        break;
      case 't': 
      case 'T':
        Serial.println("IR Tire messages (ids 0x60-0x6F) ON");
        displayMessages[4] = true;
        break;
      case 's': 
      case 'S':
        Serial.println("Shock Position messages (ids 0x70-0x7F) ON");
        displayMessages[6] = true;
        break;
      case 'w': 
      case 'W':
        Serial.println("Wheel Speed messages (ids 0x80-0x8F) ON");
        displayMessages[5] = true;
        break;
      case 'r': 
      case 'R':
        Serial.println("RPM messages (id 0x90) ON");
        displayMessages[7] = true;
        break;
      case 'x': 
      case 'X':
        Serial.println("CAN messages (id 0xA0) ON");
        displayMessages[8] = true;
        break;
      case 'b': 
      case 'B':
        Serial.println("Send Heartbeat message");
        displayMessages[8] = true;
        sendHeartbeat();
        DisplayMenu(); 
        break;
      default:
        Serial.print(menuChoice);
        Serial.println(" - unknown option");
        Serial.println("All messages OFF");
        break;
    }
    while (Serial.read() != -1); // remove rest of characters. 
  }
}
///
///
///
void DisplayMenu()
{
  for(int idx = 0; idx < DISPLAY_COUNT; idx++)
  {
    displayMessages[idx] = false;
  }
  Serial.println();
  
  Serial.println("CAN Configuration:");
  sprintf(buffer512, "Clock\t\t%d", config.clock);
  Serial.println(buffer512);  
  sprintf(buffer512, "Arbitration\t%lf", config.baudrate);
  Serial.println(buffer512);
  sprintf(buffer512, "FD\t\t%lf", config.baudrateFD);
  Serial.println(buffer512);
  Serial.println();
  Serial.println("Menu Options:");
  Serial.println("\tH - Show Hub messages            (id 0x10)");
  Serial.println("\tC - Show Control messages        (id 0x20)");
  Serial.println("\tD - Show A/D messages            (ids 0x30-0x3F)");
  Serial.println("\ti - Show IMU accel-only messages (id 0x40, 16 bytes)");
  Serial.println("\tI - Show IMU accel+gyro messages (id 0x40 28 bytes, or 0x41)");
  Serial.println("\tg - Show GPS basic messages      (id 0x50, 28 bytes)");
  Serial.println("\tG - Show GPS extended messages   (id 0x50 38 bytes, or 0x51)");
  Serial.println("\tT - Show IR Tire messages        (ids 0x60-0x6F)");
  Serial.println("\tW - Show Wheel Speed messages    (ids 0x80-0x8F)");
  Serial.println("\tS - Show Shock Position messages (ids 0x70-0x7F)");
  Serial.println("\tR - Show RPM messages            (id 0x90)");
  Serial.println("\tX - Show CAN messages            (id 0xA0)");
  Serial.println("\tA - Show All messages            (ids 0x10-0xA0)");
  Serial.println("\tB - Send Heartbeat");
  Serial.println();
  Serial.println("\tM - Menu");
  Serial.println();
}
//
// CAN mailbox message handlers
//
// control box message
//
void CAN_ControlMsg(const CANFD_message_t &msg) 
{
  if(!displayMessages[0])
  {
    return;
  }
  sprintf(buffer512, "%d\t0x%02X\tControl\tlen %d\t", millis(),
                                                      msg.id,
                                                      msg.len);
  if(msg.buf[0] == 1)
  { 
    if(msg.buf[1] == 1)
    {
      sprintf(buffer512, "%s\t%s", buffer512, "LOG START");
    }
    else if (msg.buf[1] == 0)
    {
      sprintf(buffer512, "%s\t%s", buffer512, "LOG STOP");
    }
    else 
    {
      sprintf(buffer512, "%s\t%s", buffer512, "UNKNOWN");
    }
  }
  else 
  {
    sprintf(buffer512, "%s\t%s", buffer512, "UNKNOWN");
  }
  Serial.println(buffer512);
}
//
// A2D node message
//
void CAN_A2DMessage(const CANFD_message_t &msg) 
{
  if(!displayMessages[1])
  {
    return;
  }
  for(int idx = 0; idx < AD_SIZE; idx++)
  {
    adData.dataBuffer[idx] = msg.buf[idx];
  }
  sprintf(buffer512, "%08d\t0x%X\tA+D\tAnalog\t%04d\t%04d\t%04d\t%04d\t%04d\t%04d\t%04d\t%04d Digital %d%d%d%d%d%d%d%d", adData.dataStructure.timestamp,
                                                                                                                         msg.id,
                                                                                                                         adData.dataStructure.analogVal[0],
                                                                                                                         adData.dataStructure.analogVal[1],
                                                                                                                         adData.dataStructure.analogVal[2],
                                                                                                                         adData.dataStructure.analogVal[3],
                                                                                                                         adData.dataStructure.analogVal[4],
                                                                                                                         adData.dataStructure.analogVal[5],
                                                                                                                         adData.dataStructure.analogVal[6],
                                                                                                                         adData.dataStructure.analogVal[7],
                                                                                                                         (adData.dataStructure.digitalVal &     1) == 0 ? 0 : 1,
                                                                                                                         (adData.dataStructure.digitalVal &     2) == 0 ? 0 : 1,
                                                                                                                         (adData.dataStructure.digitalVal &     4) == 0 ? 0 : 1,
                                                                                                                         (adData.dataStructure.digitalVal &     8) == 0 ? 0 : 1,
                                                                                                                         (adData.dataStructure.digitalVal &    16) == 0 ? 0 : 1,
                                                                                                                         (adData.dataStructure.digitalVal &    32) == 0 ? 0 : 1,
                                                                                                                         (adData.dataStructure.digitalVal &    64) == 0 ? 0 : 1,
                                                                                                                         (adData.dataStructure.digitalVal &   128) == 0 ? 0 : 1);
  Serial.println(buffer512);
}
//
// Accelerometer node message
//
void CAN_AccelMessage(const CANFD_message_t &msg) 
{
  // accel + gyro: 28 bytes on 0x40 (CAN FD sends it as 32), or id 0x41
  bool extended = (msg.id == 0x41) || (msg.len >= 28);
  if(!displayMessages[extended ? DISPLAY_IMU_EX : 2])
  {
    return;
  }
  for(int idx = 0; idx < IMU_SIZE; idx++)
  {
    imuData.dataBuffer[idx] = msg.buf[idx];
  }
  sprintf(buffer512, "%08d\t0x%02X\tIMU\tlen %d\tXYZ\t%0.4f\t%0.4f\t%0.4f", imuData.dataStructure.timestamp,
                                                                 msg.id,
                                                                 msg.len,
                                                                 imuData.dataStructure.accel[0],
                                                                 imuData.dataStructure.accel[1],
                                                                 imuData.dataStructure.accel[2]);
  Serial.print(buffer512);
  // gyro X/Y/Z follow the accel, deg/s
  if (extended)
  {
    float gyro[3];
    memcpy(gyro, &msg.buf[IMU_SIZE], sizeof(gyro));
    sprintf(buffer512, "\tGYRO\t%0.3f\t%0.3f\t%0.3f", gyro[0], gyro[1], gyro[2]);
    Serial.print(buffer512);
  }
  Serial.println();
}
//
// GPS node message
//
void CAN_GPSMessage(const CANFD_message_t &msg) 
{
  // extended: 38 bytes on 0x50 (CAN FD sends it as 48), or id 0x51
  bool extended = (msg.id == 0x51) || (msg.len >= 38);
  if(!displayMessages[extended ? DISPLAY_GPS_EX : 3])
  {
    return;
  }
  //for(int idx = 0; idx < GPS_SIZE; idx++)
  //{
  //  gpsData.dataBuffer[idx] = msg.buf[idx];
  //}
  memcpy(gpsData.dataBuffer, msg.buf, GPS_SIZE);
  sprintf(buffer512, "%08d\t0x%02X\tGPS\t%02d/%02d/%04d %02d:%02d:%02d\tLAT\t%d\tLONG\t%d\tCOURSE\t%d\tSPEED\t%d\tSIV\t%d", gpsData.dataStructure.timestamp,
                                                                                                                 msg.id,
                                                                                                                 gpsData.dataStructure.gpsDay,
                                                                                                                 gpsData.dataStructure.gpsMonth,
                                                                                                                 gpsData.dataStructure.gpsYear,
                                                                                                                 gpsData.dataStructure.gpsHour,
                                                                                                                 gpsData.dataStructure.gpsMinute,
                                                                                                                 gpsData.dataStructure.gpsSecond,
                                                                                                                 gpsData.dataStructure.latitude,
                                                                                                                 gpsData.dataStructure.longitude,
                                                                                                                 gpsData.dataStructure.course,
                                                                                                                 gpsData.dataStructure.speed,   // mm/s
                                                                                                                 gpsData.dataStructure.SIV);
  Serial.print(buffer512);
  // time of week (ms), horizontal accuracy (mm), fix type and RTK state follow the basic fields
  if (extended)
  {
    uint32_t iTOW, hAcc;
    memcpy(&iTOW, &msg.buf[GPS_SIZE], 4);
    memcpy(&hAcc, &msg.buf[GPS_SIZE + 4], 4);
    sprintf(buffer512, "\tTOW\t%lu\tHACC\t%lu\tFIX\t%d\tRTK\t%d", (unsigned long)iTOW, (unsigned long)hAcc,
            msg.buf[GPS_SIZE + 8], msg.buf[GPS_SIZE + 9]);
    Serial.print(buffer512);
  }
  Serial.println();
}
//
// IR tire temp node message
//
void CAN_TireIRMessage(const CANFD_message_t &msg) 
{
  if(!displayMessages[4])
  {
    return;
  }
  for(int idx = 0; idx < IR_SIZE; idx++)
  {
    irData.dataBuffer[idx] = msg.buf[idx];
  }
  sprintf(buffer512, "%08d\t0x%02X\tTireIR\t", irData.dataStructure.timestamp,
                                               msg.id);
  Serial.print(buffer512);
  for(int bufIdx = 0; bufIdx < 60; bufIdx++)
  {
    Serial.print(irData.dataStructure.tempVal[bufIdx] + 50);
    Serial.print(" ");
  }
  Serial.println();
}
//
// Wheel speed message (note 1 canID per wheel, 0x80-0x83, 0x84-0x87 etc...)
//
void CAN_WheelSpeedMessage(const CANFD_message_t &msg) 
{
  if(!displayMessages[5])
  {
    return;
  }
 
  for(int idx = 0; idx < SPEED_SIZE; idx++)
  {
    speedData.dataBuffer[idx] = msg.buf[idx];
  }
  sprintf(buffer512, "%08d\t0x%02X\tSpeed\t%05d", speedData.dataStructure.timestamp,
                                                  msg.id,
                                                  speedData.dataStructure.speedVal);
  Serial.println(buffer512);
}
//
// Shock position message (undefined)
//
void CAN_ShockMessage(const CANFD_message_t &msg) 
{
  if(!displayMessages[6])
  {
    return;
  }
  
  Serial.print(millis());
  Serial.print("\tShock");
  Serial.print("\tID\t"); Serial.print(msg.id, HEX);
  Serial.print("\tMB\t"); Serial.print(msg.mb);
  Serial.print("\tOVERRUN\t"); Serial.print(msg.flags.overrun);
  Serial.print("\tLEN\t"); Serial.print(msg.len);
  Serial.print("\tEXT\t"); Serial.print(msg.flags.extended);
  Serial.print("\tTS\t"); Serial.print(msg.timestamp);
  Serial.print("\tBuffer");
  for ( uint8_t i = 0; i < msg.len; i++ ) 
  {
    Serial.print("\t");
    Serial.print(msg.buf[i], HEX); 
  } 
  Serial.println();
}
//
// RPM message (undefined)
//
void CAN_RPMMessage(const CANFD_message_t &msg) 
{
  if(!displayMessages[7])
  {
    return;
  }
  
  Serial.print(millis());
  Serial.print("\tRPM    ");
  Serial.print("\tID\t"); Serial.print(msg.id, HEX);
  Serial.print("\tMB\t"); Serial.print(msg.mb);
  Serial.print("\tOVERRUN\t"); Serial.print(msg.flags.overrun);
  Serial.print("\tLEN\t"); Serial.print(msg.len);
  Serial.print("\tEXT\t"); Serial.print(msg.flags.extended);
  Serial.print("\tTS\t"); Serial.print(msg.timestamp);
  Serial.print("\tBuffer");
  for ( uint8_t i = 0; i < msg.len; i++ ) 
  {
    Serial.print("\t");
    Serial.print(msg.buf[i], HEX); 
  } 
  Serial.println();
}
//
// CAN transfer message  (undefined)
//
void CAN_CANMessage(const CANFD_message_t &msg) 
{
  if(!displayMessages[8])
  {
    return;
  }
  
  Serial.print(millis());
  Serial.print("\tCANxfer");
  Serial.print("\tID\t"); Serial.print(msg.id, HEX);
  Serial.print("\tMB\t"); Serial.print(msg.mb);
  Serial.print("\tOVERRUN\t"); Serial.print(msg.flags.overrun);
  Serial.print("\tLEN\t"); Serial.print(msg.len);
  Serial.print("\tEXT\t"); Serial.print(msg.flags.extended);
  Serial.print("\tTS\t"); Serial.print(msg.timestamp);
  Serial.print("\tBuffer");
  for ( uint8_t i = 0; i < msg.len; i++ ) 
  {
    Serial.print("\t");
    Serial.print(msg.buf[i], HEX); 
  } 
  Serial.println();
}
//
// CAN transfer message  (undefined)
//
void CAN_HubMessage(const CANFD_message_t &msg)  
{
  if(!displayMessages[9])
  {
    return;
  }

  if(msg.buf[0] == 1)
  {
    timeStampData.byteBuffer[0] = msg.buf[1];
    timeStampData.byteBuffer[1] = msg.buf[2];
    timeStampData.byteBuffer[2] = msg.buf[3];
    timeStampData.byteBuffer[3] = msg.buf[4];
    sprintf(buffer512, "%08d\t0x%02X\tHub\tHeartbeat message\tLocal %d\tSource %d", timeStampData.timestamp,
                                                                                  msg.id,
                                                                                  millis(),
                                                                                  timeStampData.timestamp);
  }
  else
  {
    sprintf(buffer512, "%d\t0x%02X\tHub\t%Unknown message 0x%02X (local time)", millis(), 
                                                                                msg.buf[0]);
  }
  Serial.println(buffer512);      
}
//
// send a timestamp heartbeat
//
void sendHeartbeat() 
{
  CANFD_message_t msg;
  msg.len = 5;
  msg.id = CANID;
  msg.seq = 1;
  timeStampData.timestamp = millis();
  //#ifdef DEBUG_EXTRA_VERBOSE
  Serial.printf("%d\tHeartbeat\n", timeStampData.timestamp);
  //#endif
  msg.buf[0] = 1;
  for (uint8_t i = 1; i < msg.len; i++) {
    msg.buf[i] = timeStampData.byteBuffer[i - 1];
  }
  canFD.write(msg);  // write to can3 mailbox 12
}
///
/// TIME HANDLERS
///
///
/// get the Teensy RTC value
///
//time_t getTeensy4Time()
//{
//  return Teensy3Clock.get();
//}
