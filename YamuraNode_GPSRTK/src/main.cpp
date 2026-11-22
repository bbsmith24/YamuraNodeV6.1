// YamuraLog V6.1 (Teensy 4.0 GPS-RTK node)
// Teensy 4.0 + SparkFun GPS-RTK-SMA Breakout (u-blox ZED-F9P)
//
// Brian B. Smith
// brianbsmith.com
// November 2026
// Part of the YamuraLog project
//
// Stage 1: GPS-RTK module only, no CAN yet. Each fix is packed into the same
// 38-byte extended GPS record the GPS node sends on CAN id 0x50, and printed
// to USB serial, so adding CAN later is just a write of gpsExData.
//
// Wiring (Teensy 4.0 <-> ZED-F9P breakout):
//   3.3V -> 3V3, GND -> GND, pin 18 (SDA) -> SDA, pin 19 (SCL) -> SCL
//   (or a Qwiic cable to the breakout's Qwiic connector)
//   Correction radio TX -> breakout RX2 (UART2, RTCM3 in at RTCM_UART2_BAUD)
//
// RTCM corrections reach the F9P two ways:
//   1. a radio on the breakout's UART2 (the in-car setup) - no firmware involved
//   2. bytes sent to the Teensy's USB serial port are pushed to the F9P over
//      I2C (bench testing, e.g. str2str/RTKLIB from an NTRIP caster to the COM port)
//
#define DEBUG_VERBOSE // uncomment for a line per fix on USB serial
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "../../YamuraLog_Common/src/YamuraLog_Structures.h"

#define STATUS_LED 13         // off: no RTK, blinking: RTK float, on: RTK fixed
#define GPS_AQ_LED 22         // on when there is a 3D (or better) fix
#define NAV_RATE_HZ 10        // solutions per second
#define RTCM_UART2_BAUD 57600 // must match the correction radio's serial baud
#define CANID 0x050           // same id as the GPS node, for when CAN is added

// extended GPS record - same layout as YamuraNode_GPS sends on CAN 0x50
// (the hub tells it from the 28-byte basic record by length and logs it as record 0x51)
#define GPS_EX_SIZE 38
struct __attribute__((packed)) GPS_Ex_Structure
{
  GPS_Structure base;        // 28 bytes - same layout as the basic message
  uint32_t iTOW;             //  4 bytes - GPS time of week of the fix, ms
  uint32_t hAcc;             //  4 bytes - horizontal accuracy estimate, mm
  uint8_t fixType;           //  1 byte  - 0 none, 2 2D, 3 3D, 4 GNSS + dead reckoning, 5 time only
  uint8_t carrSoln;          //  1 byte  - RTK: 0 none, 1 float, 2 fixed
                             // --------
                             // 38 total bytes
};
union GPS_Ex_Data
{
  GPS_Ex_Structure dataStructure;
  uint8_t dataBuffer[GPS_EX_SIZE];
} gpsExData;

SFE_UBLOX_GNSS gnssDevice;
char buffer512[512];
// hub time offset - stays 0 until CAN (and the hub heartbeat) is added
uint32_t deltaTime = 0;

// once-a-second status
uint32_t lastStatus = 0;
uint16_t fixesThisSecond = 0;
uint32_t rtcmBytesThisSecond = 0;
uint8_t rtcmBuffer[256];

// forward declarations
void ConfigureGNSS();
void ReadFix();
void ForwardUsbCorrections();
void UpdateLeds();
void PrintFix();
void PrintStatus();
const char* CarrSolnText(uint8_t carrSoln);

//
//
//
void setup(void)
{
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  pinMode(GPS_AQ_LED, OUTPUT);
  digitalWrite(GPS_AQ_LED, HIGH);
  Serial.begin(115200);
  delay(1000);
  Serial.println("YamuraLog GPS-RTK Node/Teensy 4.0 (ZED-F9P)");
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(GPS_AQ_LED, LOW);

  Wire.begin();
  Wire.setClock(400000);
  while (gnssDevice.begin(Wire) == false)
  {
    Serial.println("ZED-F9P not detected on I2C. Retry in 5 sec");
    delay(5000);
  }
  Serial.println("ZED-F9P detected on I2C bus");
  ConfigureGNSS();
  lastStatus = millis();
}
//
// configure the F9P - settings go to RAM and battery-backed RAM (the v3 library default)
//
void ConfigureGNSS()
{
  bool ok = true;
  // I2C: UBX only out (no NMEA noise); UBX + RTCM3 in (RTCM3 for USB-forwarded corrections)
  ok &= gnssDevice.setI2COutput(COM_TYPE_UBX);
  ok &= gnssDevice.setI2CInput(COM_TYPE_UBX | COM_TYPE_RTCM3);
  // UART2: corrections from the radio
  ok &= gnssDevice.setVal32(UBLOX_CFG_UART2_BAUDRATE, RTCM_UART2_BAUD);
  ok &= gnssDevice.setUART2Input(COM_TYPE_RTCM3);
  ok &= gnssDevice.setUART2Output(0);
  // constellations: GPS + Galileo + BeiDou. GLONASS is off so 10 Hz RTK is
  // reachable and the base doesn't need to send GLONASS corrections (1020 / 1230).
  // If the measured rate (status line) stays under NAV_RATE_HZ, also disable BeiDou.
  ok &= gnssDevice.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
  ok &= gnssDevice.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
  ok &= gnssDevice.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
  ok &= gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
  ok &= gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS);
  ok &= gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_QZSS);
  delay(2000); // a constellation change restarts the GNSS - give it time
  ok &= gnssDevice.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
  ok &= gnssDevice.setNavigationFrequency(NAV_RATE_HZ);
  ok &= gnssDevice.setAutoPVT(true);
  sprintf(buffer512, "ZED-F9P configured%s - %d Hz, UART2 RTCM3 in at %d baud",
          ok ? "" : " (some settings failed)",
          gnssDevice.getNavigationFrequency(),
          RTCM_UART2_BAUD);
  Serial.println(buffer512);
}
//
// loop - read fixes, forward any USB corrections, report once a second
//
void loop()
{
  ForwardUsbCorrections();
  if (gnssDevice.getPVT() && (gnssDevice.getInvalidLlh() == false))
  {
    ReadFix();
    fixesThisSecond++;
    #ifdef DEBUG_VERBOSE
    PrintFix();
    #endif
    // CAN goes here later: send gpsExData.dataBuffer (GPS_EX_SIZE bytes) on CANID
  }
  UpdateLeds();
  if (millis() - lastStatus >= 1000)
  {
    PrintStatus();
    lastStatus += 1000;
    fixesThisSecond = 0;
    rtcmBytesThisSecond = 0;
  }
}
//
// copy the latest NAV-PVT solution into the extended GPS record
//
void ReadFix()
{
  GPS_Structure& base = gpsExData.dataStructure.base;
  base.timestamp = millis() + deltaTime;
  base.gpsYear =   gnssDevice.getYear();
  base.gpsMonth =  gnssDevice.getMonth();
  base.gpsDay =    gnssDevice.getDay();
  base.gpsHour =   gnssDevice.getHour();
  base.gpsMinute = gnssDevice.getMinute();
  base.gpsSecond = gnssDevice.getSecond();
  base.latitude =  gnssDevice.getLatitude();
  base.longitude = gnssDevice.getLongitude();
  base.course =    gnssDevice.getHeading();
  base.speed =     gnssDevice.getGroundSpeed();
  base.SIV =       gnssDevice.getSIV();
  gpsExData.dataStructure.iTOW =     gnssDevice.getTimeOfWeek();
  gpsExData.dataStructure.hAcc =     gnssDevice.getHorizontalAccEst();
  gpsExData.dataStructure.fixType =  gnssDevice.getFixType();
  gpsExData.dataStructure.carrSoln = gnssDevice.getCarrierSolutionType();
}
//
// bytes arriving on USB serial are RTCM3 corrections - push them to the F9P over I2C
//
void ForwardUsbCorrections()
{
  int available = Serial.available();
  if (available <= 0)
  {
    return;
  }
  size_t count = Serial.readBytes((char*)rtcmBuffer, min((size_t)available, sizeof(rtcmBuffer)));
  if (count > 0)
  {
    gnssDevice.pushRawData(rtcmBuffer, count);
    rtcmBytesThisSecond += count;
  }
}
//
// GPS_AQ_LED: on with a 3D fix. STATUS_LED: off no RTK, 2 Hz blink RTK float, on RTK fixed
//
void UpdateLeds()
{
  uint8_t fixType = gpsExData.dataStructure.fixType;
  uint8_t carrSoln = gpsExData.dataStructure.carrSoln;
  digitalWrite(GPS_AQ_LED, (fixType == 3 || fixType == 4) ? HIGH : LOW);
  if (carrSoln == 2)
  {
    digitalWrite(STATUS_LED, HIGH);
  }
  else if (carrSoln == 1)
  {
    digitalWrite(STATUS_LED, (millis() / 250) % 2 ? HIGH : LOW);
  }
  else
  {
    digitalWrite(STATUS_LED, LOW);
  }
}
//
// one line per fix, in the same format YamuraNode_Analyze prints an extended GPS
// message (id 0x50, 38 bytes), so GPS_Reader and log tools read both the same way:
// millis  id  GPS  DD/MM/YYYY hh:mm:ss  LAT n  LONG n  COURSE n  SPEED n  SIV n  TOW n  HACC n  FIX n  RTK n
//
void PrintFix()
{
  const GPS_Structure& base = gpsExData.dataStructure.base;
  sprintf(buffer512, "%08lu\t0x%02X\tGPS\t%02d/%02d/%04d %02d:%02d:%02d\tLAT\t%ld\tLONG\t%ld\tCOURSE\t%ld\tSPEED\t%ld\tSIV\t%d"
                     "\tTOW\t%lu\tHACC\t%lu\tFIX\t%d\tRTK\t%d",
          (unsigned long)base.timestamp,
          CANID,
          base.gpsDay,
          base.gpsMonth,
          base.gpsYear,
          base.gpsHour,
          base.gpsMinute,
          base.gpsSecond,
          (long)base.latitude,
          (long)base.longitude,
          (long)base.course,
          (long)base.speed,   // mm/s
          base.SIV,
          (unsigned long)gpsExData.dataStructure.iTOW,
          (unsigned long)gpsExData.dataStructure.hAcc,
          gpsExData.dataStructure.fixType,
          gpsExData.dataStructure.carrSoln);
  Serial.println(buffer512);
}
//
// once a second: measured solution rate, fix quality, corrections forwarded from USB
//
void PrintStatus()
{
  sprintf(buffer512, "Status\trate\t%u Hz\tfix\t%d\tRTK\t%s\tSIV\t%d\thAcc\t%lu mm\tUSB RTCM\t%lu B/s",
          fixesThisSecond,
          gpsExData.dataStructure.fixType,
          CarrSolnText(gpsExData.dataStructure.carrSoln),
          gpsExData.dataStructure.base.SIV,
          (unsigned long)gpsExData.dataStructure.hAcc,
          (unsigned long)rtcmBytesThisSecond);
  Serial.println(buffer512);
}

const char* CarrSolnText(uint8_t carrSoln)
{
  switch (carrSoln)
  {
    case 1:  return "float";
    case 2:  return "fixed";
    default: return "none";
  }
}
