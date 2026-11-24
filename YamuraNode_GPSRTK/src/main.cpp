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
// #define SAT_DIAGNOSTICS // uncomment for a bench build: 1 Hz solutions, and satellites
//                         // per constellation (used/tracked) added to the status line.
//                         // The F9P only sends NAV-SAT at 1 Hz, so not for logging laps.
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "../../YamuraLog_Common/src/YamuraLog_Structures.h"

#define STATUS_LED 13         // off: no RTK, blinking: RTK float, on: RTK fixed
#define GPS_AQ_LED 22         // on when there is a 3D (or better) fix
#ifdef SAT_DIAGNOSTICS
#define NAV_RATE_HZ 1         // solutions per second (NAV-SAT needs 1 Hz)
#else
#define NAV_RATE_HZ 10        // solutions per second
#endif
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
#ifdef SAT_DIAGNOSTICS
// satellites per constellation from NAV-SAT: used in the solution / tracked (signal present)
enum { CONST_GPS, CONST_GLO, CONST_GAL, CONST_BDS, CONST_COUNT };
uint8_t satsUsed[CONST_COUNT] = {0};
uint8_t satsTracked[CONST_COUNT] = {0};
void NavSatReceived(UBX_NAV_SAT_data_t *navSat);
// signals per band from NAV-SIG: used / tracked (code locked), mean C/N0 of tracked
// signals, and how many signals use corrections (RTCM pseudorange or carrier)
enum { BAND_L1, BAND_L2, BAND_COUNT };
uint8_t sigUsed[BAND_COUNT] = {0};
uint8_t sigTracked[BAND_COUNT] = {0};
uint8_t sigCn0[BAND_COUNT] = {0};
uint8_t sigCorrected = 0;
void NavSigReceived(UBX_NAV_SIG_data_t *navSig);
#endif

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
// a setting that the F9P didn't acknowledge is named on the USB serial port
static bool CheckSetting(bool result, const char *name)
{
  if (!result)
  {
    Serial.print("ZED-F9P setting failed: ");
    Serial.println(name);
  }
  return result;
}

void ConfigureGNSS()
{
  bool ok = true;
  // I2C: UBX only out (no NMEA noise); UBX + RTCM3 in (RTCM3 for USB-forwarded corrections)
  ok &= CheckSetting(gnssDevice.setI2COutput(COM_TYPE_UBX), "I2C output");
  ok &= CheckSetting(gnssDevice.setI2CInput(COM_TYPE_UBX | COM_TYPE_RTCM3), "I2C input");
  // UART2: corrections from the radio
  ok &= CheckSetting(gnssDevice.setVal32(UBLOX_CFG_UART2_BAUDRATE, RTCM_UART2_BAUD), "UART2 baud");
  ok &= CheckSetting(gnssDevice.setUART2Input(COM_TYPE_RTCM3), "UART2 input");
  ok &= CheckSetting(gnssDevice.setUART2Output(0), "UART2 output");
  // constellations: GPS + GLONASS + Galileo. The MDOT CORS stream carries GPS and
  // GLONASS corrections, so those two give RTK; Galileo adds satellites to the
  // standalone solution. BeiDou is off: nothing sends corrections for it, and three
  // constellations keep the solution rate up. If the measured rate (status line)
  // stays under NAV_RATE_HZ, disable Galileo too.
  ok &= CheckSetting(gnssDevice.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS), "GPS on");
  ok &= CheckSetting(gnssDevice.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS), "GLONASS on");
  ok &= CheckSetting(gnssDevice.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO), "Galileo on");
  ok &= CheckSetting(gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU), "BeiDou off");
  ok &= CheckSetting(gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS), "SBAS off");
  ok &= CheckSetting(gnssDevice.enableGNSS(false, SFE_UBLOX_GNSS_ID_QZSS), "QZSS off");
  delay(2000); // a constellation change restarts the GNSS - give it time
  ok &= CheckSetting(gnssDevice.setDynamicModel(DYN_MODEL_AUTOMOTIVE), "dynamic model");
  ok &= CheckSetting(gnssDevice.setNavigationFrequency(NAV_RATE_HZ), "navigation rate");
  ok &= CheckSetting(gnssDevice.setAutoPVT(true), "auto NAV-PVT");
  // NAV-SAT / NAV-SIG: the F9P only sends them at a 1 Hz solution rate (tested
  // 2026-11-24: nothing at 5 or 10 Hz). The SAT_DIAGNOSTICS build runs at 1 Hz and
  // turns NAV-SAT on; the normal build turns both off, which also clears any rate
  // left in battery-backed RAM by a diagnostics build.
#ifdef SAT_DIAGNOSTICS
  ok &= CheckSetting(gnssDevice.setAutoNAVSATcallbackPtr(&NavSatReceived), "auto NAV-SAT");
  ok &= CheckSetting(gnssDevice.setAutoNAVSIGcallbackPtr(&NavSigReceived), "auto NAV-SIG");
#else
  ok &= CheckSetting(gnssDevice.setVal8(UBLOX_CFG_MSGOUT_UBX_NAV_SAT_I2C, 0), "NAV-SAT off");
  ok &= CheckSetting(gnssDevice.setVal8(UBLOX_CFG_MSGOUT_UBX_NAV_SIG_I2C, 0), "NAV-SIG off");
#endif
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
#ifdef SAT_DIAGNOSTICS
  gnssDevice.checkUblox();       // take in NAV-SAT
  gnssDevice.checkCallbacks();   // runs NavSatReceived when one has arrived
#endif
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
#ifdef SAT_DIAGNOSTICS
  // more name/value pairs: readers that look fields up by name (GPS_Reader) ignore them
  sprintf(buffer512 + strlen(buffer512), "\tGPS\t%u/%u\tGLO\t%u/%u\tGAL\t%u/%u\tBDS\t%u/%u",
          satsUsed[CONST_GPS], satsTracked[CONST_GPS],
          satsUsed[CONST_GLO], satsTracked[CONST_GLO],
          satsUsed[CONST_GAL], satsTracked[CONST_GAL],
          satsUsed[CONST_BDS], satsTracked[CONST_BDS]);
  sprintf(buffer512 + strlen(buffer512), "\tL1\t%u/%u\tL1 C/N0\t%u dBHz\tL2\t%u/%u\tL2 C/N0\t%u dBHz\tCORR\t%u",
          sigUsed[BAND_L1], sigTracked[BAND_L1], sigCn0[BAND_L1],
          sigUsed[BAND_L2], sigTracked[BAND_L2], sigCn0[BAND_L2],
          sigCorrected);
#endif
  Serial.println(buffer512);
}
#ifdef SAT_DIAGNOSTICS
//
// NAV-SAT: count satellites per constellation, used in the solution and tracked.
// gnssId: 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS
//
void NavSatReceived(UBX_NAV_SAT_data_t *navSat)
{
  uint8_t used[CONST_COUNT] = {0};
  uint8_t tracked[CONST_COUNT] = {0};
  for (uint16_t i = 0; i < navSat->header.numSvs && i < UBX_NAV_SAT_MAX_BLOCKS; i++)
  {
    int c;
    switch (navSat->blocks[i].gnssId)
    {
      case 0:  c = CONST_GPS; break;
      case 6:  c = CONST_GLO; break;
      case 2:  c = CONST_GAL; break;
      case 3:  c = CONST_BDS; break;
      default: continue;
    }
    if (navSat->blocks[i].cno > 0)
    {
      tracked[c]++;
    }
    if (navSat->blocks[i].flags.bits.svUsed)
    {
      used[c]++;
    }
  }
  memcpy(satsUsed, used, sizeof(used));
  memcpy(satsTracked, tracked, sizeof(tracked));
}
//
// NAV-SIG: signals per band. L2 here means the second frequency: GPS L2C (sigId 3, 4),
// GLONASS L2OF (2), Galileo E5b (5, 6), BeiDou B2I (2, 3); everything else is L1/E1/B1.
// sigFlags bits (u-blox interface description): 0-1 health, 2 prSmoothed, 3 prUsed,
// 4 crUsed, 5 doUsed, 6 prCorrUsed, 7 crCorrUsed. Read from the raw word, because the
// library's bitfield gives health only one bit.
//
void NavSigReceived(UBX_NAV_SIG_data_t *navSig)
{
  uint8_t used[BAND_COUNT] = {0};
  uint8_t tracked[BAND_COUNT] = {0};
  uint16_t cn0Sum[BAND_COUNT] = {0};
  uint8_t corrected = 0;
  for (uint16_t i = 0; i < navSig->header.numSigs && i < UBX_NAV_SIG_MAX_BLOCKS; i++)
  {
    const UBX_NAV_SIG_block_t &s = navSig->blocks[i];
    bool second;
    switch (s.gnssId)
    {
      case 0:  second = (s.sigId == 3 || s.sigId == 4); break;   // GPS
      case 6:  second = (s.sigId == 2); break;                    // GLONASS
      case 2:  second = (s.sigId == 5 || s.sigId == 6); break;   // Galileo
      case 3:  second = (s.sigId == 2 || s.sigId == 3); break;   // BeiDou
      default: continue;
    }
    int b = second ? BAND_L2 : BAND_L1;
    uint16_t flags = s.sigFlags.all;
    if (s.qualityInd >= 4)
    {
      tracked[b]++;
      cn0Sum[b] += s.cno;
    }
    if (flags & (1 << 3))
    {
      used[b]++;
    }
    if (flags & ((1 << 6) | (1 << 7)))
    {
      corrected++;
    }
  }
  for (int b = 0; b < BAND_COUNT; b++)
  {
    sigUsed[b] = used[b];
    sigTracked[b] = tracked[b];
    sigCn0[b] = tracked[b] ? (uint8_t)(cn0Sum[b] / tracked[b]) : 0;
  }
  sigCorrected = corrected;
}
#endif

const char* CarrSolnText(uint8_t carrSoln)
{
  switch (carrSoln)
  {
    case 1:  return "float";
    case 2:  return "fixed";
    default: return "none";
  }
}
