#ifndef YAMURALOG_COMMON_H
#define YAMURALOG_COMMON_H
#include <Arduino.h>
//
// YamuraLog V5 (sensor data structures)
//
// Brian B. Smith
// brianbsmith.com
// February 2022
//
//  MIT License
//  Copyright (c) 2022 Brian B Smith - info@brianbsmith.com
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
// 
// 
// Nodes use 11 bit CAN identifiers
//          1
// 01234567890
// -------      7 bit node type   (0x00 to 0x3F)
//        ----  4 bit node number (0x00 to 0x0F)
// hub gets lowest possible number so it has bus priority
// YamuraLog device IDs (all use standard 11 bit IDs)
// HUB       0000001 0000      (only one allowed) 0x10  
// (0x20 was the Control box - obsolete, removed)
// A2D       0000011 0000-1111                    0x30-0x3F (this node)
// IMU       0000100 0000      (only one allowed) 0x40
// GPS       0000101 0000      (only one allowed) 0x50
// Tire      0000110 0000-1111                    0x60-0x6F
// Shock     0000111 0000-1111                    0x70-0x7F
// WheelSpd  0001000 0000-1111                    0x80-0x8F
// RPM       0001001 0000      (only one allowed) 0x90
// CAN       0001010 0000      (only one allowed) 0xA0
//
// CAN setup
#define ARB_BAUD 500000
#define FD_BAUD  500000
//
// sample freq defines (value is sample rate in ms)
//
#define SAMPLE_1000Hz    1
#define SAMPLE_500Hz     2
#define SAMPLE_250Hz     4
#define SAMPLE_200Hz     5
#define SAMPLE_125Hz     8
#define SAMPLE_100Hz    10
#define SAMPLE_50Hz     20
#define SAMPLE_25Hz     40
#define SAMPLE_20Hz     50
#define SAMPLE_10Hz    100
#define SAMPLE_1Hz    1000
//
// timestamp structure (used by all)
//
union TimeStamp
{
  uint32_t timestamp;
  uint8_t byteBuffer[4];
} timeStampData;

//
// 0x10 HUB structures
// no structure, sends current millis value as heartbeat at regular intervals


//
// 0x30 A2D structures
//
#define AD_SIZE 21
struct __attribute__((packed)) AD_v2_Structure
{
  unsigned long timestamp;   // 1*4 =  4 bytes
  uint8_t digitalVal;        // 1 =    1 bytes (max 8 digital channels, 1 bit/channel)
  uint16_t analogVal[8];     // 8*2 = 16 bytes (max 8 analog channels)
                             // ------------
                             //       21 total bytes
};
union AD_v2_Data
{
  AD_v2_Structure dataStructure;
  uint8_t dataBuffer[AD_SIZE];
} adData;
//
// IMU structures
//
#define IMU_SIZE 16
struct __attribute__((packed)) IMU_Structure
{
  unsigned long timestamp;   // 1*4 =  4 bytes
  float accel[3];            // 3*4 = 12 bytes
                             // --------------
                             // 16 total bytes
};
union IMU_Data
{
  IMU_Structure dataStructure;
  uint8_t dataBuffer[IMU_SIZE];
} imuData;

//
// GPS structures
//
#define GPS_SIZE 28
struct __attribute__((packed)) GPS_Structure
{
  uint32_t timestamp;        // 4 bytes
  uint16_t gpsYear;          // 2
  uint8_t gpsMonth;          // 1
  uint8_t gpsDay;            // 1
  uint8_t gpsHour;           // 1
  uint8_t gpsMinute;         // 1
  uint8_t gpsSecond;         // 1
  int32_t latitude;          // 4
  int32_t longitude;         // 4
  int32_t course;            // 4
  int32_t speed;             // 4
  uint8_t SIV;               // 1
};
union GPS_Data
{
  GPS_Structure dataStructure;
  uint8_t dataBuffer[GPS_SIZE];
} gpsData;

//
// Tire temp structures
//
#define IR_SIZE 64
struct __attribute__((packed)) IR_Structure
{
  unsigned long timestamp;   //  1*4 =  4 bytes
  uint8_t tempVal[60];       // 60*1 = 60 bytes
                             // ------------
                             // 64 total bytes
};
union IR_Data
{
  IR_Structure dataStructure;
  uint8_t dataBuffer[IR_SIZE];
} irData;

//
// Shock structures
// not defined yet

//
// WheelSpd structures
//
#define SPEED_SIZE 8
struct __attribute__((packed)) SPEED_Structure
{
  unsigned long timestamp;   // 1*4 =  4 bytes
  unsigned long speedVal;    // 1*4 =  4 bytes
                             // ------------
                             //        8 total bytes
};
union SPEED_Data
{
  SPEED_Structure dataStructure;
  uint8_t dataBuffer[SPEED_SIZE];
} speedData;

//
// 0x90 RPM structures
// not defined yet

//
// 0xA0 CAN structures
// not defined yet

#endif