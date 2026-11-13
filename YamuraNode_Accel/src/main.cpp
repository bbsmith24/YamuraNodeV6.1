// YamuraLog V6.1 (Teensy 4.0 CAN FD IMU (accel + gyro) node)
// Teensy 4.0/CAN FD
//
// Brian B. Smith
// brianbsmith.com
// February 2022 updated for Visual Code and PlatformIO November 2026
// Part of the YamuraLog project
//
#define VERBOSE // uncomment for output to serial
#include <FlexCAN_T4.h>
#include <Wire.h>             // I2C for accelerometer
#include "../../YamuraLog_Common/src/YamuraLog_Structures.h"

#define MPU6050
#ifdef LSM9DS1
#include <SparkFunLSM9DS1.h>  // imu
// LSM9DS1 imu object
LSM9DS1 imu;
#endif
#ifdef MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// LSM9DS1 imu object
Adafruit_MPU6050 imu;
#endif 

#define STATUS_LED 13
#define CANID  0x040
// accel + gyro message - same CAN id (0x40) as the accel-only message; the hub tells
// them apart by length (16 = accel only, 28 = accel + gyro)
#define IMU_GYRO_SIZE 28
struct __attribute__((packed)) IMU_Gyro_Structure
{
  unsigned long timestamp;   // 1*4 =  4 bytes
  float accel[3];            // 3*4 = 12 bytes, g
  float gyro[3];             // 3*4 = 12 bytes, deg/s
                             // --------------
                             // 28 total bytes
};
union IMU_Gyro_Data
{
  IMU_Gyro_Structure dataStructure;
  uint8_t dataBuffer[IMU_GYRO_SIZE];
} imuGyroData;
#define RAD_TO_DEG_F 57.29577951F
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
// forward declarations
void SendSensorValues();

//
//
//
void setup(void) 
{
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  Serial.begin(115200);
  delay(1000);
  #ifdef VERBOSE
  Serial.println("YamuraLog IMU Node/Teensy 4.0");
  #endif
  digitalWrite(STATUS_LED, LOW);
  //
  // I2C setup
  //
  Wire.begin();
  Wire.setClock(400000);
  #ifdef VERBOSE
  Serial.println(F("I2C ready"));
  #endif
  // start I2C and set up sensors
  // start sensors, blink if not found
  while (!imu.begin())
  {
    #ifdef VERBOSE
      #ifdef LSM9DS1
      Serial.print("LSM9DS1 ");
      #endif
      #ifdef MPU6050
      Serial.print("MPU6050 ");
      #endif
      Serial.println("not found on I2C bus - retry in 5 sec");    
    #endif
    delay(5000);
  }
  #ifdef LSM9DS1
  imu.settings.gyro.enabled = true;  // Enable the gyro
  imu.settings.gyro.scale = 500;     // +/-500 deg/s
  // sets accelerometer and gyro sample rates if enabled
  // imu.settings.gyro.sampleRate = 6; // 952Hz sample rate for gyro and accelerometer
  // enable accelerometer
  imu.settings.accel.enabled = true; // Enable accelerometer
  // accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 4; // Set accel scale to +/-4g.
  	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 3; // Set accel to 952Hz
  #endif
  #ifdef MPU6050
  imu.setAccelerometerRange(MPU6050_RANGE_4_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  #endif

  #ifdef VERBOSE
    #ifdef LSM9DS1
    Serial.print("LSM9DS1 (imu) initialized on I2C");
    #endif
    #ifdef LSM9DS1
    Serial.print("MPU6050 (imu) initialized on I2C");
    #endif
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
  // sensor value send timer
  timer.begin(SendSensorValues, (sampleDelay * 1000));
}
//
// send sensor data
//
void SendSensorValues()
{
  // read the imu
  #ifdef LSM9DS1
  imu.readAccel();
  imu.readGyro();
  imuGyroData.dataStructure.timestamp = millis()+deltaTime;
  imuGyroData.dataStructure.accel[0] = imu.calcAccel(imu.ax);
  imuGyroData.dataStructure.accel[1] = imu.calcAccel(imu.ay);
  imuGyroData.dataStructure.accel[2] = imu.calcAccel(imu.az);
  imuGyroData.dataStructure.gyro[0] = imu.calcGyro(imu.gx);   // deg/s
  imuGyroData.dataStructure.gyro[1] = imu.calcGyro(imu.gy);
  imuGyroData.dataStructure.gyro[2] = imu.calcGyro(imu.gz);
  #endif
  #ifdef MPU6050
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);
  imuGyroData.dataStructure.timestamp = millis()+deltaTime;
  imuGyroData.dataStructure.accel[0] = (float)(a.acceleration.x/9.81F);
  imuGyroData.dataStructure.accel[1] = (float)(a.acceleration.y/9.81F);
  imuGyroData.dataStructure.accel[2] = (float)(a.acceleration.z/9.81F);
  imuGyroData.dataStructure.gyro[0] = g.gyro.x * RAD_TO_DEG_F;   // rad/s -> deg/s
  imuGyroData.dataStructure.gyro[1] = g.gyro.y * RAD_TO_DEG_F;
  imuGyroData.dataStructure.gyro[2] = g.gyro.z * RAD_TO_DEG_F;
  #endif


  CANFD_message_t msg;
  msg.len = IMU_GYRO_SIZE;
  msg.id = CANID;
  msg.seq = 1;
  #ifdef EXTRA_VERBOSE
  sprintf(buffer512, "Send\t%ld\tid\t0x%02X\tIMU\tXYZ\t%0.4f\t%0.4f\t%0.4f\tGYRO\t%0.3f\t%0.3f\t%0.3f",
          imuGyroData.dataStructure.timestamp,
          (int)msg.id,
          imuGyroData.dataStructure.accel[0],
          imuGyroData.dataStructure.accel[1],
          imuGyroData.dataStructure.accel[2],
          imuGyroData.dataStructure.gyro[0],
          imuGyroData.dataStructure.gyro[1],
          imuGyroData.dataStructure.gyro[2]);
  Serial.println(buffer512);
  #endif
  // load the message payload (copy from dataBuffer)
  for(int idx = 0; idx < IMU_GYRO_SIZE; idx++)
  {
  	msg.buf[idx] = imuGyroData.dataBuffer[idx];
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
