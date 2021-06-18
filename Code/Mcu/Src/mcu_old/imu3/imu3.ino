/*
   This codes retrieves roll, pitch, yaw angles from DMP of
   MPU6050 without the use of interrupt

   MPU6050 I2Cdevlib's default data rate for DMP is 100 Hz
   but DMP itself works @200Hz always

*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"

MPU6050 imu;

// MPU6050 control/status variables
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(57600);
  Serial.println(F("Initializing MPU6050..."));
  // powering on the device and setting accelRange to +- 2g and gyroRange ro +- 250 dps
  imu.initialize();

  Serial.println(F("Testing MPU6050 connection..."));
  Serial.println(imu.testConnection() ? F("MPU connection successful") : F("MPU connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = imu.dmpInitialize();

  // Offset values from running the example sketch IMU_ZERO
  imu.setXAccelOffset(5498);
  imu.setYAccelOffset(-5082);
  imu.setZAccelOffset(14054);
  imu.setXGyroOffset(114);
  imu.setYGyroOffset(545);
  imu.setZGyroOffset(-21);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    imu.setDMPEnabled(true);
    dmpReady = true;

    packetSize = imu.dmpGetFIFOPacketSize();
  } else {
    // Error!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print(F("DMP Initialization failed (code )"));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

}
unsigned long t1, t2;
void loop() {
  if (!dmpReady) {
    Serial.println(F("DMP was not ready!"));
    return;
  }

  t1 = micros();
  fifoCount = imu.getFIFOCount();
  if (fifoCount == 1024) {
    imu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  if (fifoCount >= 42) {
    imu.getFIFOBytes(fifoBuffer, packetSize);
    imu.resetFIFO();
  }
  t2 = micros();

  imu.dmpGetQuaternion(&q, fifoBuffer);
  imu.dmpGetGravity(&gravity, &q);
  imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[2] * 180 / M_PI);
  Serial.print("\t");
  Serial.print("time = ");
  Serial.println(t2 - t1);
}
