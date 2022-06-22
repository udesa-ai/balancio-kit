// This code is based on Jeff Rowberg's MPU6050 library.

// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

#include "config.h"
#include "imu.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector

// Gyro, accel and euler vars
int16_t ax_b, ay_b, az_b, gx_b, gy_b, gz_b;

// Pitch calculator vars
float ay, az, gx, gz;
float accelPitch, pitch_deg, pitch_rad;
float tau = 0.98; // Complementary filter parameter.
float newYaw;

// Functions
void imu_setup(void)
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(SDA, SCL, 400000);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // supply your own offsets here.
  mpu.setXAccelOffset(X_ACCEL_OFFSET);
  mpu.setYAccelOffset(Y_ACCEL_OFFSET);
  mpu.setZAccelOffset(Z_ACCEL_OFFSET);
  mpu.setXGyroOffset(X_GYRO_OFFSET);
  mpu.setYGyroOffset(Y_GYRO_OFFSET);
  mpu.setZGyroOffset(Z_GYRO_OFFSET);
}

void getAccelGyro(float *ay, float *az, float *gx, float *gz)
{
  mpu.getMotion6(&ax_b, &ay_b, &az_b, &gx_b, &gy_b, &gz_b); // gyro (+/- 250 deg/s) accel (+/- 2g)
  ay[0] = 4 * (ay_b / 65535.0);                             // g
  az[0] = 4 * (az_b / 65535.0);                             // g
  gx[0] = 2 * 250 * (gx_b / (65535.0));                     // deg/s
  gz[0] = 2 * 250 * (gz_b / (65535.0));                     // deg/s
}

float getAccelPitch(){
  getAccelGyro(&ay, &az, &gx, &gz);
  float accel_pitch = atan2(ay, az);
  return accel_pitch;
}

float updatePitch(float currentAngle)
{
  // Convert pitch from radians to degrees.
  float currentAngle_deg = -currentAngle * RAD_TO_DEG;
  // Get imu data.
  getAccelGyro(&ay, &az, &gx, &gz);
  // Pitch estimation from accelerometer.
  accelPitch = getAccelPitch()* RAD_TO_DEG;
  // Complementary filter between acceleration and gyroscopic pitch estimation.
  pitch_deg = (tau) * (currentAngle_deg + (gx)*LOOP_PERIOD) + (1 - tau) * (accelPitch);
  // Degrees to radians.
  pitch_rad = -pitch_deg * DEG_TO_RAD;

  return pitch_rad;
}

float updateYaw(float currentYaw)
{
  // Get imu data.
  getAccelGyro(&ay, &az, &gx, &gz);
  // Yaw estimation integrating angular velocity.
  newYaw = currentYaw + gz * DEG_TO_RAD;

  return newYaw;
}
