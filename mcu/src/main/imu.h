/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/

// Defines
#define SDA 26
#define SCL 27

// Global function prototypes
void imu_setup(void);
void imu_setup_dmp(void);
void getEulerAngles_dmp(float *angles);
void getAccelGyro(float *ay, float *az, float *gx, float *gz);

/**
 * Estimate robot's pitch angle (in radians) based on accelerometer and
 * gyroscope data, using a complementary filter.
 *
 * @param  {float} currentAngle : Current pitch angle (in radians).
 * @return {float}              : New updated pitch angle (in radians).
 */
float updatePitch(float currentAngle);

/**
 * Estimate robot's yaw (in radians), integrating the gyroscope data.
 * 
 * @param  {float} currentYaw : Current yaw angle (in radians).
 * @return {float}            : New updated yaw angle (in radians).
 */
float updateYaw(float currentYaw);
