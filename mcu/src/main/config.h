/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/

// Control algorithm type
#define CONTROL_ALGO "PID" // PID, RL, ...

// PID Constants for pitch control
#define KP 2000  // 2000     //1200    //1750
#define KI 22000 // 22000     //22000   //11000
#define KD 20.0  // 20.0     //17.0    //25.0

// P constant for yaw cotrol
#define KP_YAW 5.0
#define KI_YAW 0.0
#define KD_YAW 0.05

// RL parameters
#define NUMBER_OF_INPUTS 1
#define NUMBER_OF_OUTPUTS 2
#define RL_INPUT_NORMALIZATION 1.5708   // (in_norm = in / RL_INPUT_NORMALIZATION)   --- //1.5708  // 0.06519447
#define RL_OUTPUT_DENORMALIZATION 255.0 // (out = out_norm * RL_OUTPUT_DENORMALIZATION)
#define TENSOR_ARENA_SIZE 16 * 1024     // You may need to tweak this value.

// Loop period in seconds
#define LOOP_PERIOD 0.01 // 100 Hz

// Angle of (approximate) static equilibrium
#define STATIC_ANGLE -0.04 // Calibrated point

// IMU calibration parameters
#define X_ACCEL_OFFSET -1775
#define Y_ACCEL_OFFSET 756
#define Z_ACCEL_OFFSET 2706
#define X_GYRO_OFFSET 181
#define Y_GYRO_OFFSET 77
#define Z_GYRO_OFFSET 60
