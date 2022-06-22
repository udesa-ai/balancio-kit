/*
  ============================================
  Balancio-Kit is placed under the MIT License
  Copyright (c) 2021 by Linar (UdeSA)

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
#include "dc_motor.h"
#include "controllers.h"
#include "imu.h"
#include "timer.h"
#include "Wire.h"
#include "commander.h"

float currentAngle, targetAngle = 0.0, angle_limit = 0.5;
float yaw = 0.0, targetYaw = 0.0, yawCommand = 0.0;
int pwmL = 0, pwmR = 0;
volatile bool controlFlag = false;
float fwd = 0;
float rot = 0;
int time_ctr1, time_ctr2;
int delay_startup_time = 2000;
bool stop_command = false;
bool x_down;

String control_algo(CONTROL_ALGO);
controller_data_t pitch_controller_data = {KP, KI, KD};
controller_data_t yaw_controller_data = {KP_YAW, KI_YAW, KD_YAW};

Controller *yaw_control;
Controller *pitch_control;
std::vector<float> pwm;
std::vector<float> rot_v;

void setup()
{
  // UART PC connection
  Serial.begin(115200);

  // Motor initialization
  motor_init();

  // IMU init
  imu_setup();

  // Bluetooth init
  ps3_setup();

  // Timer init. (ISR at 1/LOOP_PERIOD Hz)
  timer_init();
  time_ctr1 = micros();
  time_ctr2 = millis();

  // Controllers init
  pitch_control = Controller::init_algo(control_algo, pitch_controller_data);
  yaw_control = Controller::init_algo("PID", yaw_controller_data);

  // Angle initialization
  currentAngle = -getAccelPitch();
}

// ISR at 1/LOOP_PERIOD Hz
void IRAM_ATTR onTime()
{
  controlFlag = true;
}

void loop()
{

  if (controlFlag)
  {
    // Get if X button was pressed
    x_down = x_button_pressed();
    if (x_down and (millis() - time_ctr2) >= 500)
    {
      stop_command = abs(stop_command - 1);
      time_ctr2 = millis();
    }

    // Get pitch and yaw angles.
    currentAngle = updatePitch(currentAngle);
    yaw = updateYaw(yaw);

    if (stop_command || (currentAngle > angle_limit) || (currentAngle < -angle_limit))
    {
      // Stop motors
      stop_motor();

      // Reset yaw command
      targetYaw = yaw;

      // Reset PID
      if (control_algo.equals("PID"))
      {
        ((PID *)pitch_control)->reset(0.0);
      }

      ((PID *)yaw_control)->reset(targetYaw);
    }
    else
    {
      // Pitch control.
      if (control_algo.equals("PID"))
      {
        // PID control.
        pwm = pitch_control->update(currentAngle, targetAngle);
        pwmL = pwm.at(0);
        pwmR = pwm.at(0);
      }
      else if (control_algo.equals("RL"))
      {
        // RL control.
        pwm = pitch_control->update(currentAngle, targetAngle);
        pwmL = pwm.at(0);
        pwmR = pwm.at(1);
      }

      // Yaw control.
      rot_v = yaw_control->update(yaw, targetYaw);
      rot = rot_v.at(0);

      // Pass PWM commands to motors.
      L_motor(pwmL + int(rot)); // -255 to 255
      R_motor(pwmR - int(rot)); // -255 to 255


    }
    // Print relevant data.
    if (true)
    {
        Serial.print("  targetAngle: ");
        Serial.print(targetAngle);
        Serial.print("  targetYaw: ");
        Serial.print(targetYaw);
        Serial.print(" pitch: ");
        Serial.print(currentAngle);
        Serial.print("  PWM: ");
        Serial.print(pwmL);
        Serial.print(" Loop time: ");
        Serial.println(micros() - time_ctr1);
        time_ctr1 = micros();
    }
    controlFlag = false;
  }
  else
  {
    return;
  }

  // Get joystick commands.
  targetAngle = get_pitch_command();
  targetYaw = get_yaw_command(targetYaw);
}
