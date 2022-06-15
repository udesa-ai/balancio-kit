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
#include <WiFi.h>

const char* ssid = "*****";           // Complete with your network SSID (name)!
const char* password = "*****";       // Complete with your network password!
const uint16_t port = 8090;
const char * host = "192.168.10.108"; // Complete with your host's IP!
WiFiClient client;

float current_angle_data [1000];
float target_angle_data [1000];
float pwm_data [1000];
int iteration_counter = 0;

float currentAngle = 0.0, targetAngle = STATIC_ANGLE, angle_limit = 0.5;
float yaw = 0.0, targetYaw = 0.0, yawCommand = 0.0;
int pwmL = 0, pwmR = 0;
volatile bool controlFlag = false;
float fwd = 0;
float rot = 0;
int time_ctr = 0, time_ctr2 = 0;
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

    // Controllers init
    pitch_control = Controller::init_algo("PID", pitch_controller_data);
    yaw_control = Controller::init_algo("PID", yaw_controller_data);

    // Wifi init
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("...");
    }

    Serial.print(F("WiFi connected with IP: "));
    Serial.println(WiFi.localIP());
    while (!client.connect(host, port)) {

        Serial.println(F( "Connection to host failed" ));

        delay(1000);
    }

    Serial.println(F( "Connected to server" ));
}

// ISR at 1/LOOP_PERIOD Hz
void IRAM_ATTR onTime()
{
    controlFlag = true;
}

void loop()
{
    while (iteration_counter < 600) 
    {

        if (controlFlag)
        {
            if (iteration_counter > 400)
            {
                targetAngle = 0.04;
            }

            // Get pitch and yaw angles.
            currentAngle = updatePitch(currentAngle);
            yaw = updateYaw(yaw);

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
            rot = 0;  //rot_v.at(0);

            // Pass PWM commands to motors.
            L_motor(pwmL + int(rot)); // -255 to 255
            R_motor(pwmR - int(rot)); // -255 to 255

            // Print relevant data.
            if (true)
            {
                // Serial.print(F( "  targetAngle: " ));
                // Serial.print(targetAngle);
                // Serial.print(F( "  targetYaw: " ));
                // Serial.print(targetYaw);
                // Serial.print(F( " pitch: " ));
                // Serial.print(currentAngle);
                // Serial.print(F( "  PWM: " ));
                // Serial.print(pwmL);
                // Serial.print(F( " Loop time: " ));
                // Serial.println(micros() - time_ctr);
                // time_ctr = micros();

                // Logger
                current_angle_data[iteration_counter] = currentAngle;
                target_angle_data[iteration_counter] = targetAngle;
                pwm_data[iteration_counter] = pwmL;
                iteration_counter++;
            }
            controlFlag = false;
        }
        else
        {
            return;
        }
    }

    // Stop motors
    stop_motor();

    // Send data to server.
    // Send current angle data.
    client.print("|");
    for (int i = 0; i < iteration_counter; i++)
    {
        client.print(",");
        client.print(current_angle_data[i]);
        delay(1);
    }

    // Send target angle data.
    client.print("|");
    for (int i = 0; i < iteration_counter; i++)
    {
        client.print(",");
        client.print(target_angle_data[i]);
        delay(1);
    }

    // Send PWM data.
    client.print("|");
    for (int i = 0; i < iteration_counter; i++)
    {
        client.print(",");
        client.print(pwm_data[i]);
        delay(1);
    }


    client.stop();

    while(true){
    }
}
