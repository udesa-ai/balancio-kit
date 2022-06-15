/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/

#include "commander.h"
// #include <Ps3Controller.h>
#include "config.h"

// PS3 Controller related functions.

void ps3_setup(void)
{
    // Ps3.attachOnConnect(onConnect);
    // Ps3.begin("08:d2:3e:45:6f:18");
    Serial.println("Joystick Ready.");
}

void onConnect()
{
    Serial.println("Joystick Connected.");
}

float get_pitch_command(void)
{
    float fwd = 0; //-Ps3.data.analog.stick.ry / 128.0;
    float targetAngle = fwd * 0.05 + STATIC_ANGLE;
    return targetAngle;
}

float get_yaw_command(float prev_target)
{
    float yawCommand = 0; // Ps3.data.analog.stick.lx / 40.0;
    float targetYaw = prev_target - yawCommand;
    return targetYaw;
}

bool x_button_pressed(void)
{
    bool x_down = false; //Ps3.event.button_down.cross;
    return x_down;
}
