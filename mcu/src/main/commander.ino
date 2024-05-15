/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/
#include "Arduino.h"

#include "commander.h"
#include "config.h"

// PS3 Controller related functions.
PS3Joystick::PS3Joystick(){
  return;
}
void PS3Joystick::setup(){ 
    //Ps3.attachOnConnect(this->onConnect);
    Ps3.begin("08:d2:3e:45:6f:18");
    Serial.println("Joystick Ready.");
}

void PS3Joystick::parse_input(float fill_arr[2]){ 
    fill_arr[0] = -Ps3.data.analog.stick.ry / 128.0;
    fill_arr[1] =  Ps3.data.analog.stick.lx / 40.0;
}

float PS3Joystick::get_pitch_command(float read){
    float targetAngle = read * 0.05 + STATIC_ANGLE;
    return targetAngle;
} 

float PS3Joystick::get_yaw_command(float prev_target,float read){
    float targetYaw = prev_target - read;
    return targetYaw;
} 
bool PS3Joystick:: x_button_pressed(void){
  bool x_down = Ps3.event.button_down.cross;
  return x_down;
} 

