/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/
#include "BluetoothSerial.h"
#include "config.h"
BluetoothSerial SerialBT;

// PS3 Controller related functions.

#include "Arduino.h"

#include "App.h"
//#include <Ps3Controller.h>
#include "config.h"
//#include "InputDevice.h"
#include <string>
#include <cstdlib>
// PS3 Controller related functions.
App::App(){
  return;
}
void App::setup(){ 
    //Ps3.attachOnConnect(this->onConnect);
    SerialBT.begin("Balancio Bluetooth");
}


void App::parse_input(float pitch_yaw[2]){
    char mensajeApp[10] = {'0','0','0','0','0','0','0','0','0','0'}; // Creo stringbuilder donde voy a almacenar el mensaje recibido de la app
    char recibido = 'A'; // inicializo char para ir recibiendo los valores que envía la app
    int index = 0; // inicializo variable de donde agrego en el stringbuilder
    char type; 
    if (SerialBT.available()){
        type = SerialBT.read();

    }
    while (recibido != '-' && index<10){
        if (SerialBT.available()){
            recibido = SerialBT.read();
            mensajeApp[index]= recibido;
        }
        index++;
    }
    //me fijo si tengo que actualizar el pitch o el yaw
    if (type== 'P'){ 
        pitch_yaw[0] = (atof(mensajeApp)-128.775)/126.225*3.2;
    }
    else if (type== 'Y'){
        pitch_yaw[1] =  (atof(mensajeApp)-128.775)/126.225;
    }
}

float App::get_pitch_command(float read){
    float targetAngle = read * 0.05 + STATIC_ANGLE;
    return targetAngle;
} 

float App::get_yaw_command(float prev_target,float read){
    float targetYaw = prev_target - read;
    return targetYaw;
} 
bool App:: x_button_pressed(void){
  //bool x_down = Ps3.event.button_down.cross;
  return false;
} 