#include "commander.h"
#include <Ps3Controller.h>
#include "config.h"


// PS3 Controller related functions.

void ps3_setup(void){
//    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
//    Ps3.begin("8c:7c:b5:97:58:48");
    Ps3.begin("08:d2:3e:45:6f:18");

    Serial.println("Joystick Ready.");
}

void onConnect(){
    Serial.println("Joystick Connected.");
}

float get_pitch_command(void){
    float fwd = -Ps3.data.analog.stick.ry/128.0;
    float targetAngle = fwd*0.05 + STATIC_ANGLE;
    return targetAngle;
}

float get_yaw_command(float prev_target){
    float yawCommand = Ps3.data.analog.stick.lx/40.0;
    float targetYaw = prev_target - yawCommand;
    return targetYaw;
}

bool x_button_pressed(void)
{
    bool x_down = Ps3.event.button_down.cross;
    return x_down;
}