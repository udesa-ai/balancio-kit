#include "config.h"
#include "dc_motor.h"
#include "controllers.h"
#include "imu.h"
#include "timer.h"
#include "Wire.h"
#include <Ps3Controller.h>



float currentAngle=0.0, targetAngle=0.0, angle_limit=0.5;
float yaw=0.0, targetYaw=0.0, yawCommand=0.0;
int pwmL=0, pwmR=0;
volatile bool controlFlag=false;
float fwd=0;
float rot=0;
int time_ctr=0;
bool stop_command=true;
bool x_down;

// PID
// PID pitch_pid = PID(KP, KI, KD, 5.0, 255.0);
// RL
RlSingleInput pitch_rl = RlSingleInput();

PID yaw_pid = PID(KP_YAW, KI_YAW, KD_YAW, 5.0, 255.0);

void setup() {
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
}


// ISR at 1/LOOP_PERIOD Hz
void IRAM_ATTR onTime() {
  controlFlag=true;
}


void loop() {

  if (controlFlag){ 

    x_down = Ps3.event.button_down.cross;
    if (x_down){
      stop_command = abs(stop_command -1); 
    }

    if (true){

      currentAngle = updatePitch(currentAngle);
      yaw = updateYaw(yaw);

      // Pitch control
      // PID
      // pitch_pid.update(currentAngle, targetAngle);
      // pwmL = pitch_pid.output;
      // pwmR = pitch_pid.output;

      // RL
      pitch_rl.update(currentAngle, targetAngle);
      pwmL = pitch_rl.pwmL;
      pwmR = pitch_rl.pwmR;
      
      // Yaw control
      yaw_pid.update(yaw, targetYaw);
      rot = yaw_pid.output;

      if ( (currentAngle > angle_limit) || (currentAngle < -angle_limit) ){
        pwmL = 0;
        pwmR = 0;
        rot = 0.0;
      }
      
      L_motor(pwmL + int(rot));
      R_motor(pwmR - int(rot));
  
      if (true){
        Serial.print("fwd: ");
        Serial.print(fwd);
        Serial.print(" rot: ");
        Serial.print(rot);
        Serial.print("  Pitch:  ");
        Serial.print(currentAngle, 4);
        Serial.print("  PWM:  ");
        Serial.print(pwmL);
        Serial.print("  Loop time: ");
        time_ctr = micros()-time_ctr;
        Serial.println(time_ctr);
        time_ctr = micros();    
      }
      
    }
    else{
      stop_motor(); 
    }
      
    controlFlag=false;
        
  }
  else{
    return;
  }


  
  fwd = -Ps3.data.analog.stick.ry/128.0;
  yawCommand = Ps3.data.analog.stick.lx/40.0;
  targetYaw -= yawCommand;
  
  targetAngle = fwd*0.05 + STATIC_ANGLE;


}


void ps3_setup(void){
//    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
//    Ps3.begin("8c:7c:b5:97:58:48");
    Ps3.begin("08:d2:3e:45:6f:18");

    Serial.println("Ready.");
}

void onConnect(){
    Serial.println("Connected.");
}
