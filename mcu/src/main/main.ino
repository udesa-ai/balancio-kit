#include "config.h"
#include "dc_motor.h"
#include "controllers.h"
#include "imu.h"
#include "timer.h"
#include "Wire.h"
#include "commander.h"


float currentAngle=0.0, targetAngle=0.0, angle_limit=0.5;
float yaw=0.0, targetYaw=0.0, yawCommand=0.0;
int pwmL=0, pwmR=0;
volatile bool controlFlag=false;
float fwd=0;
float rot=0;
int time_ctr=0;
bool stop_command=true;
bool x_down;

String control_algo(CONTROL_ALGO);
PID* yaw_control;
Controller* pitch_control;
std::vector<float> pwm;
std::vector<float> rot_v;


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

  // Controllers init
  pitch_control = Controller::init_algo(control_algo);
  yaw_control = new PID(KP_YAW, KI_YAW, KD_YAW, 5.0);

}


// ISR at 1/LOOP_PERIOD Hz
void IRAM_ATTR onTime() {
  controlFlag=true;
}


void loop() {

  if (controlFlag){ 

    //x_down = Ps3.event.button_down.cross;
    //if (x_down){
      //stop_command = abs(stop_command -1); 
    //}

    if (true){

      // Get pitch and yaw angles.
      currentAngle = updatePitch(currentAngle);
      yaw = updateYaw(yaw);

      // Pitch control.
      if (control_algo.equals("PID")){
        // PID control.
        pwm = pitch_control->update(currentAngle, targetAngle);
        pwmL = pwm.at(0);
        pwmR = pwm.at(0);
      }
      else if (control_algo.equals("RL"))
      {
        // RL control.
        pwm = pitch_control->update(currentAngle, targetAngle);
        pwmL = 255.0 * pwm.at(0);
        pwmR = 255.0 * pwm.at(1);
      }
      
      // Yaw control.
      rot_v = yaw_control -> update(yaw, targetYaw);
      rot = rot_v.at(0);


      // Stop motors when pitch exceeds limit.
      if ( (currentAngle > angle_limit) || (currentAngle < -angle_limit) ){
        pwmL = 0;
        pwmR = 0;
        rot = 0.0;
      } 

      pwmL = constrain(pwmL, -255.0, 255.0);
      pwmR = constrain(pwmR, -255.0, 255.0);
      
      // Pass PWM commands to motors.
      L_motor(pwmL + int(rot));
      R_motor(pwmR - int(rot));

      // Print relevant data.
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

  targetAngle = get_pitch_command();
  targetYaw = get_yaw_command(targetYaw);

}
