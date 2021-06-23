#include "dc_motor.h"
#include "imu.h"
#include "timer.h"
#include "Wire.h"

//#define CUSTOM_SETTINGS
//#define INCLUDE_GAMEPAD_MODULE
//#include <DabbleESP32.h>
#include <Ps3Controller.h>

#define Kp  1200    //1200    //1750
#define Kd  17.0    //17.0    //25.0
#define Ki  22000   //22000   //11000  
#define sampleTime  0.01  // 100 Hz
#define zero_targetAngle -0.011  // Calibrated point

float targetAngle=0.0;
float currentAngle=0.0, prevAngle=0.0, error=0.0, prevError=0.0, errorSum=0.0;
int pwm=0;
volatile bool controlFlag=false;

float fwd=0;
float rot=0;

unsigned long pid_count = 0;

int time_ctr=0;
float ax, az, gy;
float accelPitch, pitch;
float tau=0.98;

void setup() {
  // UART PC connection
  Serial.begin(115200); 
  
  // Motor initialization
  motor_init();
  
  // IMU init
  imu_setup();

  // Bluetooth init
//  Dabble.begin("MyEsp32");       //set bluetooth name of your device
    ps3_setup();

  // Timer init. (ISR at 100 Hz)
  timer_init();
}

// ISR at 100 Hz
void IRAM_ATTR onTime() {
  controlFlag=true;
}

void loop() {


  if (controlFlag){ 

    getAccelGyro(&ax, &az, &gy);
    accelPitch = atan2(ax, az) * RAD_TO_DEG;
    pitch = (tau)*(pitch + (-gy)*sampleTime) + (1-tau)*(accelPitch);  
    currentAngle = pitch * DEG_TO_RAD;
      
    error = currentAngle - targetAngle;
    errorSum += error;  
    errorSum = constrain(errorSum, -5, 5);
    pwm = Kp*(error) + Ki*(errorSum)*sampleTime + Kd*(currentAngle-prevAngle)/sampleTime;
    pwm = constrain(pwm, -255, 255);
    prevAngle = currentAngle;
    
    L_motor(pwm + int(rot*60));
    R_motor(pwm - int(rot*60));

    if (true){
      Serial.print("fwd: ");
      Serial.print(fwd);
      Serial.print(" rot: ");
      Serial.print(rot);
      Serial.print("  Pitch:  ");
      Serial.print(currentAngle, 4);
      Serial.print("  PWM:  ");
      Serial.print(pwm);
      Serial.print("  Loop time: ");
      time_ctr = micros()-time_ctr;
      Serial.println(time_ctr);
      time_ctr = micros();    
    }
    
  controlFlag=false;
  
  }
  else{
    return;
  }

  // Get joystick commands
//  Dabble.processInput();
//  fwd = GamePad.getYaxisData();
//  fwd = fwd/7.0;
//  rot = GamePad.getXaxisData();
//  rot = rot/7.0;

  
  fwd = -Ps3.data.analog.stick.ry/128.0;
  rot = Ps3.data.analog.stick.lx/128.0;

  targetAngle = fwd*0.05 + zero_targetAngle;
 

}

void ps3_setup(void){
//    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("8c:7c:b5:97:58:48");

    Serial.println("Ready.");
}

void onConnect(){
    Serial.println("Connected.");
}
