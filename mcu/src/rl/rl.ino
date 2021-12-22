#include <EloquentTinyML.h>
#include "rl_model.h"
#include "dc_motor.h"
#include "imu.h"
#include "Wire.h"

/* Neural Net */
#define NUMBER_OF_INPUTS 1
#define NUMBER_OF_OUTPUTS 2
// in future projects you may need to tweak this value.
// it's a trial and error process
#define TENSOR_ARENA_SIZE 16*1024
Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;
// IMU Complementary Filter
#define sampleTime  0.01  // 100 Hz

/* Imu + Motor */
int i;
//int speed = 100;
float targetAngle=0.05, diffAngle;
float currentAngle=0.0;
int pwmL, pwmR;
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
unsigned long timer = 0;
volatile bool controlFlag=false;

int time_ctr=0;
float ay, az, gx;
float accelPitch, pitch;
float tau=0.98;

//float predicted[2]= {0};

void setup() {
    /* UART PC connection */
    Serial.begin(115200);
    /* NN */
    ml.begin(rl_model);
    /* Motor initialization */
    motor_init();
    /* IMU init */
    imu_setup();
    /* Timer init. (ISR at 100 Hz) */
    timer_init();
}

// ISR at 100 Hz
void IRAM_ATTR onTime() {
  controlFlag=true;
}

void loop() {

  if (controlFlag){ 
//
//    x_down = Ps3.event.button_down.cross;
//    if (x_down){
//      stop_command = abs(stop_command -1); 
//    }

//    if (!stop_command){
      getAccelGyro(&ay, &az, &gx);
      accelPitch = atan2(ay, az) * RAD_TO_DEG;
      pitch = (tau)*(pitch + (gx)*sampleTime) + (1-tau)*(accelPitch);  
      currentAngle = - pitch * DEG_TO_RAD;

//    Input Normalization (for neural net compatibility). 
//    WARNING: This must be the same normalization used during training.
      diffAngle = currentAngle - targetAngle;
//      diffAngle /= 1.5708;
      diffAngle /= 0.06519447;
      diffAngle = constrain(diffAngle, -1, 1);
      
      float input[1] = { diffAngle };
      float predicted[2]= {0};
      ml.predict(input, predicted);
      pwmL = predicted[0] * 255;
      pwmL = constrain(pwmL, -255, 255);
      pwmR = predicted[1] * 255;
      pwmR = constrain(pwmR, -255, 255);
      
      L_motor(pwmL);
      R_motor(pwmR);
  
      if (true){
//        Serial.print("fwd: ");
//        Serial.print(fwd);
//        Serial.print(" rot: ");
//        Serial.print(rot);
        Serial.print("  Pitch:  ");
        Serial.print(currentAngle, 4);
        Serial.print("  PWML:  ");
        Serial.print(pwmL);
        Serial.print("  Loop time: ");
        time_ctr = micros()-time_ctr;
        Serial.println(time_ctr);
        time_ctr = micros();    
      }
      
//    }
//    else{
//      stop_motor(); 
//    }
      
    controlFlag=false;
        
  }
  else{
    return;
  }
  
//    getEulerAngles(ypr);
//    pitch = ypr[1];
//    // pick up a random x 
////    float x = 1.0;
//    float input[1] = { pitch };
//    float predicted = ml.predict(input);
//
//    Serial.print("Pitch: ");
//    Serial.print(pitch);
//    Serial.print(" --> ");
//    Serial.print(" Predicted: ");
//    Serial.println(predicted);
////    delay(1000);
//
//    if (predicted>=0){
//      pwm = 30 * predicted;
//      clockwise_motor(pwm);
//    }
//    else{
//      pwm = 30 * (-predicted);
//      counterclockwise_motor(pwm);
//    }
//    Serial.print(" --> PWM : ");
//    Serial.println(pwm);
}
