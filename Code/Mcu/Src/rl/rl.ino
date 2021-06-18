#include <EloquentTinyML.h>
#include "rl_model.h"
#include "dc_motor.h"
#include "imu.h"
#include "Wire.h"

/* Neural Net */
#define NUMBER_OF_INPUTS 1
#define NUMBER_OF_OUTPUTS 1
// in future projects you may need to tweak this value.
// it's a trial and error process
#define TENSOR_ARENA_SIZE 2*1024
Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;

/* Imu + Motor */
int i;
int speed = 100;
float pitch; 
int pwm;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
unsigned long timer = 0;


void setup() {
    /* UART PC connection */
    Serial.begin(115200);
    /* NN */
    ml.begin(rl_model);
    /* Motor initialization */
    motor_init();
    /* IMU init */
    imu_setup();
}

void loop() {
    getEulerAngles(ypr);
    pitch = ypr[1];
    // pick up a random x 
//    float x = 1.0;
    float input[1] = { pitch };
    float predicted = ml.predict(input);

    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" --> ");
    Serial.print(" Predicted: ");
    Serial.println(predicted);
//    delay(1000);

    if (predicted>=0){
      pwm = 30 * predicted;
      clockwise_motor(pwm);
    }
    else{
      pwm = 30 * (-predicted);
      counterclockwise_motor(pwm);
    }
    Serial.print(" --> PWM : ");
    Serial.println(pwm);
}
