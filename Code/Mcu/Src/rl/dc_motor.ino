#include "dc_motor.h"

int PWM_freq = 200;

void motor_init(void){
  // Pins configuration
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Start with stopped motor
  stop_motor();

  // PWM
  ledcAttachPin(ENB, 1);  // (Pin, Channel)
  ledcSetup(1, PWM_freq, 8);  // (Channel, PWM frequency, bits resolution)
  ledcWrite(1, 0);  // (channel, bits)
}

void stop_motor(void){
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void clockwise_motor(int pwm){
  // PWM 8 bits int --> [0; 255] 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(1, pwm);
}

void counterclockwise_motor(int pwm){
  // PWM 8 bits int --> [0; 255]
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(1, pwm);
}
