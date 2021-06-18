#include "dc_motor.h"

int PWM_freq = 200;

void motor_init(void){
  // Pins configuration
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Start with stopped motor
  stop_motor();

  // PWM
  ledcAttachPin(ENA, 1);  // (Pin, Channel)
  ledcAttachPin(ENB, 1);  // (Pin, Channel)
  ledcSetup(1, PWM_freq, 8);  // (Channel, PWM frequency, bits resolution)
  ledcWrite(1, 0);  // (channel, bits)
}

void stop_motor(void){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void bwd_L_motor(int pwm){
  // PWM 8 bits int --> [0; 255] 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(1, pwm);

}

void bwd_R_motor(int pwm){
  // PWM 8 bits int --> [0; 255] 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(1, pwm);

}

void fwd_L_motor(int pwm){
  // PWM 8 bits int --> [0; 255].
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(1, pwm);
}

void fwd_R_motor(int pwm){
  // PWM 8 bits int --> [0; 255].
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(1, pwm);
}

void R_motor(int pwm){
  if (pwm>=0){
    fwd_R_motor(pwm);
  }
  else {
    bwd_R_motor(-pwm);
  }
}

void L_motor(int pwm){
  if (pwm>=0){
    fwd_L_motor(pwm);
  }
  else {
    bwd_L_motor(-pwm);
  }
}
