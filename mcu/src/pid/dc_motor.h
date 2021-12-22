
// Defines
#define IN1 18
#define IN2 19
#define ENA 5

#define IN3 0
#define IN4 2
#define ENB 15



// Global Function Protoyes
void motor_init(void);
void stop_motor(void);
void fwd_L_motor(int pwm);
void fwd_R_motor(int pwm);
void bwd_L_motor(int pwm);
void bwd_R_motor(int pwm);
void R_motor(int pwm);
void L_motor(int pwm);
