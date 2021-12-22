#include "timer.h"

void timer_init(void){
   hw_timer_t * timer = NULL;
   portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
   
   // Configure the Prescaler at 80. Clock frequency at 80Mhz.
   // 80000000 / 80 = 1000000 tics / second
   timer = timerBegin(0, 80, true);                
   timerAttachInterrupt(timer, &onTime, true);    
    
   // Sets timer at 100Hz.
   timerAlarmWrite(timer, 10000, true);           
   timerAlarmEnable(timer);
}
