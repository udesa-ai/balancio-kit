/* 
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
====================================================================== 
*/

#include "timer.h"
#include "config.h"

void timer_init(void)
{
   hw_timer_t *timer = NULL;
   portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

   // Configure the Prescaler at 80. Clock frequency at 80Mhz.
   // 80000000 / 80 = 1000000 tics / second
   timer = timerBegin(0, 80, true);
   timerAttachInterrupt(timer, &onTime, true);

   // Sets timer at 100Hz.
   // timerAlarmWrite(timer, 10000, true);
   int scaler = 1000000 * LOOP_PERIOD;
   timerAlarmWrite(timer, scaler, true);
   timerAlarmEnable(timer);
}
