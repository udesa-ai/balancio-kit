/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/
#pragma once

#ifndef App_h
#define App_h

#include "config.h"
#include "InputDevice.h"

// PS3 Controller related functions.

class App : public InputDevice {
    public: 

        App();
        /**
        * Callback executed when the joystick connects to the board.
        */
        void setup(void);


        /**
        * Return by interface in the array that was indicated as parameter what was sent from the app. Format: [pitch,yaw].
        */
        void parse_input(float pitch_yaw[2] );

        /**
        * Adjusts the read pitch and returns it .
        * 
        * @return Pitch target
        */
        float get_pitch_command(float read_n);

        /**
        * Get the commanded yaw value from the PS3 joystick.
        * 
        * @param  {float} prev_target : Previous commanded yaw.
        * @return New commanded yaw.
        */
        float get_yaw_command(float prev_target,float read);
        
        /**
        * Checks  if the x button is pressed (Currently not implemented here, but it is in the PS3 controller).
        * 
        * @return True if the x button is pressed. False if not. (Currentyl returning only false)
        */
        bool x_button_pressed(void);
};
#endif

