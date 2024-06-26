/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/
#pragma once

#ifndef commander_h
#define commander_h

#include <Ps3Controller.h>
#include "config.h"
#include "InputDevice.h"

// PS3 Controller related functions.

class PS3Joystick : public InputDevice {
    public: 
        /**
        * Connect board to PS3 joystick via bluetooth.
        * The targeted device is specified via the MAC address.
        */
        PS3Joystick();
        void setup(void);
        /**
        * Callback executed when the joystick connects to the board.
        */

        /**
        * Devuelve en formato pitch,yaw
        */
        void parse_input(float fill_arr[2] );

        /**
        * Get the commanded pitch value from the PS3 joystick.
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
        * Get if the x button of the PS3 joystick is pressed.
        * 
        * @return True if the x button is pressed. False if not.
        */
        bool x_button_pressed(void);
};

#endif
