/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/

// PS3 Controller related function prototypes.

/**
 * Connect board to PS3 joystick via bluetooth.
 * The targeted device is specified via the MAC address.
 */
void ps3_setup(void);

/**
 * Callback executed when the joystick connects to the board.
 */
void onConnect();

/**
 * Get the commanded pitch value from the PS3 joystick.
 * 
 * @return Pitch target
 */
float get_pitch_command(void);

/**
 * Get the commanded yaw value from the PS3 joystick.
 * 
 * @param  {float} prev_target : Previous commanded yaw.
 * @return New commanded yaw.
 */
float get_yaw_command(float prev_target);

/**
 * Get if the x button of the PS3 joystick is pressed.
 * 
 * @return True if the x button is pressed. False if not.
 */
bool x_button_pressed(void);
