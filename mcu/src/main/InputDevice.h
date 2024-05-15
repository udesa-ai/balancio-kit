#pragma once
class InputDevice{
    public: 
        InputDevice();
        //virtual ~InputDevice();
        virtual void setup(void);
        static  void onConnect();
        
        virtual void parse_input(float pitch_yaw[2]) = 0;//Devuelve el pitch y yaw en un par en ese orden

        virtual float get_pitch_command(float read)= 0;
        virtual float get_yaw_command(float prev_target,float read) = 0;

        virtual bool x_button_pressed(void) = 0;
};