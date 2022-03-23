/******************************************************************************
BJG_TB6612FNG.h
TB6612FNG H-Bridge Motor Driver Example code
Brandon Gamble @ University of Vermont
2022/03/22
<URL>
******************************************************************************/


#ifndef BJG_TB6612FNG_h
#define BJG_TB6612FNG_h

#include <Arduino.h>

class Motor
{
    public:
        // constructor
        Motor(int IN1_pin, int IN2_pin, int PWM_pin, int polarity, int STBY_pin);

        // functions
        void forward(int speed);
        void reverse(int speed);
        void drive(int speed);
        void brake();
        void standby(bool status);

    private:
        //variables for the 2 inputs, PWM input, polarity value, and the Standby pin
        int IN1, IN2, PWM, polarity, STBY, DRIVE_H, DRIVE_L;
};

#endif
