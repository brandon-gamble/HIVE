/******************************************************************************
BJG_TB6612FNG.cpp
Brandon Gamble @ University of Vermont
2022/03/22
<URL>
******************************************************************************/

#include "BJG_TB6612FNG.h"
#include <Arduino.h>

Motor::Motor(int IN1_pin, int IN2_pin, int PWM_pin, int polarity, int STBY_pin)
{
    IN1 = IN1_pin;
    IN2 = IN2_pin;
    PWM = PWM_pin;
    STBY = STBY_pin;
    polarity = polarity;

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(STBY, OUTPUT);


    if (polarity == 1) {
        DRIVE_H = HIGH;
        DRIVE_L = LOW;
    } else if (polarity == -1) {
        DRIVE_H = LOW;
        DRIVE_L = HIGH;
    }

    // disable motor to start
    digitalWrite(STBY, LOW);
}

void Motor::forward(int speed)
{
    digitalWrite(IN1, DRIVE_H);
    digitalWrite(IN2, DRIVE_L);
    analogWrite(PWM, speed);
}

void Motor::reverse(int speed)
{
    digitalWrite(IN1, DRIVE_L);
    digitalWrite(IN2, DRIVE_H);
    analogWrite(PWM, speed);
}

void Motor::brake()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(PWM,0);
}

void Motor::standby(bool status)
{
    if (status == 0) {
        // if STBY is false, power on
        digitalWrite(STBY, HIGH);
    } else {
        // any other input, put into standby
        digitalWrite(STBY, LOW);

        // whenever you go into standby, brake the motor
        // this way when you take it off standby it will
        // "forget" its previous velocity
        Motor::brake();
    }

}
