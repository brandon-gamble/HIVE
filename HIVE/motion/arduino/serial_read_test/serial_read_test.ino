

#include <SparkFun_TB6612.h>

// H-Bridge pins
#define AIN1 3
#define AIN2 4
#define BIN1 7
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// command time
// in case something hangs, motors will never run for longer
// than the CMD_TIME
#define CMD_TIME 1000

int m1_speed = 0;
int m2_speed = 0;

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
// Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(10);

    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);

    // write motor speeds to zero, then enable H-Bridge
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(STBY, HIGH);

    Serial.println("setup complete");
}

void loop() {
    if (Serial.available() > 0) {

        m1_speed = Serial.parseInt();
        m2_speed = Serial.parseInt();
        Serial.println(m1_speed);
        Serial.println(m2_speed);

        if (m1_speed >= 0) {
            // set forward
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
        } else {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            m1_speed = -m1_speed;
        }
        analogWrite(PWMA, m1_speed);


        if (m2_speed >= 0) {
            // set forward
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
        } else {
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, HIGH);
            m2_speed = -m2_speed;
        }
        analogWrite(PWMB, m2_speed);

    } // end serial read
} // end loop
