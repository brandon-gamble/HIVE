/*
Initial test for TB6612FNG H-Bridge

Two motors, unidirectional.

wiring:
1) Nano -> H-Bridge
D3 -> AI1
D4 -> AI2
D5 -> PWMA
D6 -> PWMB
D7 -> BI1
D8 -> BI2
5v -> STBY
gnd -> gnd

2) Nano -> pot
5V -> pin1
A0/A1 -> pin2 (wiper, middle pin)
gnd -> pin3

3) separate 5V source -> H-Bridge
5v -> VM
gnd -> gnd

4) motors -> H-Bridge
motorA -> A01/A02
motorB -> B01/B02
*/

////////////////////
// initialization //
////////////////////

// motor A
int pwmA = 5;
int in1A = 3;
int in2A = 4;

// motor B
int pwmB = 6;
int in1B = 7;
int in2B = 8;

// potentiometers for speed control
int SpeedControl1 = A0;
int SpeedControl2 = A1;

// motor speed vals
int MotorSpeed1 = 0;
int MotorSpeed2 = 0;

void setup()
{
    pinMode(pwmA, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(in1A, OUTPUT);
    pinMode(in2A, OUTPUT);
    pinMode(in1B, OUTPUT);
    pinMode(in2B, OUTPUT);
}

void loop()
{
    // set motor A forward
    digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);

    // set motor B forward
    digitalWrite(in1B, HIGH);
    digitalWrite(in2B, LOW);

    // read pots and map to 0-255
    MotorSpeed1 = map(analogRead(SpeedControl1), 0, 1023, 0, 255);
    MotorSpeed2 = map(analogRead(SpeedControl2), 0, 1023, 0, 255);

    // set motor speeds
    analogWrite(pwmA, MotorSpeed1);
    analogWrite(pwmB, MotorSpeed2);

}
