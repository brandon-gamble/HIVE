# include <BJG_TB6612FNG.h>

// H-Bridge pins
#define AIN1 3
#define AIN2 4
#define PWMA 5
#define BIN1 7
#define BIN2 8
#define PWMB 6
#define STBY 9

// used to flip motor configuration without rewiring
// if motor is spinning opposite direction of intention, flip sign
const int polarity_A = 1;
const int polarity_B = 1;

Motor motorA = Motor(AIN1, AIN2, PWMA, polarity_A, STBY);
Motor motorB = Motor(BIN1, BIN2, PWMB, polarity_B, STBY);

void setup() {
    // disable standby (i.e., turn on motors)
    motorA.standby(LOW);
    motorB.standby(LOW);
}

void loop() {
    motorA.forward(100);
    motorB.forward(100);
    Serial.println("both motors forward @ 100");
    delay(2000);

    motorA.brake();
    motorB.brake();
    Serial.println("brake both motors");
    delay(1000);

    motorA.reverse(50);
    motorB.reverse(50);
    Serial.println("both motors reverse @ 50");
    delay(2000);
    motorA.brake();
    Serial.println("brake motor A");
    delay(500);
    motorB.brake();
    Serial.println("brake motor B");
    delay(500);

    motorA.forward(250);
    motorB.reverse(250);
    Serial.println("A forward @ 250, B reverse @ 250")
    delay(500);
    motorA.brake();
    motorB.brake();
    Serial.println("brake both motors");
    delay(2000);

    motorA.standby(TRUE);
    motorB.standby(TRUE);
    Serial.println("both motors on standby")
    motorA.foward(100);
    motorB.foward(100);
    Serial.println("motors set to foward @ 100 (but on standby, shouldn't be spinning)");
    delay(1000);
    motorA.standby(FALSE);
    Serial.println("motor A off standby. should spin forward @ 100");

    motorA.brake();
    motorB.brake();
    Serial.println("brake both motors");
    delay(2000);


}
