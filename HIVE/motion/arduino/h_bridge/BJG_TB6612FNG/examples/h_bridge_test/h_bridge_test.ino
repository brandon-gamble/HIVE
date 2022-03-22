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
    Serial.begin(9600);

    // disable standby (i.e., turn on motors)
    motorA.standby(LOW);
    motorB.standby(LOW);
}

void loop() {
    Serial.println("----------------------------------");
    Serial.println("-----  starting test loop --------");
    Serial.println("----------------------------------");

    motorA.forward(150);
    motorB.forward(150);
    Serial.println("both motors forward @ 150");
    Serial.println("----------------------------------");
    delay(3000);

    motorA.brake();
    motorB.brake();
    Serial.println("brake both motors");
    Serial.println("----------------------------------");
    delay(2000);

    motorA.reverse(100);
    motorB.reverse(100);
    Serial.println("both motors reverse @ 100");
    delay(3000);
    motorA.brake();
    Serial.println("brake motor A");
    delay(2000);
    motorB.brake();
    Serial.println("brake motor B");
    Serial.println("----------------------------------");
    delay(2000);

    motorA.forward(250);
    motorB.reverse(250);
    Serial.println("A forward @ 250, B reverse @ 250");
    delay(3000);
    motorA.brake();
    motorB.brake();
    Serial.println("brake both motors");
    Serial.println("----------------------------------");
    delay(2000);

    motorA.forward(200);
    motorB.forward(200);
    motorA.standby(HIGH);
    Serial.println("speeds set to 200, motorA put on standby.");
    Serial.println("both motors should turn off.");
    delay(3000);
    motorA.standby(LOW);
    Serial.println("out of standby. motorA should still be stopped because it had its velocity reset when put on standby...");
    Serial.println("motorB should turn back on to 200 forward");
    Serial.println("----------------------------------");
    delay(3000);

    motorB.brake();
    Serial.println("brake B. both motors should be stopped");
    Serial.println("----------------------------------");
    delay(5000);


}
