/*******************************
gives omega of enccoder, irrespective of direction
*******************************/

// pulses per rev of encoder
#define ENC_REV_COUNT 360

// encoder outout A to arduino
#define ENC_INA 2
#define ENC_INB 3
// encoder pulse count

volatile float encoder_count_0 = 0;
volatile float encoder_count_1 = 0;

// interval for measurements
float interval = 1000;

// time tracking
long previousMillis = 0;
long currentMillis = 0;

// motor omega
float omega_0 = 0;
float omega_1 = 0;

void setup() {
//    Serial.begin(9600);
    Serial.begin(38400);
    Serial.println("beginning encoder test");

    // set encoder pin as input
    pinMode(ENC_INA, INPUT_PULLUP);
    pinMode(ENC_INB, INPUT_PULLUP);

    // attach interrupt for reading encoder
    attachInterrupt(digitalPinToInterrupt(ENC_INA), updateEncoder_0, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_INB), updateEncoder_1, RISING);

    // set initial time
    previousMillis = millis();

}

void loop() {
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;

        // calc omega
        omega_0 = encoder_count_0/interval*1000/ENC_REV_COUNT*2*3.14159;
        omega_1 = encoder_count_1/interval*1000/ENC_REV_COUNT*2*3.14159;
        // update displace when motor is spinning
        if ((omega_0 > 0) || (omega_1 > 0)) {
            Serial.print(omega_0);
            Serial.print(", ");
            Serial.println(omega_1);
        }

        // reset encoder count
        encoder_count_0 = 0;
        encoder_count_1 = 0;
    }
}

void updateEncoder_0() {
    encoder_count_0 ++;
}


void updateEncoder_1() {
    encoder_count_1 ++;
}
