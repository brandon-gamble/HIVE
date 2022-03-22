/*******************************
gives RPM of enccoder, irrespective of direction
*******************************/

// pulses per rev of encoder
#define ENC_REV_COUNT 360

// encoder outout A to arduino
#define ENC_INA 3
#define ENC_INB 5

// encoder pulse count
volatile long encoder_count = 0;

// interval for measurements
int interval = 1000;

// time tracking
long previousMillis = 0;
long currentMillis = 0;

// motor rpm
int rpm = 0;

void setup() {
    Serial.begin(9600);

    // set encoder pin as input
    pinMode(ENC_INA, INPUT_PULLUP);

    // attach interrupt for reading encoder
    attachInterrupt(digitalPinToInterrupt(ENC_INA), updateEncoder, RISING);

    // set initial time
    previousMillis = millis();
}

void loop() {
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;

        // calc rpm
        rpm = (float)(encoder_count * 60 / ENC_REV_COUNT);

        // update displace when motor is spinning
        if (rpm > 0) {
            Serial.print(rpm);
            Serial.println(" RPM");
        }

        // reset encoder count
        encoder_count = 0;
    }
}

void updateEncoder() {
    encoder_count ++;
}
