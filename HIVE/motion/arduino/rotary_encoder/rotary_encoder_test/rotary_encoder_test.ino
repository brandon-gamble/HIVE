/*******************************
gives omega of enccoder, irrespective of direction
*******************************/

// pulses per rev of encoder
#define ENC_REV_COUNT 360

// encoder outout A to arduino
#define ENC_INA 2

// encoder pulse count
volatile float encoder_count = 0;

// interval for measurements
float interval = 1000;

// time tracking
long previousMillis = 0;
long currentMillis = 0;

// motor omega
float omega = 0;

void setup() {
//    Serial.begin(9600);
    Serial.begin(38400);
    Serial.println("beginning encoder test");

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

        // calc omega
        omega = encoder_count/interval*1000/ENC_REV_COUNT*2*3.14159;
        // update displace when motor is spinning
        if (omega > 0) {
            Serial.print(omega);
            Serial.println(" rad/s");
        }

        // reset encoder count
        encoder_count = 0;
    }
}

void updateEncoder() {
    encoder_count ++;
}
