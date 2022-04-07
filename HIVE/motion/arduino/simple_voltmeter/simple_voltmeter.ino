#define ANALOG_IN_PIN A0

float divider_voltage = 0.0;
float device_voltage = 0.0;

float R1 = 22000.0;
float R2 = 22000.0;

float ref_voltage = 5.0;

int adc_val = 0;

int interval = 1000;

long currentMillis = 0;
long previousMillis = 0;
void setup() {
    Serial.begin(9600);
    Serial.println("simple_voltmeter.ino");
}

void loop(){
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;

        // get adc analog value
        adc_val = analogRead(ANALOG_IN_PIN);

        // compute voltage at the divider
        divider_voltage = adc_val/1024.0*ref_voltage;

        // compute voltage at device
        device_voltage = divider_voltage/(R2/(R1+R2));

        // print voltage
        Serial.print(device_voltage);
        Serial.println(" V");
    }

}
