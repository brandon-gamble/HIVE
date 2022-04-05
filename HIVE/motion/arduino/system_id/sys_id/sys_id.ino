/*
TO ADD
- send motor command
- read motor RPM
- save data to csv
    - motor command (analog write val)
    - motor RPM
    - motor voltage 
*/

#define ANALOG_IN_PIN A0

float divider_voltage = 0.0;
float device_voltage = 0.0;

float R1 = 30000.0;
float R2 = 7500.0;

float ref_voltage = 5.0;

int adc_val = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("PROGRAM: sys_id.ino");
}

void loop(){
    adc_val = analogRead(ANALOG_IN_PIN);

    divider_voltage = adc_val/1024.0*ref_voltage;

    device_voltage = divider_voltage/(R2/(R1+R2));

    Serial.print("device voltage: ");
    Serial.println(device_voltage, 2);

    delay(500);
}
