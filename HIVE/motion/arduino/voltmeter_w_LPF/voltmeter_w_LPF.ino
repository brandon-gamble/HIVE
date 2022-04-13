// voltmeter with double lowpass filter

#define ANALOG_RAW A1
#define ANALOG_LPF A0

//////////////////////////////////////////
// calculated using raw ADC value (PWM) //
//////////////////////////////////////////
int adc_raw = 0;
float divider_v_raw = 0.0; // raw voltage calculated from ADC measurement
float device_v_raw = 0.0; // raw voltage calculated from divider_voltage
float device_v_digFilt = 0.0;
const float tau = 64;

float R1_RAW = 220.0;
float R2_RAW = 220.0;

//////////////////////////////////////////////
// calculated using ADC val coming thru LPF //
//////////////////////////////////////////////
int adc_LPF = 0;
float divider_v_LPF = 0.0;
float device_v_LPF = 0.0; // calculated using LPF reading

float R1_LPF = 22000.0;
float R2_LPF = 22000.0;

///////////////////////
float ref_voltage = 5.0;


// TIMING TABLE //
// # Filters | Read Interval (ms) //
// 0 | 5
// 1 | 15
// 2 | 35
// doing double filter, run at 35 ms
int interval = 25;

long currentMillis = 0;
long previousMillis = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("voltmeter_w_LPF.ino");
    Serial.println("2022 APR 13");
//    Serial.println("time, device_v_raw, device_v_digFilt, device_v_LPF");
}

void loop(){
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;

        /////////////////////////////////////
        // RAW READINGS AND DIGITAL FILTER //
        /////////////////////////////////////
        // get adc analog value
        adc_raw = analogRead(ANALOG_RAW);

        // compute voltage at the divider
        divider_v_raw = adc_raw/1024.0*ref_voltage;

        // compute voltage at device
        device_v_raw = divider_v_raw/(R2_RAW/(R1_RAW+R2_RAW));
        device_v_digFilt = device_v_digFilt + (device_v_raw - device_v_digFilt)/tau;

        /////////////////////////////////
        // LPF READING (ANALOG FILTER) //
        /////////////////////////////////
        adc_LPF = analogRead(ANALOG_LPF);
        divider_v_LPF = adc_LPF/1024.0*ref_voltage;
        device_v_LPF = divider_v_LPF/(R2_LPF/(R1_LPF+R2_LPF));

        //////////////////////////
        // PRINT DATA TO SCREEN //
        //////////////////////////
        Serial.print(device_v_raw);
        Serial.print(", ");
        Serial.print(device_v_digFilt);
        Serial.print(", ");
        Serial.print(device_v_LPF);
        Serial.print(", ");
//        Serial.print(adc_raw);
//        Serial.print(", ");
//        Serial.print(adc_LPF);
//        Serial.print(", ");
        Serial.println("");

        // print voltage
//        Serial.print(device_voltage);
//        Serial.println(" V");

    }

}
