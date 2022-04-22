/*
TO ADD
- save data to csv
    - motor command (analog write val)
    - motor RPM
    - motor voltage
*/

//////////////////////////////////
//       VOLTMETER SETUP        //
//////////////////////////////////
#define ANALOG_IN_PIN A0

int adc_val = 0;

float divider_voltage = 0.0;
float device_voltage = 0.0;
float device_voltage_filt = 0.0;
float tau = 16;

float R1 = 22000.0;
float R2 = 22000.0;

float ref_voltage = 4.096;


//////////////////////////////////
//      MOTOR DRIVER SETUP      //
//////////////////////////////////
# include <BJG_TB6612FNG.h>

// H-Bridge pins
#define AIN1 5
#define AIN2 4
#define PWMA 3
#define BIN1 7
#define BIN2 8
#define PWMB 9
#define STBY 6

// used to flip motor configuration without rewiring
// if motor is spinning opposite direction of intention, flip sign
const int polarity_A = 1;
const int polarity_B = 1;

Motor tread_left = Motor(AIN1, AIN2, PWMA, polarity_A, STBY);
Motor tread_right = Motor(BIN1, BIN2, PWMB, polarity_B, STBY);

long curr_motorTimer = 0;
long prev_motorTimer = 0;
int motorInterval = 8000;

int command_step = 0;
int prev_command_val = 0;
int curr_command_val = 0;

//////////////////////////////////
//     ROTARY ENCODER SETUP     //
//////////////////////////////////
// pulses per rev of encoder
#define ENC_REV_COUNT 360

// encoder outout A to arduino
#define ENC_INA 2

// encoder pulse count
volatile long encoder_count = 0;

// interval for measurements
int sensorInterval = 100;

// time tracking
long prev_sensorTimer = 0;
long curr_sensorTimer = 0;
long actual_interval = 0;

// motor speed [rad/s]
int omega = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("PROGRAM: sys_id.ino");
    Serial.println("UPLOAD DATE: 2022 APR 19");

    Serial.print("Sample time [ms]: ");
    Serial.println(sensorInterval);
    Serial.println("Time, Command Value, omega [rad/s], ADC, Motor Voltage (LPF), Motor Voltage (Filtered LPF)");

    //////////////////////////////////
    //      VOLTMETER EXTERNAL      //
    //////////////////////////////////
    analogReference(EXTERNAL);

    //////////////////////////////////
    //      MOTOR DRIVER SETUP      //
    //////////////////////////////////
    // disable standby (turn on motors)
    tread_left.standby(LOW);
    tread_right.standby(LOW);

    //////////////////////////////////
    //     ROTARY ENCODER SETUP     //
    //////////////////////////////////
    // set encoder pin as input
    pinMode(ENC_INA, INPUT_PULLUP);

    // attach interrupt for reading encoder
    attachInterrupt(digitalPinToInterrupt(ENC_INA), updateEncoder, RISING);

    // set initial time
    prev_sensorTimer = millis();
    prev_motorTimer = millis();

}

void loop(){
    curr_sensorTimer = millis();
    if (curr_sensorTimer - prev_sensorTimer > sensorInterval) {
        actual_interval = curr_sensorTimer - prev_sensorTimer;
        prev_sensorTimer = curr_sensorTimer;

        /////////////////////////////
        //     ROTARY ENCODER      //
        /////////////////////////////

        // calc motor speed [rad/s]
        omega = (float)(encoder_count / actual_interval / ENC_REV_COUNT * 2*3.14159);

        // reset encoder count
        encoder_count = 0;

        /////////////////////////////
        //       VOLTMETER         //
        /////////////////////////////

        // get adc analog value
        adc_val = analogRead(ANALOG_IN_PIN);

        // compute voltage at the divider
        divider_voltage = adc_val/1024.0*ref_voltage;

        // compute voltage at device and filter it
        device_voltage = divider_voltage/(R2/(R1+R2));
        device_voltage_filt = device_voltage_filt + (device_voltage - device_voltage_filt)/tau;

        ////////////////
        // PRINT DATA //
        ////////////////
        Serial.print(curr_command_val);
        Serial.print(", ");
        Serial.print(omega);
        Serial.print(", ");
        Serial.print(adc_val);
        Serial.print(", ");
        Serial.print(device_voltage);
        Serial.print(", ");
        Serial.print(device_voltage_filt);
        Serial.print(", ");
        Serial.println("");

    } // end rpm/voltage/text-output loop

    // do motor command steps
    curr_motorTimer = millis();
    if (curr_motorTimer - prev_motorTimer > motorInterval) {
        prev_motorTimer = curr_motorTimer;

        if (command_step == 0) {
            curr_command_val = 50;
        } else if (command_step == 1) {
            curr_command_val = 100;
        } else if (command_step == 2) {
            curr_command_val = 200;
        } else {
          curr_command_val = 0;
        }
        command_step ++;

        // if command val has changed, write to motor
        if (curr_command_val != prev_command_val) {
            prev_command_val = curr_command_val;
            tread_left.drive(curr_command_val);
//            tread_right.drive(curr_command_val);
        }
    } // end motor timer if loop

} // end main loop

///////////////////////////
//   encoder functions   //
///////////////////////////
void updateEncoder() {
    encoder_count ++;
}
