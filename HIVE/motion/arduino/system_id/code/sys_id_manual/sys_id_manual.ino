/*
TO ADD
- save data to csv
    - motor command (analog write val)
    - motor RPM
    - motor voltage
*/

//////////////////
// serial setup //
//////////////////

const byte msg_length = 16;       // max message size
char chars_received[msg_length];   // char array to store message
char temp_chars[msg_length];       // char array for parsing

// hold parsed data
char char_from_msg[msg_length] = {0};
int int_from_msg = 0;

boolean new_data = false;

//////////////////////////////////
//       VOLTMETER SETUP        //
//////////////////////////////////
#define ANALOG_IN_PIN A0

int adc_val = 0;

float divider_voltage = 0.0;
float device_voltage = 0.0;
float device_voltage_filt = 0.0;
float tau = 64;

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
int motorInterval = 10000;

int command_step = 0;
int command_val = 0;

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
int sensorInterval = 50;

// time tracking
long prev_sensorTimer = 0;
long curr_sensorTimer = 0;
float actual_interval = 0;

// motor speed [rad/s]
int omega = 0;

void setup() {
    Serial.begin(38400);
    Serial.println("PROGRAM: sys_id_manual.ino");
    Serial.println("UPLOAD DATE: 2022 DEC 05");
    Serial.println("BAUD 38400");

    Serial.println("Command Value, RAD/S, ADC, Motor Voltage (LPF), Motor Voltage (Filtered LPF)");

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
    receive_message();
    if (new_data == true) {
        // temporary copy to preserve original data
        // bc strtok() in parse_msg() replaces commas with \0
        strcpy(temp_chars, chars_received);
        parse_msg();
        //show_parsed_msg();
        send_commands();
        new_data = false;
    }

    curr_sensorTimer = millis();
    if (curr_sensorTimer - prev_sensorTimer > sensorInterval) {
        actual_interval = curr_sensorTimer - prev_sensorTimer;
        prev_sensorTimer = curr_sensorTimer;

        /////////////////////////////
        //     ROTARY ENCODER      //
        /////////////////////////////

        // calc wheel speed [rad/s]
        omega = (float)(encoder_count/actual_interval*1000*2*3.14159 / ENC_REV_COUNT);

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
        Serial.print(int_from_msg);
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


} // end main loop

///////////////////////////
//   encoder functions   //
///////////////////////////
void updateEncoder() {
    encoder_count ++;
}

////////////////////
// serial control //
////////////////////
void receive_message() {
    static boolean recv_in_progress = false;
    static byte buffer_index = 0;
    char start_marker = '<';
    char end_marker = '>';
    char current_char;

    while (Serial.available() > 0 && new_data == false) {
        current_char = Serial.read();

        if (recv_in_progress == true) {
            if (current_char != end_marker) {
                chars_received[buffer_index] = current_char;
                buffer_index++;
                if (buffer_index >= msg_length) {
                    buffer_index = msg_length - 1;
                }
            }
            else {
                chars_received[buffer_index] = '\0'; // terminate string
                recv_in_progress = false;
                buffer_index = 0;
                new_data = true;
            }
        }
        else if (current_char == start_marker) {
            recv_in_progress = true;
        }
    }
}

void parse_msg() {
    char * strtok_index; // index for strtok()

    strtok_index = strtok(temp_chars,","); // get string
    strcpy(char_from_msg, strtok_index);  // copy string to char_from_msg

    strtok_index = strtok(NULL, ",");     // continuing from previous indx, get int
    int_from_msg = atoi(strtok_index);   // convert to int
}

void show_parsed_msg() {
    Serial.print("act_id: ");
    Serial.print(char_from_msg);
    Serial.print("\t");
    Serial.print("act_val: ");
    Serial.println(int_from_msg);
}

void send_commands() {
    // process and send actuation commands
    switch (char_from_msg[0]) {
        case 'L':                                   // 'L' = left tread
            // left tread
            tread_left.drive(int_from_msg);
            break;
        case 'R':                                   // 'R' = right tread
            // right tread
            tread_right.drive(int_from_msg);
            break;
        case 'B':                                   // 'B' = brake
            tread_left.brake();
            tread_right.brake();
        case 'S':                                   // 'S' = standby
            if (int_from_msg == 0) {
                // standby = false so turn on
                tread_left.standby(LOW);
                tread_right.standby(LOW);
            } else if (int_from_msg == 1) {
                // standby = true so turn off
                tread_left.standby(HIGH);
                tread_right.standby(HIGH);
            }
        default:
            // if unknown actuator id is received, brake and standby
            tread_left.brake();
            tread_right.brake();
    }
}
