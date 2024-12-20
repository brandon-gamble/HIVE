//////////////////////////////////
//      MOTOR DRIVER SETUP      //
//////////////////////////////////
# include <BJG_TB6612FNG.h>

// H-Bridge pins
#define AIN1 8
#define AIN2 7
#define PWMA 6

#define BIN1 10
#define BIN2 11
#define PWMB 12

#define STBY 9

// used to flip motor configuration without rewiring
// if motor is spinning opposite direction of intention, flip sign
const int polarity_A = -1;
const int polarity_B = 1;

Motor tread_left = Motor(AIN1, AIN2, PWMA, polarity_A, STBY);
Motor tread_right = Motor(BIN1, BIN2, PWMB, polarity_B, STBY);

//////////////////////////////////
//     ROTARY ENCODER SETUP     //
//////////////////////////////////
// pulses per rev of encoder
#define ENC_REV_COUNT 360

// encoder outout A to arduino
#define ENC_INA 3
#define ENC_INB 4

// encoder pulse count
volatile long encoder_count = 0;

// motor speed [rad/s]
float omega_l = 0;
float dir_l = 0.0;

////////////////////
//     timers     //
////////////////////

// interval for measurements
int sensorInterval = 50;
// int sensorInterval = 20;

// time tracking
long prev_sensorTimer = 0;
long curr_sensorTimer = 0;
float actual_interval = 0;


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

////////////////////////////
// CONTROL LOOP VARIABLES //
////////////////////////////
// motor parameters
#define J_L 1
#define K_L 1.809
#define B_L 16.17
// controller parameters
//#define KI_L 26.4
//#define KP_L 811

#define KI_L .02
#define KP_L 0.8

//#define KI_L .2
//#define KP_L 8

float omega_l_des = 0;
float error_l = 0;
float u_integral_l = 0;
float u_l = 0;

void setup() {
    Serial.begin(38400);
    Serial.println("PROGRAM: wheel_speed_control_test.ino");
    Serial.println("UPLOAD DATE: 2022 DEC 12");
    Serial.println("BAUD 38400");

    Serial.print("KI_L: ");
    Serial.println(KI_L);
    Serial.print("KP_L: ");
    Serial.println(KP_L);
    Serial.print("T_controller [ms]: ");
    Serial.println(sensorInterval);

    // Serial.println("int_from_msg, Desired Speed [rad/s], Control Action [8bit], Tread Speed [rad/s]");
    Serial.println("omega_l_des [rad/s], omega [rad/s], error_l [rad/s], u_integral_l , u_l [8bit]");

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
    pinMode(ENC_INB, INPUT);

    // attach interrupt for reading encoder
    attachInterrupt(digitalPinToInterrupt(ENC_INA), updateEncoder, RISING);

    // set initial time
    prev_sensorTimer = millis();

}

void loop(){
    receive_message();
    if (new_data == true) {
        // temporary copy to preserve original data
        // bc strtok() in parse_msg() replaces commas with \0
        strcpy(temp_chars, chars_received);
        parse_msg();
        //show_parsed_msg();
        get_command();
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
        omega_l = (float)(encoder_count/actual_interval*1000*2*3.14159 / ENC_REV_COUNT);

        // find direction of spin
        // if (digitalRead(ENC_INA) != digitalRead(ENC_INB)) {
        //     omega_l = -1*omega_l;
        // }

        // reset encoder count
        encoder_count = 0;

        //////////////////////////
        //     CONTROL LOOP     //
        //////////////////////////

        // encoders only wired for positive signal:
        // need to flip sign of omega when desired is negative
        if (omega_l_des >= 0) {
            error_l = omega_l_des - omega_l;
        } else {
            error_l = omega_l_des + omega_l;
        }
        u_integral_l = u_integral_l + error_l*actual_interval;
        u_l = KP_L*error_l + KI_L*u_integral_l;


        if (u_l > 255) {
           u_l = 255;
        }
        if (u_l < -255) {
           u_l = -255;
        }
        tread_left.drive(u_l);


        ////////////////
        // PRINT DATA //
        ////////////////

        // Serial.print(omega_l_des);
        // Serial.print(", ");
        Serial.print(omega_l);
        Serial.print(", ");

        Serial.print(dir_l);

        // Serial.print(error_l);
        // Serial.print(", ");
        // Serial.print(u_integral_l);
        // Serial.print(", ");
        // Serial.print(u_l);
        // Serial.print(", ");
        Serial.println("");

    } // end rpm/voltage/text-output loop


} // end main loop

///////////////////////////
//   encoder functions   //
///////////////////////////
// void updateEncoder() {
//     encoder_count ++;
// }
void updateEncoder() {
    encoder_count ++;
    dir_l = getDir();
}
float getDir() {
    // this is not working properly
    if (digitalRead(ENC_INA) != digitalRead(ENC_INB)) {
        return -1;
    } else {
        return +1;
    }
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

void get_command() {
    // process and send actuation commands
    switch (char_from_msg[0]) {
        case 'L':                                   // 'L' = left tread
            // left tread
            omega_l_des = int_from_msg;
            break;
        case 'R':                                   // 'R' = right tread
            // right tread
            Serial.println("THIS TEST IS DESIGNED TO ONLY DRIVE LEFT TREAD");
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
