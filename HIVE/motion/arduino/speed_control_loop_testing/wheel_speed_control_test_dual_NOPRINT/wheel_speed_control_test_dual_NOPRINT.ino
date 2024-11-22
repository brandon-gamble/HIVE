//////////////////////////////////
//      MOTOR DRIVER SETUP      //
//////////////////////////////////
# include <BJG_TB6612FNG.h>

// H-Bridge pins
#define AIN1 7
#define AIN2 6
#define PWMA 5

#define BIN1 9
#define BIN2 10
#define PWMB 11

#define STBY 8

// used to flip motor configuration without rewiring
// if motor is spinning opposite direction of intention, flip sign
const int polarity_A = 1;
const int polarity_B = 1;

Motor tread_left = Motor(AIN1, AIN2, PWMA, polarity_A, STBY);
Motor tread_right = Motor(BIN1, BIN2, PWMB, polarity_B, STBY);

//////////////////////////////////
//     ROTARY ENCODER SETUP     //
//////////////////////////////////

//////////////////////
// ENCODER 0 - LEFT //
//////////////////////

// pulses per rev of encoder
#define ENC_0_REV_COUNT 360
// gear ratio between encoder and output shaft
//#define ENC_0_GEAR_RATIO 0.88949
#define ENC_0_GEAR_RATIO 1
// encoder outout A to arduino
#define ENC_0_INA 2
// encoder pulse count
volatile long encoder_0_count = 0;
// motor speed [rad/s]
float omega_enc_0 = 0;

///////////////////////
// ENCODER 1 - RIGHT //
///////////////////////

// pulses per rev of encoder
#define ENC_1_REV_COUNT 360
// gear ratio between encoder and output shaft
//#define ENC_1_GEAR_RATIO 1.1786
#define ENC_1_GEAR_RATIO 1
// encoder outout A to arduino
#define ENC_1_INA 3
// #define ENC_1_INB 4s
// encoder pulse count
volatile long encoder_1_count = 0;
// motor speed [rad/s]
float omega_enc_1 = 0.0;

////////////////////
//     timers     //
////////////////////

// interval for measurements
//int sensorInterval = 50;
int sensorInterval = 50;

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
// motor parameters (ONLY FOR REFERENCE) !!!
//#define J_L 1
//#define K_L 1.809
//#define B_L 16.17
//
//#define J_R 1
//#define K_R 1.809
//#define B_R 16.17

// controller parameters
#define KI_L 0.02
#define KP_L 0.8

#define KI_R 0.02
#define KP_R 0.8

///////////////////////////////////////
// OLD MOTOR PARAMS AND CONTROL VALS //
///////////////////////////////////////
/*
(J ALWAYS = 1)
----------------------------------------------
  DATE   |      LEFT       |      RIGHT
         |    K      B     |    K      B
----------------------------------------------
23-12-06 |  1.809,  16.17  |  1.809,  16.17  |

----------------------------------------------
  DATE   |       LEFT     |       RIGHT
         |    KI      KP  |     KI      KP
----------------------------------------------
23-12-06 |  0.020,  0.800  |  0.020,  0.800  |

*/

// controller variables
float omega_l = 0.0;
float dir_l = 0.0;      // omega is only positive so need a +/- flag: dir
float omega_l_des = 0;
float error_l = 0;
float u_integral_l = 0;
float u_l = 0;

float omega_r = 0.0;
float dir_r = 0.0;
float omega_r_des = 0;
float error_r = 0;
float u_integral_r = 0;
float u_r = 0;

// pointers
float *omega_l_ptr;
float *omega_r_ptr;

void setup() {
    Serial.begin(38400);
//    Serial.begin(9600);
    Serial.println("PROGRAM: wheel_speed_control_test_dual_NOPRINT.ino");
    Serial.println("UPLOAD DATE: 2024 NOV 05");
    Serial.println("BAUD 38400");

    Serial.print("Enc L ratio: ");
    Serial.println(ENC_0_GEAR_RATIO);
    Serial.print("Enc R ratio: ");
    Serial.println(ENC_1_GEAR_RATIO);

    Serial.print("KI_L: ");
    Serial.println(KI_L);
    Serial.print("KP_L: ");
    Serial.println(KP_L);
    Serial.print("KI_R: ");
    Serial.println(KI_R);
    Serial.print("KP_R: ");
    Serial.println(KP_R);
    Serial.print("T_controller [ms]: ");
    Serial.println(sensorInterval);

    // Serial.println("int_from_msg, Desired Speed [rad/s], Control Action [8bit], Tread Speed [rad/s]");
    Serial.print("omega_l_des [rad/s], omega_l [rad/s], error_l [rad/s], u_integral_l , u_l [8bit], ");
    Serial.println("omega_r_des [rad/s], omega_r [rad/s], error_r [rad/s], u_integral_r , u_r [8bit]");

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
    pinMode(ENC_0_INA, INPUT_PULLUP);
    pinMode(ENC_1_INA, INPUT_PULLUP);

    // attach interrupt for reading encoder
    attachInterrupt(digitalPinToInterrupt(ENC_0_INA), updateEncoder_0, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_1_INA), updateEncoder_1, RISING);

    // assign pointer addresses
    omega_l_ptr = &omega_enc_0;
    omega_r_ptr = &omega_enc_1;

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
        omega_enc_0 = (float)(encoder_0_count/actual_interval*1000*2*3.14159 / ENC_0_REV_COUNT * ENC_0_GEAR_RATIO);
        omega_enc_1 = (float)(encoder_1_count/actual_interval*1000*2*3.14159 / ENC_1_REV_COUNT * ENC_1_GEAR_RATIO);

        omega_enc_0 = abs(omega_enc_0);
        omega_enc_1 = abs(omega_enc_1);
        
        // encoders only wired for positive signal;
        // need to get direction from inputs:
        if (u_l < 0) {
            omega_enc_0 = -1 * omega_enc_0;
        }
        if (u_r < 0) {
            omega_enc_1 = -1 * omega_enc_1;
        }

        // reset encoder count
        encoder_0_count = 0;
        encoder_1_count = 0;

        //////////////////////////
        //     CONTROL LOOP     //
        //////////////////////////

        // left side ///////////////////////////////////////////
        error_l = omega_l_des - *omega_l_ptr;
        u_integral_l = u_integral_l + error_l*actual_interval;
        u_l = KP_L*error_l + KI_L*u_integral_l;

        if (u_l > 255) {
           u_l = 255;
        }
        if (u_l < -255) {
           u_l = -255;
        }

        tread_left.drive(u_l);

        // right side ///////////////////////////////////////////
        error_r = omega_r_des - *omega_r_ptr;
        u_integral_r = u_integral_r + error_r*actual_interval;
        u_r = KP_R*error_r + KI_R*u_integral_r;

        if (u_r > 255) {
           u_r = 255;
        }
        if (u_r < -255) {
           u_r = -255;
        }

        tread_right.drive(u_r);

//        ////////////////
//        // print data //
//        ////////////////
//        // left side  data
//        Serial.print(omega_l_des);
//        Serial.print(", ");
        Serial.print(*omega_l_ptr);
        Serial.print(", ");
//        Serial.print(error_l);
//        Serial.print(", ");
//        Serial.print(u_integral_l);
//        Serial.print(", ");
//        Serial.print(u_l);
//        Serial.print(", ");
//
//        // right side data
//        Serial.print(omega_r_des);
//        Serial.print(", ");
        Serial.print(*omega_r_ptr);
        Serial.print(", ");
//        Serial.print(error_r);
//        Serial.print(", ");
//        Serial.print(u_integral_r);
//        Serial.print(", ");
//        Serial.print(u_r);
//        Serial.print(", ");
//
        Serial.println("");
    } // end rpm/voltage/text-output loop
} // end main loop

///////////////////////////
//   encoder functions   //
///////////////////////////
void updateEncoder_0() {
    encoder_0_count ++;
}

void updateEncoder_1() {
    encoder_1_count ++;
    // dir_l = getDir();
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
            omega_r_des = int_from_msg;
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
