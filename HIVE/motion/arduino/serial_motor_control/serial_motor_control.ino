int flag = 0;
/////////////////////////////////////
//        serial comm vars         //
/////////////////////////////////////

const byte msg_length = 16;       // max message size
char chars_received[msg_length];   // char array to store message
char temp_chars[msg_length];       // char array for parsing

// hold parsed data
char char_from_msg[msg_length] = {0};
int int_from_msg = 0;

boolean new_data = false;

/////////////////////////////////////
//       motor control vars        //
/////////////////////////////////////

# include <BJG_TB6612FNG.h>

/////////////////////////////
/////////////////////////////
// H-Bridge pins
#define AIN1 7
#define AIN2 6
#define PWMA 5

#define BIN1 9
#define BIN2 10
#define PWMB 11

#define STBY 8
/////////////////////////////
/////////////////////////////

// used to flip motor configuration without rewiring
// if motor is spinning opposite direction of intention, flip sign
const int polarity_A = 1;
const int polarity_B = 1;

Motor tread_left = Motor(AIN1, AIN2, PWMA, polarity_A, STBY);
Motor tread_right = Motor(BIN1, BIN2, PWMB, polarity_B, STBY);


void setup(){
    Serial.begin(38400);
//    Serial.begin(9600);

    // disable standby (turn on motors)
    tread_left.standby(LOW);
    tread_right.standby(LOW);

    Serial.println("PROGRAM: serial_motor_control.ino");
    Serial.println("UPLOAD: 2022-04-15");

    Serial.print("flag: ");
    Serial.println(flag);
}

void loop() {
    receive_message();
    if (new_data == true) {
        // temporary copy to preserve original data
        // bc strtok() in parse_msg() replaces commas with \0
        strcpy(temp_chars, chars_received);
        parse_msg();
        //show_parsed_msg();
        send_commands();
        new_data = false;

        Serial.println(flag);
    }
}

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
            flag = int_from_msg;
            break;
        case 'R':                                   // 'R' = right tread
            // right tread
            tread_right.drive(int_from_msg);
            flag = int_from_msg;
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
