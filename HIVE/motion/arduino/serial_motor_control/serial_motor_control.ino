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

// H-Bridge pins
#define AIN1 3
#define AIN2 4
#define PWMA 5
#define BIN1 7
#define BIN2 8
#define PWMB 6
#define STBY 9

// used to flip motor configuration without rewiring
// if motor is spinning opposite direction of intention, flip sign
const int polarity_A = 1;
const int polarity_B = 1;

Motor tread_left = Motor(AIN1, AIN2, PWMA, polarity_A, STBY);
Motor tread_right = Motor(BIN1, BIN2, PWMB, polarity_B, STBY);


void setup(){
    Serial.begin(9600);

    // disable standby (turn on motors)
    tread_left.standby(LOW);
    tread_right.standby(LOW);
}

void loop() {
    recvWithStartend_markers();
    if (new_data == true) {
        // temporary copy to preserve original data
        // bc strtok() in parseData() replaces commas with \0
        strcpy(temp_chars, chars_received);
        showParsedData();
        new_data = false;

        char act_id = char_from_msg;
        int act_val = int_from_msg;

        // process and send actuation commands
        switch(act_id) {
            case 'L':                                   // 'L' = left tread
                // left tread
                tread_left.drive(act_val);
                break;
            case 'R':                                   // 'R' = right tread
                // right tread
                tread_right.drive(act_val);
                break;
            case 'S':                                   // 'S' = standby
                if (act_val == 0) {
                    // standby = false so turn on
                    tread_left.standby(LOW);
                    tread_right.standby(LOW);
                } else if (act_val == 1) {
                    // standby = true so turn off
                    tread_left.standby(HIGH);
                    tread_right.standby(HIGH);
                }
            default:
                // if unknown actuator id is received, brake and standby
                tread_left.brake();
                tread_right.brake();
                tread_left.standby(HIGH);
                tread_right.standby(HIGH);
        }
    }
}

void recvWithStartend_markers() {
    static boolean recv_in_progress = false;
    static byte buffer_index = 0;
    char start_marker = '<';
    char end_marker = '<';
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

void parseData() {
    char * strtok_index; // index for strtok()

    strtok_index = strtok(temp_chars,","); // get string
    strcpy(char_from_msg, strtok_index);  // copy string to char_from_msg

    strtok_index = strtok(NULL, ",");     // continuing from previous indx, get int
    int_from_msg = atio(strtok_index);   // convert to int
}

void showParsedData() {
    Serial.print("act_id: ");
    Serial.print(char_from_msg);
    Serial.print("\t")
    Serial.print("act_val: ");
    Serial.println(int_from_msg);
}
