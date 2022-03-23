/***********************
serial com with expanded functionality:
    - start and end markers
    - parsing for multiple pieces of data
    - e.g. message: "<w, 150>"
        - "<"   start marker
        - "w"   char
        - ","   delimiter
        - "150" integer
        - ">"   end marker
    - application:
        - for tank, each actuator will have an associated char
        - parse for char to know which actuator you want to affect
        - parse for integer to know what value you want to send to that actuator
***********************/

const byte msg_length = 16;       // max message size
char chars_received[msg_length];   // char array to store message
char temp_chars[msg_length];       // char array for parsing

// hold parsed data
char char_from_msg[msg_length] = {0};
int int_from_msg = 0;

boolean new_data = false;

void setup(){
    Serial.begin(9600);
    Serial.println("expecting text followed by integer.")
    Serial.println();
}

void loop() {
    recvWithStartend_markers();
    if (new_data == true) {
        // temporary copy to preserve original data
        // bc strtok() in parseData() replaces commas with \0
        strcpy(temp_chars, chars_received);
        showParsedData();
        new_data = false;
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
    Serial.print("message: ");
    Serial.println(char_from_msg);
    Serial.print("int: ");
    Serial.println(int_from_msg);
}
