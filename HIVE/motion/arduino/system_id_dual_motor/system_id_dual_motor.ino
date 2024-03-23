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

long curr_motorTimer = 0;
long prev_motorTimer = 0;
int motorInterval = 4000;

int command_step = 0;
int prev_command_val = 0;
int curr_command_val = 0;

//////////////////////////////////
//     ROTARY ENCODER SETUP     //
//////////////////////////////////

//////////////////////
// ENCODER 0 - LEFT //
//////////////////////

// pulses per rev of encoder
#define ENC_0_REV_COUNT 360

// gear ratio between encoder and output shaft
//const float ENC_0_GEAR_RATIO = 0.86253;
const float ENC_0_GEAR_RATIO = 1;

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
//const float ENC_1_GEAR_RATIO = 1.1428;
const float ENC_1_GEAR_RATIO = 1;

// encoder outout A to arduino
#define ENC_1_INA 3

// encoder pulse count
volatile long encoder_1_count = 0;

// motor speed [rad/s]
float omega_enc_1 = 0;

////////////
// TIMERS //
////////////

// interval for measurements
float sensorInterval = 50;

// time tracking
long prev_sensorTimer = 0;
long curr_sensorTimer = 0;
float actual_interval = 0;
float elapsed_time_sec = 0;



void setup() {
    Serial.begin(38400);
    Serial.println("PROGRAM: system_id_dual_motor.ino");
    Serial.println("UPLOAD DATE: 2024 MAR 19");

    Serial.print("Sample time [ms]: ");
    Serial.println(sensorInterval);
    Serial.println("Time elapsed [s], Command Value, omega_enc_0_L [rad/s], omega_enc_1_R [rad/s]");

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

    // set initial time
    prev_sensorTimer = millis();
    prev_motorTimer = millis();

}

void loop(){
    curr_sensorTimer = millis();
    if (curr_sensorTimer - prev_sensorTimer > sensorInterval) {
        actual_interval = curr_sensorTimer - prev_sensorTimer;
        prev_sensorTimer = curr_sensorTimer;

        elapsed_time_sec = float(curr_sensorTimer)/1000.0;

        /////////////////////////////
        //     ROTARY ENCODER      //
        /////////////////////////////

        // calc motor speed [rad/s]
        omega_enc_0 = (float)(encoder_0_count/actual_interval*1000*2*3.14159 / ENC_0_REV_COUNT * ENC_0_GEAR_RATIO);
        omega_enc_1 = (float)(encoder_1_count/actual_interval*1000*2*3.14159 / ENC_1_REV_COUNT * ENC_1_GEAR_RATIO);

        // reset encoder count
        encoder_0_count = 0;
        encoder_1_count = 0;

        ////////////////
        // PRINT DATA //
        ////////////////
        Serial.print(elapsed_time_sec);
        Serial.print(", ");
        Serial.print(curr_command_val);
        Serial.print(", ");
        Serial.print(omega_enc_0);
        Serial.print(", ");
        Serial.print(omega_enc_1);
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
            tread_right.drive(curr_command_val);
        }
    } // end motor timer if loop


} // end main loop

///////////////////////////
//   encoder functions   //
///////////////////////////
void updateEncoder_0() {
    encoder_0_count ++;
}

void updateEncoder_1() {
    encoder_1_count ++;
}
