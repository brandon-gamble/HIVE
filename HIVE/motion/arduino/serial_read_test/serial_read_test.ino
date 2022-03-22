

#include <SparkFun_TB6612.h>

// H-Bridge pins
#define AIN1 3
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// command IDs and associated variables
#define CMD_ID_MOTORS 0
int m0_speed = 0;
int m1_speed = 0;
#define CMD_ID_TURRET 1
int tur_rotation = 0;
int tur_elevation = 0;

// command time
// in case something hangs, motors will never run for longer
// than the CMD_TIME
#define CMD_TIME 1000

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

Motor motor0 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor1 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


void setup() {
	Serial.begin(9600);
	Serial.setTimeout(10);

	motor0.brake();
	motor1.brake();
	delay(1000);
}

void loop() {
	if (Serial.available() > 0) {
		Serial.println("serial buffer > 0");

		// parse command id, which informs how many variables will follow
		int cmd_id = Serial.parseInt();

//        // dump rest of buffer
//        while (Serial.available() > 0) {
//            char leftover = Serial.read();
//            Serial.print(leftover);
//        }
//        Serial.println("");

		// send command to motors
		switch (cmd_id) {
			case CMD_ID_MOTORS:
                // get motor speeds
                m0_speed = Serial.parseInt();
                m1_speed = Serial.parseInt();

                // set motor speeds
                motor0.drive(m0_speed, CMD_TIME);
                motor1.drive(m1_speed, CMD_TIME);
                
            case CMD_ID_TURRET:
                break;
            default:
                // if corrupt command is passed, stop tank
                motor0.brake();
                motor1.brake();
        } // end switch

	} // end serial read
} // end loop
