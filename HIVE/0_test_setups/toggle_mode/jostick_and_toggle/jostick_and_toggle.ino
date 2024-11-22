/////////////////////////
// toggle button setup //
/////////////////////////

/*
  - LED attached from pin 3 to ground through 220 ohm resistor
  - LED attached from pin 4 to ground through 220 ohm resistor
  
  - pushbutton attached from pin 5 to +5V
  - 10 kilohm resistor attached from pin 2 to ground
*/

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 5;    // the number of the pushbutton pin

// Variables will change:
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

int button_press_counter = 0;// will be used to count 
const int NUM_MODES = 2;           // how many modes will be used
int current_mode = 0;
// ref: mode 0 = follow
//      mode 1 = wait
const int mode0_pin = 3;
const int mode1_pin = 4;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

///////////////////////////////////////////////////////////
// joystick setup //
////////////////////
#define joyX A0
#define joyY A1

int xCenter = 0;
int yCenter = 0;

int xValue = 0;
int yValue = 0;

int left_val = 0;
int right_val = 0;

int joystick_range = 1024/2;
int x_lo = 0;
int x_hi = 0;

int y_lo = 0;
int y_hi = 0;

void setup() {
  ////////////////////
  // toggle button setup
  ////////////////////
  pinMode(buttonPin, INPUT);

  pinMode(mode0_pin, OUTPUT);
  pinMode(mode1_pin, OUTPUT);

  // set initial LED state
  digitalWrite(mode0_pin, LOW);
  digitalWrite(mode1_pin, LOW);
  Serial.begin(38400);

  ////////////////////
  // joystick setup //
  ////////////////////
  // find values at joystick rest
  xCenter = analogRead(joyX);
  yCenter = analogRead(joyY);

  x_lo = xCenter - joystick_range;
  x_hi = xCenter + joystick_range;

  y_lo = yCenter - joystick_range;
  y_hi = yCenter + joystick_range;
}

void loop() {

  //////////////////////////////////////////////////////////////
  // joysstick stuff
  // get values from joystick
  xValue = analogRead(joyX);
  yValue = analogRead(joyY);

  // map joystick values to motor command values
  xValue =  map(xValue, x_lo,x_hi, -255,255);
  yValue = map(yValue, y_hi,y_lo, -255,255);

  // rotate joystick 45deg
  // https://robotics.stackexchange.com/questions/20347/how-do-i-convert-centre-returning-joystick-values-to-dual-hobby-motor-direction
  left_val =  int(+xValue*sqrt(2.0)/2.0 + yValue*sqrt(2.0)/2.0);
  right_val = int(-xValue*sqrt(2.0)/2.0 + yValue*sqrt(2.0)/2.0);

  // create command strings
  String cmd_left =  "<L," + String(left_val) +  ">";
  String cmd_right = "<R," + String(right_val) + ">";

  // send command strings 
  Serial.print(cmd_left);
  Serial.println(cmd_right);


  //////////////////////////////////////////////////////////////
  // toggle button stuff
  
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        button_press_counter = button_press_counter + 1;

        current_mode = button_press_counter % NUM_MODES;
        Serial.println(button_press_counter);
        Serial.println(current_mode);

        if (current_mode == 0){
          Serial.println("FOLLOW MODE");
          digitalWrite(mode0_pin, HIGH);
          digitalWrite(mode1_pin, LOW);
          
        } else if (current_mode == 1) {
          Serial.println("WAIT MODE");
          digitalWrite(mode0_pin, LOW);
          digitalWrite(mode1_pin, HIGH);
        }
        
      }
    }
  }

//  // set the LED:
//  digitalWrite(ledPin, ledState);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}
