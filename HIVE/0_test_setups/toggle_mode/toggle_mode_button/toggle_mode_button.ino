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

void setup() {
  pinMode(buttonPin, INPUT);

  pinMode(mode0_pin, OUTPUT);
  pinMode(mode1_pin, OUTPUT);

  // set initial LED state
  digitalWrite(mode0_pin, LOW);
  digitalWrite(mode1_pin, LOW);
  Serial.begin(9600);
}

void loop() {
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
