#define joyX A0
#define joyY A1

const int ledPin = 3; //led pin on PWM pin 3
const int maxBrightness = 120; // led max bright

int initial_xValue_zero = 500;

void setup() {
  Serial.begin(38400);
  pinMode(ledPin, OUTPUT);
  Serial.println("------------------------------------------");
  Serial.println("SKETCH: joystick_led_control.ino");
  Serial.println("UPLOAD DATE: 2024 NOV 14");
  Serial.println("joystick center is 0, joystick right (away from pins) is 255");
  Serial.println("led on pin 3");
  Serial.print("led max brightness: ");
  Serial.println(maxBrightness);
  Serial.println("------------------------------------------");
  initial_xValue_zero = analogRead(joyX);
}
 
void loop() {
  // put your main code here, to run repeatedly:
  int xValue = analogRead(joyX);
  int yValue = analogRead(joyY);
 
  //print the values with to plot or view
//  Serial.print("raw X: ");
//  Serial.print(xValue);
//  Serial.print("\t");
//  Serial.print("| raw Y: ");
//  Serial.println(yValue);

//  int ledValue = map(xValue, 0,1023, 0,255);
//  int ledValue = map(xValue, 530,6, 0,250);
  // map(val, fromLow, fromHigh, toLow, toHigh)
  int ledValue = map(xValue, initial_xValue_zero, 1023, 0,maxBrightness);
  if (ledValue < 0){
    ledValue = 0;
  }
  analogWrite(ledPin, ledValue);
  String ledCommand = "<" + String(ledValue) + ">";
  Serial.println(ledCommand);
  delay(30); // at 9600 baud, cannot decrease delay less than 20ms

}
