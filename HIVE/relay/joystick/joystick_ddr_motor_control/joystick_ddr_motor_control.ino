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
  Serial.begin(9600);

  // find values at joystick rest
  xCenter = analogRead(joyX);
  yCenter = analogRead(joyY);

  x_lo = xCenter - joystick_range;
  x_hi = xCenter + joystick_range;

  y_lo = yCenter - joystick_range;
  y_hi = yCenter + joystick_range;
}
 
void loop() {
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
  delay(30); // at 9600 baud, cannot decrease delay less than 20ms
}
