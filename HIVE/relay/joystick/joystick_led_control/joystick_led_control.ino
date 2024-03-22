#define joyX A0
#define joyY A1
 
void setup() {
  Serial.begin(9600);
}
 
void loop() {
  // put your main code here, to run repeatedly:
  int xValue = analogRead(joyX);
  int yValue = analogRead(joyY);
 
  //print the values with to plot or view
//  Serial.print(xValue);
//  Serial.print("\t");
//  Serial.println(yValue);

//  int ledValue = map(xValue, 0,1023, 0,255);
  int ledValue = map(xValue, 530,6, 0,250);
  String ledCommand = "<" + String(ledValue) + ">";
  Serial.println(ledCommand);
  delay(30); // at 9600 baud, cannot decrease delay less than 20ms

}
