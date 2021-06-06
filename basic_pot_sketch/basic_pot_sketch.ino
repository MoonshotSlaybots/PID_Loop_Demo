/**
 * Reads the value of a potentiometer connected to analog pin 2
 * and prints it out to the serial port 10 times per second
 */

int potPin =0;   //input pin for the potentiometer
int val = 0;      //current value of the potentiometer

void setup() {
  Serial.begin(9600);

}

void loop() {
  val = analogRead(potPin);     //read value from pot
  Serial.print(val);            //print value to serial
  Serial.print("\n");
  delay(100);
}
