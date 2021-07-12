/**
 * Reads the value of a potentiometer connected to analog pin 0
 * to control the speed of a continous servo plugged into digital pin 2
 */
#include <Servo.h>

int potPin = 3;   //input pin for the potentiometer
int val = 0;      //current value of the potentiometer

int servoPin = 2;   //PWM pin for the servo
Servo servo;
int maxPWM = 1750;  //in microseconds, the duty cycle PWM range to control speed
int minPWM = 1250;  //this is specific to the HSR-1425CR servo, tweak as needed for others

void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
}

void loop() {
  val = analogRead(potPin);     //read value from pot
  //Serial.println(val);
  //scale pot value to be between -1 and 1
  float s = (val - 512) / 512.0;
  Serial.println(s);
  setServoSpeed(s);
}

void setServoSpeed(float s){
  servo.writeMicroseconds(minPWM + (s+1)/2.0 * (maxPWM - minPWM));
}
