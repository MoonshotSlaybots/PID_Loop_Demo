#include <Stepper.h>

const int stepsPerRevolution = 200;
//stepper controller inputs are on digital pins 8-11
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

int stepCount = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read the sensor value:
  int sensorReading = analogRead(A0);
  Serial.println(sensorReading);
  
  // map it to a range from 0 to 100:
  int motorSpeed = map(sensorReading, 0, 1023, 0, 100);
  // set the motor speed:
  Serial.println(motorSpeed);
  
  if (motorSpeed > 0) {
    myStepper.setSpeed(motorSpeed);
    // step 1/200 of a revolution (one step):
    myStepper.step(stepsPerRevolution / 200);
  }
}
