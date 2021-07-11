#include "HalfStepper.h"

const int stepsPerRevolution = 200;
//stepper controller inputs are on digital pins 8-11
HalfStepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

long lastStep = 0;

void setup() {
  Serial.begin(9600);
  myStepper.setSpeed(0);
}

void loop() {
  // read the sensor value:
  int sensorReading = analogRead(A0);
  float speed = sensorReading/4 - 100.0;
  Serial.println(speed);

  myStepper.setSpeed(speed);
  if(speed<0) {
    myStepper.step(-1);
  }else{
    myStepper.step(1);
  }

  
}

//step only if its been long enough since the last step
//else returns immediatly
void checkStep(float speed){
  int steps = 1;
  
  if(speed != 0){
    if (speed < 0){ 
      speed *= -1;
      steps *= -1;
    }

    //1 speed = 0 delay, 0 speed = 200 ms delay
    float delay = -200.0 * speed + 200;
  
    if(lastStep + delay <= millis()){
      lastStep = millis();
      myStepper.step(steps);
    } 
  }
}
