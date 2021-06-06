#include <Stepper.h>

const int stepsPerRevolution = 200;
//stepper controller inputs are on digital pins 8-11
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

long lastStep = 0;

void setup() {
  Serial.begin(9600);
  myStepper.setSpeed(100);
}

void loop() {
  // read the sensor value:
  int sensorReading = analogRead(A0);
  float speed = sensorReading/512.0 - 1.0;
  Serial.println(speed);
    
  checkStep(speed);
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
