#include <Servo.h>

//Constants
const int setPosPot = A3;
const int currPosEncoder = A4;
const int Ppot = A0;
const int Ipot = A1;
const int Dpot = A2;

const int invertSetPosSwitch = 3;
const int enableMotorSwitch = 4;
const int servoPin = 5;

const int pEnablePin = 6; //TODO: setup other channel enable switches

const int maxPWM = 1750;  //in microseconds, the duty cycle PWM range to control speed
const int minPWM = 1250;  //this is specific to the HSR-1425CR servo, tweak as needed for others

//calculation variables
float setPos, currPos, pIn, iIn, dIn = 0.0f;   //analog inputs
bool enableMotor, invertSetPos, pEnable = false;     //switch inputs
float error, lastError, pid = 0.0f;    //TODO: label
float p, i, d = 0.0f;       //values for p, i, and d factors
float Kp, Ki, Kd = 0.0f;    //coefficents 

//last interation of inputs for filtering
float lastSetPos, lastCurrPos, lastPIn, lastIIn, lastDIn = 0.0f;

long now = 0L;

Servo servo;

/**
   initialize arduino for operation
*/
void setup() {
  Serial.begin(9600);   //start serial communication for console
  now = millis() + 500;
  
  //input setup
  pinMode(invertSetPosSwitch, INPUT);
  pinMode(enableMotorSwitch, INPUT);
  pinMode(pEnablePin, INPUT_PULLUP);

  //motor setup
  servo.attach(servoPin);
}


/**
 * Main loop that controls the motor
 */
void loop() {
  Serial.println(digitalRead(pEnablePin) == HIGH);
  
  //set last values
  lastPIn = pIn;
  lastIIn = iIn;
  lastDIn = dIn;
  lastCurrPos = currPos;  
  
  //first read all the inputs into the system
  pIn = analogReadFiltered(Ppot, lastPIn, 0.9);   //range 0 to 1024
  iIn = analogReadFiltered(Ipot, lastIIn, 0.9);
  dIn = analogReadFiltered(Dpot, lastDIn, 0.9);
  setPos = analogReadFiltered(setPosPot, lastSetPos, 0.9);
  
  lastSetPos = setPos;  //TODO: clean this up, order things better and comment

  //setPos = analogRead(setPosPot);
  
  enableMotor = digitalRead(enableMotorSwitch) == HIGH;
  invertSetPos = digitalRead(invertSetPosSwitch) == HIGH;

  //read set and current position 
  //normal range int 0 to 1024, make range float -180 to 180
  setPos = ((setPos / 1024.0) * 360) - 180;
  currPos = readAnalogAngle(currPosEncoder, lastCurrPos, 0.9);


  //invert setPos if needed
  if(invertSetPos){
    //turn 180 degrees depending on the current state of the set position
    if(setPos > 0){
      setPos -= 180;
    }else{
      setPos += 180;
    }
  }
  
  //begin PID loop calculations

  //Scale the coefficients before they are passed in
  //from the input values (trial and error for scaling)
  Kp = pIn / 4096.0f;     // 0 to 1
  Ki = iIn / 16384.0f;     // 0 to 0.5
  Kd = dIn / 4096.0f ;    // 0 to 1
  
  error = currPos - setPos;    //error is degrees difference between where we want to be and where we are

  p = error;              //proportional is equal to the amount of error measured

  //i = i + error;         //integral accumulates the error each iteration
  
  //Optional: create a dead band so the so integrel won't hunt back and fourth
  if(abs(error) >  5) i = i + error;  // Integrate error if error > 1
  if(error < 5.0 && error > -5.0) i = 0;               //Clear intergal if zero error

  i = constrain(i, -500.0f, 500.0f);      //stop i from increasing without bound

  d = error - lastError;              //derivitive is the rate of change of the error
  lastError = error;                  //save this error for the next loop

  //sum the PID factors with their coeffients
  pid = (Kp * p) + (Ki * i) + (Kd * d);


  //only move motor if enabled switch is on
  if(enableMotor){
    setServoSpeed(pid);
  }else{
    setServoSpeed(0);
  }

  //debug print twice a second
  if(millis() >= now){
    //debug();
    now += 500;
  }

  //loop 100 times per second
  delay(10);
}

//reads the current angle (-180 to 180) of the analog encoder on the given pin
//takes n samples and averages the output
float readAnalogAngle(int pin, float lastVal, float weight){
  const float minVoltage = 0.015;
  const float maxVoltage = 4.987;

  float raw = analogRead(pin) / 1024.0f * 5.0f;
  float angle = (raw/maxVoltage * 360) - 180;
  
  float newVal = weight * lastVal + (1 - weight) * angle; 

  return newVal;
}

//reads an analog value and weights it against its previous value
//this smooths out the signal
float analogReadFiltered(int pin, float lastVal, float weight){
  int raw = analogRead(pin);
  float newVal = weight * lastVal + (1 - weight) * raw;
  return newVal;
}

/*
 * sets the servo speed, -1 for full reverse, 1 for full forward
 */
void setServoSpeed(float s){
  s = constrain(s, -1.0, 1.0);
  servo.writeMicroseconds(minPWM + (s+1)/2.0 * (maxPWM - minPWM));
}

//print out all important values
void debug(){

  //-----------------inputs-----------------

  Serial.print("Set pos: ");
  Serial.print(setPos);
  Serial.print("    ");

  Serial.print("curr pos: ");
  Serial.print(currPos);
  Serial.print("    ");
  
  Serial.print("pIn: ");
  Serial.print(pIn);
  Serial.print("    ");

  Serial.print("iIn: ");
  Serial.print(iIn);
  Serial.print("    ");

  Serial.print("dIn: ");
  Serial.print(dIn);
  Serial.print("    ");

  Serial.print("invert set pos: ");
  Serial.print(invertSetPos);
  Serial.print("    "); 
  
  Serial.print("enable motor: ");
  Serial.print(enableMotor);
  Serial.print("    "); 
  

  //-----------------coeffients-----------------

  Serial.print("Kp: ");
  Serial.print(Kp, 4);
  Serial.print("    ");

  Serial.print("Ki: ");
  Serial.print(Ki, 4);
  Serial.print("    ");

  Serial.print("Kd: ");
  Serial.print(Kd, 4);
  Serial.print("    ");

  //-----------------calculations-----------------

  Serial.print("error: ");
  Serial.print(error, 4);
  Serial.print("    ");
  
  Serial.print("p: ");
  Serial.print(p, 4);
  Serial.print("    ");

  Serial.print("i: ");
  Serial.print(i, 4);
  Serial.print("    ");

  Serial.print("d: ");
  Serial.print(d, 4);
  Serial.print("    ");

  Serial.print("pid: ");
  Serial.print(pid, 4);
  Serial.print("    ");

  Serial.println();

}
