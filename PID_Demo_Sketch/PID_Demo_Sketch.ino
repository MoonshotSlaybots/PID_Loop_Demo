#include <Servo.h>

//Constants
const int setPositionPot = 3;
const int currPositionEncoder = 4;
const int Ppot = 0;
const int Ipot = 1;
const int Dpot = 2;

const int invertSetPositionSwitch = 3;
const int enableMotorSwitch = 4;

const int maxPWM = 1750;  //in microseconds, the duty cycle PWM range to control speed
const int minPWM = 1250;  //this is specific to the HSR-1425CR servo, tweak as needed for others
const int servoPin = 2;

//calculation variables
float setPosition, currPosition, pIn, iIn, dIn = 0;   //inputs
long error, lastError, pid, now = 0;
long p, i, d = 0;       //values for p, i, and d factors
long Kp, Ki, Kd = 0;    //coefficents 

Servo servo;

/**
   initialize arduino for operation
*/
void setup() {
  Serial.begin(9600);   //start serial communication for console
  
  //input setup
  pinMode(invertSetPositionSwitch, INPUT);
  pinMode(enableMotorSwitch, INPUT);

  //motor setup
  servo.attach(servoPin);

}


/**
 * Main loop that controls the motor
 */
void loop() {
  //first read all the inputs into the system
  pIn = analogRead(Ppot);   //range 0 to 1024
  iIn = analogRead(Ipot);
  dIn = analogRead(Dpot);

  //normally range 0 to 1024, make range -512 to +511
  setPosition = (analogRead(setPositionPot)/512.0 * 180.0 ) - 180;
  currPosition = readAnalogAngle(currPositionEncoder, 5);
  
  //invert setPosition if needed
  if(digitalRead(invertSetPositionSwitch) == HIGH){
    //TODO: fix inversion, instead of 1 to -1, needs to be 1 to -179 degrees
    setPosition = setPosition * -1;
  }
  

  //begin PID loop calculations

  //Create and Scale the coefficients 
  //from the input values (trial and error for scaling)
  Kp = pIn / 4;      // 0 to 255
  Ki = iIn / 4;     // 0 to 255
  Kd = dIn ;      // 0 to 1023
  
  error = currPosition - setPosition;    //error is differene between where we want to be and where we are

  p = error;              //proportional is equal to the amount of error

  i = i + error;         //integreal accumulates the error each iteration
  
  //Optional: create a dead band so the so integrel won't hunt back and fourth
  //if(abs(error) >  1) i = i + error;  // Integrate error if error > 1
  //if(error == 0) i = 0;               //Clear intergal if zero error

  i = constrain(i, -2000, 2000);      //stop i from increasing without bound

  d = error - lastError;              //rate of change of the error
  lastError = error;                  //save this error for the next loop

  //sum the PID factors with their coeffients
  pid = (Kp * p) + (Ki * i) + (Kd * d);

  

  //only move motor if enabled switch is on
  if(digitalRead(enableMotorSwitch) == HIGH){
    
  }  


  //temp testing servo and encoder
  setServoSpeed(setPosition / 512.0);
  //Serial.println(readAnalogAngle(currPositionEncoder, 1));
  debug();



}

//reads the current angle (0 to 360) of the analog encoder on the given pin
//takes n samples and averages the output
float readAnalogAngle(int pin, int numSamples){
  const float minVoltage = 0.015;
  const float maxVoltage = 4.987;
  int samples[numSamples]; //0-1023

  //take samples
  for(int i=0; i<numSamples; i++){
    samples[i] = analogRead(pin); //0-1023
  }

  //find average
  float sum = 0.0;
  for(int i=0; i<numSamples; i++){
    sum += samples[i];
  }
  float voltage = (sum / numSamples)/1024 * 5.0; 

  //convert from voltage to angle
  return (voltage/maxVoltage * 360) - 180;
}

/*
 * sets the servo speed, -1 for full reverse, 1 for full forward
 */
void setServoSpeed(float s){
  servo.writeMicroseconds(minPWM + (s+1)/2.0 * (maxPWM - minPWM));
}

//print out all important values
void debug(){
  /*
  Serial.print("error = " + error);
  Serial.print("    ");
  Serial.print("Kp = " + Kp);
  Serial.print("    ");
  Serial.print("Ki = " + Ki);
  Serial.print("    ");
  Serial.print("Kd = " + Kd);
  Serial.print("    ");
  Serial.print("pid = " + pid);
  Serial.print("    ");
  Serial.print("p = " + p);
  Serial.print("    ");
  Serial.print("i = " + i);
  Serial.print("    ");
  Serial.print("d = " + d);
  Serial.println(" ");
  */

  Serial.print("Set pos: ");
  Serial.print(setPosition);
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
  
  Serial.print("invert motor: ");
  Serial.print(digitalRead(invertSetPositionSwitch) == HIGH);
  Serial.print("    "); 
  
  Serial.print("enable motor: ");
  Serial.print(digitalRead(enableMotorSwitch) == HIGH);
  Serial.print("    "); 

  Serial.println();

}
