#include <Servo.h>

//----------Constants----------

//defines where the inputs are plugged in
const int Ppot = A0;
const int Ipot = A1;
const int Dpot = A2;
const int setPosPot = A3;
const int currPosEncoder = A4;

//digital inputs
const int invertSetPosSwitch = 3;
const int enableMotorSwitch = 4;
const int pEnablePin = 6;
const int iEnablePin = 7;
const int dEnablePin = 8;

//servo output pin 
const int servoPin = 5;

const int maxPWM = 1750;  //in microseconds, the duty cycle PWM range to control speed
const int minPWM = 1250;  //this is specific to the HSR-1425CR servo, tweak as needed for others

Servo servo;    //output motor

//calculation variables
float setPos, currPos, pIn, iIn, dIn = 0.0f;   //analog inputs
float lastSetPos, lastCurrPos, lastPIn, lastIIn, lastDIn = 0.0f; //last inputs for smoothing

bool enableMotor, invertSetPos, pEnable, iEnable, dEnable = false;     //digital inputs
float error, lastError;    //error is the difference between the current position, and the set point
float p, i, d = 0.0f;       //values for p, i, and d factors
float Kp, Ki, Kd = 0.0f;    //coefficients to scale the p, i, and d factors, these are used to tune the PID controller
float pid = 0.0f;           //the final value passed to the motor to set the current speed

long now = 0L;  //number of milliseconds since the arduino turned on
                //for limiting number of calculations per second

/*
 * initialize arduino for operation
 */
void setup() {
  now = millis() + 500;
  
  //setup input for required analog
  //and ON-OFF digital switches
  pinMode(invertSetPosSwitch, INPUT);
  pinMode(enableMotorSwitch, INPUT);

  /*A pullup resistor is required for these switches because they are ON-OFF
   *switches, where the other are ON-ON switches.
   *the ON-OFF switches tie the pin to ground when in the on position and disconnects it from ground
   *when in the off position. This state is called a *floating pin* and makes reading
   *an input from the pin unreliable due to background noise on the board.
   *A pullup resistor is connected internally on the arduino to the 5v rail and this pin,
   *letting only a small amount of current flow through it.
   *With this, when the pin is not connected to ground, it is connected to 5v
   *through the pullup resistor, making the off position (5v) much more stable for the switch.
   *The ON-ON switch connects either ground or 5v to the pin and the floating
   *pin issue never happens, so a pullup resistor is not needed.
  */
  pinMode(pEnablePin, INPUT_PULLUP);
  pinMode(iEnablePin, INPUT_PULLUP);
  pinMode(dEnablePin, INPUT_PULLUP);

  //motor setup
  servo.attach(servoPin);

  //Serial output setup, only print the heading for the type being called
  //in loop
  Serial.begin(9600);   //start serial communication for console
  Serial.println("\n");
  
  //CSV output setup
  //Serial.println("time,setPos,currPos,p,i,d,Kp,Ki,Kd,PID");
  
  //serial plotter output setup
  Serial.println("setPos,currPos");
}


/**
 * Main loop that controls the motor
 */
void loop() {    
  //first read all the inputs into the system
  pIn = analogReadSmoothed(Ppot, lastPIn, 0.9);   //range 0 to 1024
  iIn = analogReadSmoothed(Ipot, lastIIn, 0.9);
  dIn = analogReadSmoothed(Dpot, lastDIn, 0.9);
  setPos = analogReadSmoothed(setPosPot, lastSetPos, 0.2);        //dont use as much smoothing for angle
  currPos = readEncoderAngle(currPosEncoder, lastCurrPos, 0.2);   //due to it switching from 180 to -180 quickly

  //set last values for next loop
  lastPIn = pIn;
  lastIIn = iIn;
  lastDIn = dIn;
  lastCurrPos = currPos;  
  lastSetPos = setPos;

  //scale set position 
  //normal range 0 to 1024, make range -180.0f to 180.0f
  setPos = ((setPos / 1024.0f) * 360) - 180;

  //read in digital inputs
  enableMotor = digitalRead(enableMotorSwitch) == HIGH;
  invertSetPos = digitalRead(invertSetPosSwitch) == HIGH;
  pEnable = digitalRead(pEnablePin) == HIGH;
  iEnable = digitalRead(iEnablePin) == HIGH;
  dEnable = digitalRead(dEnablePin) == HIGH;

  //invert setPos if needed
  if(invertSetPos){
    //turn 180 degrees depending on the current state of the set position
    if(setPos > 0){
      setPos -= 180;
    }else{
      setPos += 180;
    }
  }
  
  //--------------begin PID loop calculations--------------------

  //Scale the coefficients before they are passed in
  //from the input values (trial and error for scaling)
  //starts as 0 to 1023
  
  //These coefficients are converting from an angle measurement (-180 to 180)
  //to a motor power (-1 to 1). Because the scale of the two units are far apart
  //a small number is required to be in a usable range.
  //If the scale of the units were more similar, say from -5 to 5, to -1 to 1
  //then a Kp closer to 1 would be used
  
  Kp = pIn / 1024.0f * 0.08f;    //based on motor speed input
  Ki = iIn / 1024.0f * 0.001f;   //based on max i value (constrained later)
  Kd = dIn / 1024.0f * 0.08f;     //slows the motor down as it approaches set point
  
  error = currPos - setPos;    //error is the degrees difference between where we want to be and where we are
  //if error > 180 or < -180, it is faster to go the other way around
  if(error > 180){
    error = error - 360;
  }else if(error < -180){
    error = 360 + error;
  }

  p = error;              //proportional is equal to the amount of error measured

  //i = i + error;         //integral accumulates the error each iteration
  
  //Optional: create a dead band so the integrel won't hunt back and fourth
  if(abs(error) >  1) i = i + error;              // accumulate error if error > 1
  if(error < 1.0 && error > -1.0) i = 0;          //Clear intergal if between -1 and 1 error

  i = constrain(i, -200.0f, 200.0f);      //stop i from increasing without bound

  d = error - lastError;              //derivitive is the rate of change of the error
  lastError = error;                  //save current error for the next loop calculation of d

  //disable coefficients
  if(pEnable == false){
    Kp=0;
  }
  if(iEnable == false){
    Ki=0;
  }
  if(dEnable == false){
    Kd=0;
  }

  //sum the PID factors with their coefficients to get final motor speed
  pid = (Kp * p) + (Ki * i) + (Kd * d);


  //only move motor if enabled switch is on
  if(enableMotor){
    setServoSpeed(pid);
  }else{
    setServoSpeed(0);
  }

  //-------------------output-----------------------

  //debug print twice a second
  if(millis() >= now){
    //debug();
    now += 500;
  }

  //csvOut();
  plotterOut();
  
  //run loop 100 times per second
  delay(10);
}

//reads the current angle (-180 to 180) of the analog encoder on the given pin
//takes in the last value from this function for smoothing
//weight 0.0 to 1.0, 0.0=no smoothing, 1.0=output will not change from previous
float readEncoderAngle(int pin, float lastVal, float weight){
  const float minVoltage = 0.015;   //from encoder MA3-A10-250-N spec sheet
  const float maxVoltage = 4.987;

  float raw = analogRead(pin) / 1024.0f * 5.0f;
  float angle = (raw/maxVoltage * 360) - 180;

  //Smoothing function
  float newVal = weight * lastVal + (1 - weight) * angle; 

  return newVal;
}

/*
 * reads an analog value and weights it against its previous value
 * this smooths out the signal
 * 0.0 to 1.0, 0.0=no smoothing, 1.0=output will not change from previous
 */
float analogReadSmoothed(int pin, float lastVal, float weight){
  int raw = analogRead(pin);
  //Smoothing function
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

/*
 * print out all values 
 */
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
  

  //-----------------coefficients-----------------

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

void csvOut(){

  //Serial.println("time, setPos, currPos, p, i, d, kP, kI, kD, PID");
  int precision = 8;

  Serial.print(millis()/1000.0f, precision);
  Serial.print(",");
  Serial.print(setPos, precision);
  Serial.print(",");
  Serial.print(currPos, precision);
  Serial.print(",");
  Serial.print(p, precision);
  Serial.print(",");
  Serial.print(i, precision);
  Serial.print(",");
  Serial.print(d, precision);
  Serial.print(",");
  Serial.print(Kp, precision);
  Serial.print(",");
  Serial.print(Ki, precision);
  Serial.print(",");
  Serial.print(Kd, precision);
  Serial.print(",");
  Serial.print(pid, precision);
  Serial.print("\n");
}

void plotterOut(){
  int precision = 8;

  Serial.print(setPos, precision);
  Serial.print(",");
  Serial.print(currPos, precision);
  Serial.print("\n");
}
