
//Constants
const int setPositionPot = 3;
const int currPositionPot = 4;
const int Ppot = 0;
const int Ipot = 1;
const int Dpot = 2;

const int invertSetPositionSwitch = 2;
const int enableMotorSwitch = 3;

//calculation variables
int setPosition, currPosition, pIn, iIn, dIn;   //inputs
long error, lastError, pid, now;
long p, i, d;       //values for p, i, and d factors
long Kp, Ki, Kd;    //coefficents 



/**
   initialize arduino for operation
*/
void setup() {
  Serial.begin(9600);   //start serial communication for console
  
  //input setup
  pinMode(invertSetPositionSwitch, INPUT);
  pinMode(enableMotorSwitch, INPUT);

  //motor setup
  

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
  setPosition = analogRead(setPositionPot) - 512;
  currPosition = analogRead(currPositionPot) - 512;

  //invert setPosition if needed
  if(digitalRead(invertSetPositionSwitch) == HIGH){
    setPosition = setPosition * -1;
  }


  //begin PID loop calculations

  //Create and Scale the coefficients 
  //from the input values (trial and error for scaling)
  Kp = pIn / 4;      // 0 to 255
  Ki = iIn / 4;     // 0 to 255
  Kd = dIn ;      // 0 to 1023
  
  error =  - setPosition;    //error is differene between where we want to be and where we are

  p = error;            //proportional is equal to the amount of error

  i = i + error         //integreal accumulates the error each iteration
  
  //Optional: create a dead band so the so integrel won't hunt back and fourth
  //if(abs(error) >  1) i = i + error;  // Integrate error if error > 1
  //if(error == 0) i = 0;               //Clear intergal if zero error

  i = constrain(i, -2000, 2000);      //stop i from increasing without bound

  d = error - lastError;              //rate of change of the error
  lastError = error;                  //save this error for the next loop  TODO: first loop lastError would be null

  //sum the PID factors with their coeffients
  pid = (Kp * p) + (kI * i) + (kD * d);

  

  //only move motor if enabled switch is on
  if(digitalRead(enableMotorSwitch) == HIGH){

    
  }  

}


//print out all important values
void debug(){
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
}
