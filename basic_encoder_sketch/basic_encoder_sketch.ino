
const int encoderPin = 4;


/**
   initialize arduino for operation
*/
void setup() {
  Serial.begin(9600);   //start serial communication for console
}


/**
 * Main loop that controls the motor
 */
void loop() {
  Serial.println(readAnalogAngle(encoderPin, 1));

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
  return voltage/maxVoltage * 360;
}
