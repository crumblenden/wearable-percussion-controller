// The size of the buffers used to calcualate differentials and velocity
const int diffBufferLength = 2;
const int velBufferLength = 10;//10
// The number of sensor inputs available
const int numSensors = 3;
const int numInputs = numSensors * 2; //(2 inputs per sensor, Y axis and Z axis)

//buffers for each input hold raw values, first differentials and second differentials
double inputBuffers[numInputs][diffBufferLength]; 
double diff1Buffers[numInputs][diffBufferLength];
double diff2Buffers[numInputs][diffBufferLength];

//buffers for each sensor used to approximate input velocity
double velBuffers[numSensors][velBufferLength];

//array holding the value of the summed velocity buffers for each sensor
double summedDiffs[numSensors];

//array holding the moduli of the first differentials for each sensor
double diff1Abs[numSensors];

//array holding the modului of the second differentials for each sensor
double diff2Abs[numSensors];

//array holding the trigger state for each sensor (1 or 0)
double triggers[numSensors];

//array holding the velocity value for each sensor
double velocities[numSensors];

//trigger threshold
double threshold = 80;//150

//time is recorded by millis() when a trigger occurs, used to prevent output after trigger
double timeOutTimes[numSensors];
//time in milliseconds for which triggers are supressed for after a previous trigger
double gateTime = 40;//40

//provides a velocity curve using pow(), e.g. 1.1 increases velocity range, 0.9 decreases it
double velCurveFactor = 1;

// time to wait at end of loop function, not currently used
double delayAmt = 0;//1

void setup() {
  for (int j = 0; j < numInputs; j++) {
    for (int i = 0; i < diffBufferLength; i++) {
      inputBuffers[j][i] = 0.0;
    }
  }
  for (int i = 0; i < numSensors; i++) {
    timeOutTimes[i] = 0;
    triggers[i] = 0;
    velocities[i] = 0;
  }
  Serial.begin(115200);
}

void loop() {

  for (int j = 0; j < numInputs; j++) { //for each input pin used (Y and Z for each sensor)
    int sample = 0;
    if (analogRead(j) >= 0 && analogRead(j) <= 1023) {
      sample = analogRead(j);
    }
    //add input samples to buffer
    addToBuffer(inputBuffers[j], sample, diffBufferLength);
    //calculate first differential, print to diff1 buffers
    differential(inputBuffers[j], diff1Buffers[j]);
    //calculate second differential, pront to diff2 buffers
    differential(diff1Buffers[j], diff2Buffers[j]);
  }
  for (int j = 0; j < numSensors; j++) { //for each sensor connected
    //sum Y and Z axis of second differentials
    diff2Abs[j] = ((diff2Buffers[(j * 2)][0] + diff2Buffers[(j * 2) + 1][0]));
    //sum modulus of Y and Z axis of first differentials
    diff1Abs[j] = abs(diff1Buffers[(j * 2)][1] + diff1Buffers[(j * 2) + 1][1] + diff2Abs[j]);
    //add diff1abs to the velocity buffer
    addToBuffer(velBuffers[j], diff1Abs[j], velBufferLength);
    //sum velocity bufer
    summedDiffs[j] = sumBuffer(velBuffers[j], velBufferLength);

    //determine if threshold is exceeded
    if (diff2Abs[j] > threshold) {
      //determine if a trigger occured since gateTime milliseconds ago
      if (millis() > (timeOutTimes[j] + gateTime)) {
        if (triggers[j] >= 1) { //cycle value between 0 and 1 when a trigger occurs
          triggers[j] = 0;
        } else {
          triggers[j] ++;
        }
        //calculate velocity, apply velocity curve
        velocities[j] = pow(summedDiffs[j], velCurveFactor) / velBufferLength;
        //record time of trigger
        timeOutTimes[j] = millis();
      }
    }
    //print data to serial for reception by MaxMSP
    Serial.print(triggers[j]);
    Serial.print(" ");
    Serial.print(velocities[j]);
    Serial.print(" ");
  }
  //end serial line
  Serial.println();
  //delay loop function execution for delayAmt milliseconds
  delay(delayAmt);
  //Serial.print(millis()); //used to determine loop function execution speed
}

//calculates the differential between two values stored in a buffer, outputs to a specified buffer
double differential(double bufferIn[], double bufferOut[]) {
  double diff = bufferIn[0] - bufferIn[1];
  addToBuffer(bufferOut, diff, diffBufferLength);
}

//adds a value to a specified buffer, shifting it's contents down
void addToBuffer(double bufferIn[], double sample, int bufferLength) {
  for (int i = bufferLength - 1; i > 0; i--) {
    bufferIn[i] = bufferIn[i - 1];
  }
  bufferIn[0] = sample;
}

//sums the contents of a buffer
double sumBuffer(double bufferIn[], int bufferLength) {
  double total;
  for (int i = 0; i < bufferLength; i++) {
    total += bufferIn[i];
  }
  return total;
}

