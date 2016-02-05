#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*****************************************************************
 * 
 *  COMMAND DOCUMENTATION
 *  
 *  <value> means that value is sent, with no spaces or < > characters
 *  
 *  Q - Returns Azimuth Position <space> Altitude Position <space> Voltage on Analog 0 (connected to sensor)
 *  S - Returns only voltage on Analog 0 (connected to sensor)
 *  V - Toggles persistent output (continuously sends Q command, reporting position and sensor voltage)
 *  I - Toggles IMU readout
 *  
 *  Any single digit 0-9 - Sets default movement speed to that value (9 is fast, 4 is slow, 3 and below do not move)
 *  L - Azimuth motor turns left at default speed (and persists)
 *  R - Azimuth motor turns right at default speed (and persists)
 *  U - Altitude motor turns up at default speed (and persists)
 *  D - Altitude motor turns down at default speed (and persists)
 *  X - Both motors stop (creates 600ms delay to acheive full stop)
 *  G<azmPos>,<altPos> - Drives arm to specified position. HANGS PROGRAM UNTIL POSITION IS REACHED
 *  
 *  ~ - Turns on/off beam hold, keeping laser on persistently
 *  ! - Turns on 1 Hz laser pulse
 *  B - Blinks LED on board. HANGS PROGRAM FOR 200 ms
 *  
 *  P - Allows for in-test redefinition of PPM parameters (use is discouraged)
 *  ><message> - Transmits message over PPM
 *  < - Waits to receive message over PPM. HANGS PROGRAM UNTIL MESSAGE IS RECEIVED OR 10 SECONDS ELAPSE (whichever comes first. Timeout currently untested)
 *  W - Persistently waits to receive message over PPM. HANGS PROGRAM UNITL 10 SECONDS ELAPSE WITHOUT A MESSAGE (Timeout currently untested)
 *  
 *****************************************************************/

#define LASER 5
#define N_BITS 2
#define SENSOR_PIN A0
#define MSG_BUF_LEN 1024

//These are not explicitly constant because they should be field configurable
int highTime = 100; //us
int lowTime = 100; //us
int sampleTime = 10; //Should be ~10us less than a factor of lowTime
int sampleTimeShiftVal = 2; //Rightshifting is much cheaper than dividing; 2^this is how many samples per interval
int sensorThreshold = 8;

#define WAIT_FOR_MSG_TIMEOUT 500000 //us

char msgBuf[MSG_BUF_LEN];


//Code to allow microcontroller to control Celestron arm

/*******************************************************************************/

#define RX 2 //Data coming to board from Celestron
#define TX 3 //Data coming from Celestron to board. 

/******** IMPORTANT ********/
//TX and RX are electrically connected within the Celestron arm
//When one is in use, the other pin needs to be put into high-impedance (usually input mode does this) 

#define EN_PIN 4 //Additional line that both Celestron and the board pull down when they are sending data over the TX/RX line
#define LED 13 //LED pin

#define BITTIME 50 //Time to transmit a bit, in microseconds, Teensy version 
//#define BITTIME 45 //Time to transmit a bit, in microseconds, Arduino version 
                   //This should really be closer to 50 uSec, but the transmission execution time on a 16MHz Arduino chews up ~5 uSec

#define POSMAX 16777216 //2^24; all angles are represented as 24 bit integers, and dividing by POSMAX converts to a fraction of a complete rotation
#define POS_TOLERANCE 5000 //Max value two position values are allowed to deviate from each other while the system believes that they followed each other

/******** IMPORTANT ********/
//If Serial seems to be breaking, adjust BITTIIME. The communication expects to run at 19200 baud, and depending on your microcontroller
//the time required to initiate a transition may eat into the ~50 uSec needed for each pulse

/*******************************************************************************/

//Axis Command Constants
//These define several concepts used in sending commands to and from the Celestron's altitude and azimuth axes

#define POS '%' //37 - rotationally, defined as counterclockwise
#define NEG '$' //36 - rotationally, defined as clockwise
#define AZM 16
#define ALT 17
#define RIGHT NEG + (2*AZM) //68 - Right is a negative azimuth movement
#define LEFT POS + (2*AZM) //69 - Left is a positive azimuth movement
#define DOWN NEG + (2*ALT) //70 - Down is a negative altitude movement, because logic is overrated
#define UP POS + (2*ALT) //71 - Up is a positive azimuth movement

/*******************************************************************************/

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

/*******************************************************************************/

#define BUFLEN 64
#define CHARBUFLEN 10

signed char posBuf[BUFLEN];
unsigned char charBuf[CHARBUFLEN];
bool waitMode = false;
bool beamHold = false;
bool blinkMode = false;
bool bnoEnabled = true; //True by default; if fails to enable, will be set to false. Initialize to false to completely disable
bool bnoVerbose = false; //By default, do not print out position data when queried

void setup()
{
  analogReference(INTERNAL);
  Serial.begin(250000);

  pinMode(EN_PIN, OUTPUT);
  pinMode(TX, OUTPUT);
  pinMode(RX, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(LASER, OUTPUT);
  
  digitalWrite(EN_PIN, HIGH);
  digitalWrite(TX, HIGH);

  randomSeed(analogRead(A2));

  /* Initialise the orientation sensor */
  if(!bnoEnabled || !bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    bnoEnabled = false;
  }

/*******************************************************************************/
//Initialization code above this line is required for proper startup
//Code below is optional

//Startup LED blink

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED,LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED,LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED,LOW);
  delay(200);

  long azmTarget = 1000000;
  long altTarget = 1000000;
}

int globalSpeed = 9;
bool vomitData = false;
bool blinkState = 0;

long currentAzm = -1;
long currentAlt = -1;

void loop() // run over and over
{
  if(Serial.available() > 0){
    byte incomingByte = Serial.read();
    if(incomingByte == 'Q'){
      query();
    }
    if(incomingByte - '0' <= 9 && incomingByte - '0' >= 1) globalSpeed = incomingByte - '0';
    if(incomingByte == 'L') celestronDriveMotor(LEFT, globalSpeed);
    if(incomingByte == 'U') celestronDriveMotor(UP, globalSpeed);
    if(incomingByte == 'D') celestronDriveMotor(DOWN, globalSpeed);
    if(incomingByte == 'R') celestronDriveMotor(RIGHT, globalSpeed);
    if(incomingByte == 'X'){
      celestronDriveMotor(UP, 0);
      celestronDriveMotor(LEFT, 0);
    }
    if(incomingByte == 'S') Serial.println(analogRead(SENSOR_PIN));
    if(incomingByte == 'G') celestronGoToPos(Serial.parseInt(),Serial.parseInt());
    if(incomingByte == 'V') vomitData = !vomitData;
    if(incomingByte == 'I') bnoVerbose = !bnoVerbose;
    
    if(incomingByte == '~'){
      beamHold = !beamHold;
      blinkMode = false;
    }
    if(incomingByte == '!'){
      beamHold = false;
      blinkMode = !blinkMode;
    }

    if(incomingByte == 'B'){
      blinkLED();
    }
    
    if(incomingByte == 'P') defineParameters();
    if(incomingByte == '>'){
      int msgLen = Serial.available(); //Counts number of bytes to be sent
      Serial.readBytes(msgBuf, msgLen);
      blink_Packet(msgBuf, msgLen);
      clearMsgBuf();
    }
    if(incomingByte == 'W') waitMode = !waitMode;
    
    if(incomingByte == '<'){
      Serial.println("Waiting for msg:");
      while(Serial.available()) Serial.read(); //For no obvious reason, not clearing the Serial buffer prior to listening causes weird timing bugs
      int charsRead = listen_for_msg();
      
      if(charsRead != -1){
        Serial.print(charsRead);
        Serial.print(msgBuf);
        clearMsgBuf();
      }else{
        Serial.print(0);
      }
    }
  }
  digitalWrite(LASER,beamHold ? HIGH : LOW);
  
  if(blinkMode){
    digitalWrite(LASER, blinkState ? HIGH : LOW);
    blinkState = !blinkState;
    delay(500);
  }

  if(waitMode){
    while(Serial.available()) Serial.read(); //For no obvious reason, not clearing the Serial buffer prior to listening causes weird timing bugs
    int charsRead = listen_for_msg();
    
    if(charsRead != -1){
      Serial.print(charsRead);
      Serial.print(msgBuf);
      clearMsgBuf();
    }else{
        Serial.print(0);
    }
  }

  
  if(vomitData){
      query();
   }
}

//Call before beginning a command to configure pins
void beginCmd(){ //Reconfigure pins to prepare to transmit
  pinMode(EN_PIN, OUTPUT);
  pinMode(TX, OUTPUT);
  digitalWrite(EN_PIN,LOW);
  digitalWrite(TX, HIGH);
}


//Call when ending a command to configure pins
void endCmd(){
  pinMode(EN_PIN, INPUT_PULLUP); //Hi-Z pin, but pull up - EN is normally high and only pulled low when receiving (Celestron pulls low) or transmitting
  pinMode(TX, INPUT_PULLUP); //Hi-Z pin, but pull up - Same as above
}


//Send a character to Celestron
void celestronWrite(char c){

  //To begin sending a character, send one low bit
  
  digitalWrite(TX, LOW);
  delayMicroseconds(BITTIME);

  //Send each bit in the character one at a time, starting with the LSB
  
  for(int i = 0; i < 8; i++){
    digitalWrite(TX,c & 0b1); //Mask to set pin to value of LSB
    c = c >> 1; //Move to next LSB
    delayMicroseconds(BITTIME);
  }

  //Send a high bit to end the character
  
  digitalWrite(TX, HIGH);
  delayMicroseconds(BITTIME);

  //Character looks like this: 0XXXXXXXX1
}


//Send command to drive motors in specified direction at specified speed
//9 is relatively fast (2.78 +/- 0.01 degrees/sec), while 1 is unfathomably slow
void celestronDriveMotor(char dir, int spd){
  char axis = (dir - NEG) / 2; //convert direction to axis variable
  char realdir = dir - (2 *axis); //convert "direction" constant to actual value needed by Celestron

  unsigned char lastChar = 239 - (realdir + axis + spd); //Weird checksum character; command won't be accepted without it

//This is the list of characters used in a movement command - please don't touch

  beginCmd();
  celestronWrite(';');
  celestronWrite(4);
  celestronWrite('\r');
  celestronWrite(axis);
  celestronWrite(realdir);
  celestronWrite(spd);
  celestronWrite(lastChar);
  endCmd();
  
}


//Command to get angular position on the specified axis
long celestronGetPos(char axis){
  long verifyPos = celestronRoughGetPos(axis);
  long tentativePos = verifyPos + POS_TOLERANCE + 1; //Make value guaranteed to fail test
  
  while(abs(verifyPos-tentativePos) > POS_TOLERANCE){ //If positions are not within tolerance, keep polling
    tentativePos = verifyPos;
    verifyPos = celestronRoughGetPos(axis);
  }
  
  return(verifyPos);
}

long celestronRoughGetPos(char axis){
  unsigned char lastChar = 239 - axis; //Weird checksum character; command won't be accepted without it

//This is the list of characters to trigger a position query on the axis specified by "axis" - please don't touch
  
  beginCmd();
  celestronWrite(';');
  celestronWrite(3);
  celestronWrite('\r');
  celestronWrite(axis);
  celestronWrite(1);
  celestronWrite(lastChar);
  endCmd();

  return celestronListenForResponse(20); //Spend 20 milliseconds paused while reading back the queried position
}

void celestronStopCmd(){ //Stop motion in both axes and wait 600 mSec for them to come to a halt
  celestronDriveMotor(LEFT,0);
  celestronDriveMotor(UP,0);
  delay(600);
}

void celestronGoToPos(long azmPos, long altPos){
  long currAzmPos = celestronGetPos(AZM);
  long currAltPos = celestronGetPos(ALT);
  long errorAzm = calcSmallestError(currAzmPos, azmPos);
  long errorAlt = calcSmallestError(currAltPos, altPos);

  int goodAlign = 0;
  digitalWrite(LED, LOW);
  
  while(goodAlign < 2){
    goodAlign = 0;
    currAzmPos = celestronGetPos(AZM);
    currAltPos = celestronGetPos(ALT);
    long lastErrorAzm = errorAzm;
    long lastErrorAlt = errorAlt;
    errorAzm = calcSmallestError(currAzmPos, azmPos);
    errorAlt = calcSmallestError(currAltPos, altPos);
    Serial.print('@');
    Serial.print(currAzmPos);
    Serial.print(',');
    Serial.println(currAltPos);
    Serial.print("Delta Azm/Alt: ");
    Serial.print(errorAzm);
    Serial.print('\t');
    Serial.println(errorAlt);

    if(errorAzm == 0 || abs(errorAzm-lastErrorAzm) < (0.25 * float(abs(lastErrorAzm)))){
      if(errorAzm < -50000){
        celestronDriveMotor(LEFT, 9);
      }else if(errorAzm > 50000){
        celestronDriveMotor(RIGHT, 9);
      }else if(errorAzm < -100){
        celestronDriveMotor(LEFT, 4);
      }else if(errorAzm > 100){
        celestronDriveMotor(RIGHT, 4);
      }else{
        celestronDriveMotor(RIGHT, 0);
        goodAlign++;
      }
    }else{
      Serial.print("AZM Transient ");
      Serial.print(errorAzm);
      Serial.print(' ');
      Serial.print(lastErrorAzm);
      Serial.print(' ');
      Serial.println(0.2 * float(lastErrorAzm));
      celestronDriveMotor(RIGHT, 0);
    }

    if(errorAlt == 0 || abs(errorAlt-lastErrorAlt) < (0.25 * float(abs(lastErrorAlt)))){
      if(errorAlt < -50000){
        celestronDriveMotor(UP, 9);
      }else if(errorAlt > 50000){
        celestronDriveMotor(DOWN, 9);
      }else if(errorAlt < -100){
        celestronDriveMotor(UP, 4);
      }else if(errorAlt > 100){
        celestronDriveMotor(DOWN, 4);
      }else{
        celestronDriveMotor(DOWN, 0);
        goodAlign++;
      }
    }else{
      Serial.print("ALT Transient ");
      Serial.print(errorAlt);
      Serial.print(' ');
      Serial.print(lastErrorAlt);
      Serial.print(' ');
      Serial.println(0.2 * float(lastErrorAlt));
      celestronDriveMotor(DOWN, 0);
    }
    
    Serial.println();
    delay(25);
  }
  blinkLED();
  Serial.println("Aligned!");
}

//Figures out whether going in the normal direction or rolling over is shorter
long calcSmallestError(long currentPos, long targetPos){

  if(currentPos == targetPos) return 0; //DON'T MOVE
  
  long upDistance = (currentPos < targetPos) ? targetPos - currentPos : POSMAX - (currentPos-targetPos); //If target is greater than current, going up is easy; otherwise you have to wrap around
  long downDistance = (currentPos > targetPos) ? targetPos - currentPos : (targetPos-currentPos) - POSMAX; //If target is lesser than current, going down is easy; otherwise you have to wrap around
  
  /*Serial.print(upDistance);
  Serial.print('\t');
  Serial.print(downDistance);
  Serial.print('\t');
  Serial.println(upDistance < abs(downDistance));*/
  
  return (upDistance < abs(downDistance)) ? upDistance : downDistance;
}

long celestronListenForResponse(long msToDelay){
  /*for(int i = 0; i<10000; i++){
    Serial.print(digitalRead(EN_PIN));
    Serial.print(" x");
    Serial.print(digitalRead(RX));
    Serial.print(' ');
    delayMicroseconds(1);
  }*/
  
  int bufIndex = 0;
  
  unsigned long startTime = micros();
  
  bool incomingMsg = 0;
  bool lastState = 0;

  /*while(micros() - startTime < msToDelay * 1000){
    Serial.print(digitalRead(EN_PIN));
    Serial.print(' ');
    Serial.print(digitalRead(RX));
    Serial.print('\t');
    delayMicroseconds(5);
  }
  Serial.println();*/
  
  while(micros() - startTime < msToDelay * 1000){ //While time elapsed since start of function is less than prescribed time
    
    if(digitalRead(EN_PIN) == LOW){ //Look to see if EN_PIN is low, as that means a message is incoming
      //digitalWrite(LED, LOW);
      incomingMsg = 1;
    }else{
      //digitalWrite(LED, HIGH);
      incomingMsg = 0;
    }

    if(incomingMsg){
      if(digitalRead(RX) != lastState){
        lastState = !lastState;
        bufIndex++;
      }

      if(bufIndex < BUFLEN){
        if(lastState) posBuf[bufIndex] ++;
        else posBuf[bufIndex] --;

        if((lastState && posBuf[bufIndex] >= 100) || (posBuf[bufIndex] <= -100)) bufIndex++;
      }
    }

    delayMicroseconds(11);  //Teensy: 11uSec is pretty much bae. Don't touch
                            //Arduino: 7 uSec with LEDs, 11 uSec without works pretty well    
  }

  /*for(int i = 0; i< BUFLEN; i++){
    Serial.print(posBuf[i]/4, DEC);
    Serial.print('\t');
  }
  Serial.println();*/
  
  //digitalWrite(LED, LOW);
  
//This next part is stupidly complicated and I can't quite explain how it works
//It takes posBuf and tries to parse out a series of times spent high/low into a string of characters

  int N = -1;
  int c = 0;
  bool isActive = 0;
  unsigned char workingChar = 0;

  //iterate over posBuf
  for(int i = 0; i < BUFLEN; i++){
    posBuf[i] = posBuf[i]/4; //two or three samples are taken for each bit low or high; this condenses them

    
    if(posBuf[i] < 0){
      for(int j = posBuf[i]; j < 0; j++){
        if(!isActive){
          isActive = 1;
        }

        N++;
        workingChar = workingChar >> 1;
        if(N == 8){ //Done building a character
          N = -1;
          charBuf[c] = workingChar;
          c++;
          isActive = 0;
        }
      }
    }else if(posBuf[i] > 0){
      for(int j = 0; j < posBuf[i]; j++){

        if(isActive){
          N++;
          workingChar = (workingChar >> 1) | 0b10000000;
          if(N == 8){
            N = -1;
            charBuf[c] = workingChar;
            c++;
            isActive = 0;
          }
        }
      }
    }
  }
  
  for(int i = 0; i < BUFLEN; i++){  
    posBuf[i]=0; //Clear buffer
  }

  long fullPosition = (long(charBuf[5]) << 16) | (long(charBuf[6]) << 8) | charBuf[7];
  //Serial.println(fullPosition);
  
  printAndClearCharBuf(false);
  //Serial.println();
  delay(5);
  
  return fullPosition;
}

void printAndClearCharBuf(bool shouldPrint){
  ///// Print and clear charBuf
  
  for(int i = 0; i < CHARBUFLEN; i++){
    if(shouldPrint){
      Serial.print(charBuf[i],DEC);
      Serial.print('\t');
    }
    charBuf[i] = 0; //Clear buffer
  }

  /////
}

void defineParameters(){
  Serial.println("Input pulse time (us; blank for 100): ");
  while(!Serial.available());
  int desiredPulseTime = Serial.parseInt();
  Serial.println("Input sample interval (us; blank for 10): ");
  while(!Serial.available());
  int desiredSampleTime = Serial.parseInt();
  Serial.println("Input threshold (0-1023; blank for 5): ");
  while(!Serial.available());
  int desiredThreshold = Serial.parseInt();
  
  Serial.print("Pulse Time: ");
  Serial.print(desiredPulseTime);
  highTime = desiredPulseTime;
  lowTime = desiredPulseTime;
  if(desiredPulseTime <= 0){
    Serial.print(" - Ignored");
    highTime = 100;
    lowTime = 100;
  }
  Serial.print(" --> ");
  Serial.println(highTime);

  Serial.print("Input Sample Interval: ");
  Serial.print(desiredSampleTime);
  sampleTime = desiredSampleTime;
  if(desiredSampleTime <= 0){
    Serial.print(" - Ignored");
    sampleTime = 10;
  }
  Serial.print(" --> ");
  Serial.println(sampleTime);

  Serial.print("Input Threshold: ");
  Serial.print(desiredThreshold);
  sensorThreshold = desiredThreshold;
  if(desiredThreshold <= 0){
    Serial.print(" - Ignored");
    sensorThreshold = 5;
  }
  Serial.print(" --> ");
  Serial.println(sensorThreshold);
  Serial.println("---------------------------------------");
  Serial.println();
}

bool checkSensor(int pin){
  return (analogRead(pin) > sensorThreshold);
}

/*Function: listen_for_msg:
    Postconditions:
        populates MSG_BUF_LEN with received bytes
*/

int listen_for_msg(){
  int charsRead = 0;
  bool stopChar = 0;
  char charBeingRead;
  int samplesCounted;


  elapsedMillis waiting;
  digitalWrite(LED, HIGH);
  //while(checkSensor(SENSOR_PIN)==0){} //Note: this hangs the board
  while(checkSensor(SENSOR_PIN)==0 && waiting < 10000){} //Note: this hangs the board until a message arrives or 10 seconds elapse, whatever comes first
  digitalWrite(LED, LOW);

  if(waiting >= 10000){
    waitMode = false;
    return -1; //Time out if time without message exceeds 10s
  }
  
  while(charsRead < MSG_BUF_LEN && stopChar == 0){

    //The 4 here needs to be changed to accept a different PPM value and I couldn't think of a smart way of doing it in advance
    for(int i = 0; i < 4; i++){
      delayMicroseconds(highTime); //Let start pulse pass
      samplesCounted = 0;

      while(checkSensor(SENSOR_PIN)==1){
        samplesCounted++;
      }
      if(samplesCounted > 20){
        stopChar = 1; //detect end of packet
        //Serial.println("EOP");
      }

      if(!stopChar){
        samplesCounted = 0;
        while(samplesCounted < 100 && checkSensor(SENSOR_PIN)==0){
          samplesCounted++;
          delayMicroseconds(sampleTime);
        }

        if(samplesCounted >= 100) stopChar = 1; //Detect failure and terminate packet
  
        /*Serial.print(micros());
        Serial.print(' ');*/
        //Serial.println(samplesCounted);
         
        samplesCounted = samplesCountedToNibblet(samplesCounted);
        charBeingRead = (charBeingRead << N_BITS) | samplesCounted; //Moves existing bits to the left, inserts new bits on right
      }
    }
    if(!stopChar){
      msgBuf[charsRead] = charBeingRead;
      //Serial.println(charBeingRead);
      charsRead++;
    }
  }

  return charsRead;
}

/*Function: samplesCountedToNibblet
 * 
 * Postconditions:
 *      returns the actual bits of data represented by samplesCounted
 * 
 * Method of Operation:
 *      samplesCounted should equal the number of low intervals observed,
 *      leftshifted by sampleTimeShiftVal. This function rightshifts to
 *      correct for that and then subtracts 1 to make 1 interval correspond
 *      to 0, 2 intervals to 1, etc. 
 *      
 * This function is discrete only because this comment block would have
 * to be stupidly long in an already long function
 */
 
int samplesCountedToNibblet(int samplesCounted){
  if(samplesCounted == 3) samplesCounted++; //SUPER HACKY - really short lows will occasionally be dropped, so this helps correct for that
  return(samplesCounted >> sampleTimeShiftVal) - 1;
}

/*Function: clearMsgBuf:
    Postconditions:
        msgBuf contains all zeroes
*/

void clearMsgBuf(){
  for(int i = 0; i < MSG_BUF_LEN; i++){
    msgBuf[i] = 0;
  }
}

/*Function: blink_packet:
    Preconditions: 
        len is index of last filled character in buffer
    Postconditions:
        transmits the packet buffer
*/
void blink_Packet(char* buffer, int len)
{   
    //Encodes and transmits each character
    for (int i=0; i < len; i++) {
        blink_char(buffer[i]);
    }
    
    //Signifies end of packet
    digitalWrite(LASER,HIGH);
    delayMicroseconds(lowTime*((0b1 << N_BITS)+1));
    digitalWrite(LASER,LOW);
}


/*Function: blink_char:
    Postconditions:
        transmits the char c via DPPM 4
*/
void blink_char(char c) {
    Serial.println(c);
    for (int i=3; i>=0; i--) {
        blink((c & (0b11 << i*2)) >> i*2); //Send MSB first
    } 
}


/*Function: blink_packet:
    Preconditions: 
        data < DPPM used
    Postconditions:
        pulses the light to transmit data
*/
void blink(int data) {
    //Serial.println(data);
    //Time on = PULSE_LENGTH
    digitalWrite(LASER,HIGH);
    delayMicroseconds(highTime);
    
    //Time off = PULSE_LENGTH*(data value)
    //  For example, a 01 transmitted would have a difference between pulses of 2 PULSE_LENGTH
    digitalWrite(LASER,LOW);
    delayMicroseconds(lowTime*(data+1));
    digitalWrite(LASER,HIGH);
} 

char checksum(char* buffer, int len)
{
    char sum = 0;
    for(int i = 0; i < len; i++) sum ^= buffer[i];
    return sum;
}

void blinkLED(){
  blinkLED(200);
}

void blinkLED(int onTimeIn_ms){
  digitalWrite(LED, HIGH);
  delay(onTimeIn_ms);
  digitalWrite(LED, LOW);
}

void query(){
  Serial.print(celestronGetPos(AZM));
  Serial.print(' ');
  Serial.print(celestronGetPos(ALT));
  Serial.print(' ');
  Serial.print(analogRead(SENSOR_PIN));
  Serial.print(' ');
  if(bnoEnabled && bnoVerbose) queryBNO();
  Serial.println();
}

void queryBNO(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t");

  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  /* Display the floating point data */
  Serial.print(" Gravity X: ");
  Serial.print(grav.x());
  Serial.print(" Gravity Y: ");
  Serial.print(grav.y());
  Serial.print(" Gravity Z: ");
  Serial.print(grav.z());
  Serial.print("\t");

  imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  /* Display the floating point data */
  Serial.print(" Magnetic X: ");
  Serial.print(magnet.x());
  Serial.print(" Magnetic Y: ");
  Serial.print(magnet.y());
  Serial.print(" Magnetic Z: ");
  Serial.print(magnet.z());
  Serial.print("\t\t");

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
}


