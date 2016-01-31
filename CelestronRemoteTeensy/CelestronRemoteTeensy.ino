//16086062 101926

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

#define BUFLEN 64
#define CHARBUFLEN 10

signed char msgBuf[BUFLEN];
unsigned char charBuf[CHARBUFLEN];

void setup()
{
  Serial.begin(250000);
  Serial.println("Serial Initialized!");

  pinMode(EN_PIN, OUTPUT);
  pinMode(TX, OUTPUT);
  pinMode(RX, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  
  digitalWrite(EN_PIN, HIGH);
  digitalWrite(TX, HIGH);

  randomSeed(analogRead(0));

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

  //celestronGoToPos(azmTarget,altTarget); //AZM, ALT

  /*celestronDriveMotor(UP,9);
  delay(2000);
  celestronStopCmd();*/
}

int globalSpeed = 9;
bool vomitData = false;

void loop() // run over and over
{
  if(Serial.available() > 0){
    byte incomingByte = Serial.read();
    if(incomingByte == 'Q'){
      Serial.print(celestronGetPos(AZM));
      Serial.print(' ');
      Serial.print(celestronGetPos(ALT));
      Serial.print(' ');
      Serial.println(analogRead(A0));
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
    if(incomingByte == 'S') Serial.println(analogRead(A0));
    if(incomingByte == 'G') celestronGoToPos(Serial.parseInt(),Serial.parseInt());
    if(incomingByte == 'V') vomitData = !vomitData;
  }
  if(vomitData){
      Serial.print(celestronGetPos(AZM));
      Serial.print(' ');
      Serial.print(celestronGetPos(ALT));
      Serial.print(' ');
      Serial.println(analogRead(A0));
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
      }else if(errorAzm < -1000){
        celestronDriveMotor(LEFT, 4);
      }else if(errorAzm > 1000){
        celestronDriveMotor(RIGHT, 4);
      }else if(errorAzm < -10){
        celestronDriveMotor(LEFT, 3);
      }else if(errorAzm > 10){
        celestronDriveMotor(RIGHT, 3);
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
      }else if(errorAlt < -1000){
        celestronDriveMotor(UP, 4);
      }else if(errorAlt > 1000){
        celestronDriveMotor(DOWN, 4);
      }else if(errorAlt < -10){
        celestronDriveMotor(UP, 3);
      }else if(errorAlt > 10){
        celestronDriveMotor(DOWN, 3);
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
  digitalWrite(LED, HIGH);
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
        if(lastState) msgBuf[bufIndex] ++;
        else msgBuf[bufIndex] --;

        if((lastState && msgBuf[bufIndex] >= 100) || (msgBuf[bufIndex] <= -100)) bufIndex++;
      }
    }

    delayMicroseconds(11);  //Teensy: 11uSec is pretty much bae. Don't touch
                            //Arduino: 7 uSec with LEDs, 11 uSec without works pretty well    
  }

  /*for(int i = 0; i< BUFLEN; i++){
    Serial.print(msgBuf[i]/4, DEC);
    Serial.print('\t');
  }
  Serial.println();*/
  
  //digitalWrite(LED, LOW);
  
//This next part is stupidly complicated and I can't quite explain how it works
//It takes msgBuf and tries to parse out a series of times spent high/low into a string of characters

  int N = -1;
  int c = 0;
  bool isActive = 0;
  unsigned char workingChar = 0;

  //iterate over msgBuf
  for(int i = 0; i < BUFLEN; i++){
    msgBuf[i] = msgBuf[i]/4; //two or three samples are taken for each bit low or high; this condenses them

    
    if(msgBuf[i] < 0){
      for(int j = msgBuf[i]; j < 0; j++){
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
    }else if(msgBuf[i] > 0){
      for(int j = 0; j < msgBuf[i]; j++){

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
    msgBuf[i]=0; //Clear buffer
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

