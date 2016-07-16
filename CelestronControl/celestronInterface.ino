//Call to configure pins before beginning a command 
void beginCmd(){ //Reconfigure pins to prepare to transmit
  pinMode(EN_PIN, OUTPUT);
  pinMode(TX, OUTPUT);
  digitalWrite(EN_PIN,LOW);
  digitalWrite(TX, HIGH);
}


//Call to configure pins when ending a command
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
long celestronGetPos(char axis, bool dangerous){
  long verifyPos = celestronRoughGetPos(axis);

  if(dangerous) return verifyPos; //Fuck error checking
  
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

void celestronStopCmd(){
  celestronStopCmd(true);
}

void celestronStopCmd(bool shouldWait){ //Stop motion in both axes and wait 600 mSec for them to come to a halt
  celestronDriveMotor(LEFT,0);
  celestronDriveMotor(UP,0);
  if(shouldWait)delay(600);
}

void celestronGoToPos(long azmPos, long altPos){
  long currAzmPos = celestronGetPos(AZM,false);
  long currAltPos = celestronGetPos(ALT,false);
  long errorAzm = calcSmallestError(currAzmPos, azmPos);
  long errorAlt = calcSmallestError(currAltPos, altPos);

  int goodAlign = 0;
  digitalWrite(LED, LOW);
  
  while(goodAlign < 2){
    goodAlign = 0;
    currAzmPos = celestronGetPos(AZM,false);
    currAltPos = celestronGetPos(ALT,false);
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

  int bufIndex = 0;
  
  unsigned long startTime = micros();
  
  bool incomingMsg = 0;
  bool lastState = 0;
  
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
}
