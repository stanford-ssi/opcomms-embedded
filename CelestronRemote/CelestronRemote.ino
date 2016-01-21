//Code to allow microcontroller to control Celestron arm

/*******************************************************************************/

#define RX 2 //Data coming to board from Celestron
#define TX 3 //Data coming from Celestron to board. 

/******** IMPORTANT ********/
//TX and RX are electrically connected within the Celestron arm
//When one is in use, the other pin needs to be put into high-impedance (usually input mode does this) 

#define EN_PIN 4 //Additional line that both Celestron and the board pull down when they are sending data over the TX/RX line
#define LED 13 //LED pin

#define BITTIME 45 //Time to transmit a bit, in microseconds. 
                   //This should really be closer to 50 uSec, but the transmission execution time on a 16MHz Arduino chews up ~5 uSec

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
#define UP NEG + (2*ALT) //70 - Up is a negative altitude movement
#define DOWN POS + (2*ALT) //71 - Down is a positive azimuth movement

/*******************************************************************************/

#define BUFLEN 64
#define CHARBUFLEN 10

char msgBuf[BUFLEN];
unsigned char charBuf[CHARBUFLEN];

void setup()
{
  Serial.begin(9600);
  Serial.println("potato!");
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(TX, OUTPUT);
  pinMode(RX, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  
  digitalWrite(EN_PIN, HIGH);
  digitalWrite(TX, HIGH);
  digitalWrite(LED, LOW);
  delay(1000);

/*******************************************************************************/
//Initialization code above this line is required for proper startup
//Code below is optional

  celestronMoveCmd(UP,9);
  delay(2000);
  celestronStopCmd();
}

void loop() // run over and over
{
  celestronMoveCmd(RIGHT,9);
  delay(1000);
  celestronStopCmd();
  celestronPosCmd(AZM);
  celestronPosCmd(AZM);
  celestronPosCmd(AZM);
  celestronPosCmd(AZM);
  celestronPosCmd(AZM);
  delay(3000);
}

void beginCmd(){ //Reconfigure pins to prepare to transmit
  pinMode(EN_PIN, OUTPUT);
  pinMode(TX, OUTPUT);
  digitalWrite(EN_PIN,LOW);
  digitalWrite(TX, HIGH);
}

void endCmd(){
  pinMode(EN_PIN, INPUT_PULLUP); //Hi-Z pin, but pull up - EN is normally high and only pulled low when receiving (Celestron pulls low) or transmitting
  pinMode(TX, INPUT_PULLUP); //Hi-Z pin, but pull up - Same as above
}

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

void celestronMoveCmd(char dir, int spd){
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

void celestronPosCmd(char axis){
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

  delayWhileReading(20); //Spend 20 milliseconds paused while reading back the queried position
}

void celestronStopCmd(){ //Stop motion in both axes and wait 500 mSec for them to come to a halt
  celestronMoveCmd(LEFT,0);
  celestronMoveCmd(UP,0);
  delay(500);
}

void delayWhileReading(long msToDelay){
  int bufIndex = 0;
  
  digitalWrite(LED, HIGH);
  unsigned long startTime = micros();
  
  bool incomingMsg = 0;
  bool lastState = 0;
  
  while(micros() - startTime < msToDelay * 1000){ //While time elapsed since start of function is less than prescribed time
    
    if(digitalRead(EN_PIN) == LOW){ //Look to see if EN_PIN is low, as that means a message is incoming
      digitalWrite(LED, LOW);
      incomingMsg = 1;
    }else{
      digitalWrite(LED, HIGH);
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

    delayMicroseconds(7); //7 uSec with LEDs, 11 uSec without works pretty well    
  }
  digitalWrite(LED, LOW);


//This next part is stupidly complicated and I can't quite explain how it works in the time I have right now
//It takes msgBuf and tries to parse out a series of times spent high/low into a string of characters

  int N = -1;
  int c = 0;
  bool isActive = 0;
  unsigned char workingChar = 0;

  for(int i = 0; i < BUFLEN; i++){
    msgBuf[i] = msgBuf[i]/2;

    
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
        //Serial.print('1');

        if(isActive){
          N++;
          workingChar = (workingChar >> 1) | 0b10000000;
          if(N == 8){
            //Serial.println();
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
  
  Serial.println();
  delay(5);
}

void printAndClearCharBuf(){
  ///// Print and clear charBuf
  
  for(int i = 0; i < CHARBUFLEN; i++){
    Serial.print("erwrethr");
    //Serial.print(charBuf[i], DEC);
    Serial.print('\t');
    charBuf[i] = 0; //Clear buffer
  }

  /////
}

