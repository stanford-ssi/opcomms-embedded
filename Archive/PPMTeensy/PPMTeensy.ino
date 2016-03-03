#define LASER 5
#define LED 13
#define N_BITS 2
#define SENSOR_PIN A1
#define MSG_BUF_LEN 1024

//These are not explicitly constant because they should be field configurable
int highTime = 100; //us
int lowTime = 100; //us
int sampleTime = 10; //Should be ~10us less than a factor of lowTime
int sampleTimeShiftVal = 2; //Rightshifting is much cheaper than dividing; 2^this is how many samples per interval
int sensorThreshold = 8;

#define WAIT_FOR_MSG_TIMEOUT 500000 //us

char msgBuf[MSG_BUF_LEN];

void setup() {
  pinMode(LASER, OUTPUT);
  pinMode(LED, OUTPUT);
  Serial.begin(250000);
  delay(1000);
  defineParameters();
}

bool waitMode = false;

void loop() {

  //listen_for_msg();

  
  if(Serial.available()){
    byte incomingByte = Serial.read();
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
      listen_for_msg();
    }
  }

  if(waitMode){
    while(Serial.available()) Serial.read(); //For no obvious reason, not clearing the Serial buffer prior to listening causes weird timing bugs
    listen_for_msg();
  }
  
  //char potato[] = "This string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\nThis string is sixty four characters (achieved with difficulty)\n"; 
  //blink_Packet(potato, 1024);
  
  /*Serial.print(analogRead(SENSOR_PIN));
  Serial.print('\t');
  Serial.println(digitalRead(SENSOR_PIN));
  delay(5);*/
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
  bool timeoutFailure = false;
  char charBeingRead;
  int samplesCounted;

  digitalWrite(LED, HIGH);
  while(checkSensor(SENSOR_PIN)==0){} //Note: this hangs the board
  digitalWrite(LED, LOW);

  
  while(charsRead < MSG_BUF_LEN && stopChar == 0){

    //The 4 here needs to be changed to accept a different PPM value and I couldn't think of a smart way of doing it in advance
    for(int i = 0; i < 4; i++){
      delayMicroseconds(highTime); //Let start pulse pass
      timeoutFailure = false;
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

  Serial.print(msgBuf);
  clearMsgBuf();
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

/*int main()
{
    pc.printf("3 CM Link Board - Transmit\r\n");
    pc.printf("Enter pulse length in microseconds (10e-6), enter for 1000\r\n");
    while(1) {
        char d = pc.getc();
        if(d != '\n') {
            PULSE_LENGTH = PULSE_LENGTH*10 + (d-'0'); 
        }
        else {
            if (PULSE_LENGTH == 0) PULSE_LENGTH = 1000;
            pc.printf("Pulse length is "); 
            pc.printf("%d", PULSE_LENGTH);
            pc.printf("\r\n");
            break;
        }
    }
    
    //Packet
    char buffer[PACKET_LENGTH + 2];
    int idx = 0;
    while(1) {
        char a = pc.getc();
        
        //Fills buffer then transmits  
        if (a == '~') {
            digitalWrite(LASER,tx ^ 1;
            while (pc.getc() != '~');
        }
        else if(a != '\n' && idx < PACKET_LENGTH){
            buffer[idx] = a;
            idx++;
        }
        else {
            //Adds ending characters
            // No need to add line ending characters -BZ
//            buffer[idx] = '\r';
//            idx++;
//            buffer[idx] = '\n';
//            idx++;
            // add the checksum
            buffer[idx] = checksum(buffer, idx);
            idx++;
            
            //Transmits packet
            blink_packet(buffer, idx);
            idx = 0;
        }
    }
}*/
