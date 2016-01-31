#define LASER 5
#define N_BITS 2
#define SENSOR_PIN A0
#define MSG_BUF_LEN 10

//These are not explicitly constant because they should be field configurable
int highTime = 100000; //us
int lowTime = 100000; //us
int sampleTime = 24000; //Should be ~10us less than a factor of lowTime
int sampleTimeShiftVal = 2; //Rightshifting is much cheaper than dividing; 2^this is how many samples per interval
int sensorThreshold = 8;


#define WAIT_FOR_MSG_TIMEOUT 500000 //us

char msgBuf[MSG_BUF_LEN];

void setup() {
  pinMode(LASER, OUTPUT);
  Serial.begin(250000);
}

void loop() {
  /*char potato[] = "potato"; 
  blink_Packet(potato, 6);*/
  
  if(listen_for_msg() == 0) Serial.println("Timed out");
  
  /*Serial.print(analogRead(SENSOR_PIN));
  Serial.print('\t');
  Serial.println(digitalRead(SENSOR_PIN));
  delay(5);*/
}

/*Function: listen_for_msg:
    Postconditions:
        populates MSG_BUF_LEN with received bytes
*/

bool checkSensor(int pin){
  return (analogRead(pin) > sensorThreshold);
}

int listen_for_msg(){
  int charsRead = 0;
  bool stopChar = 0;
  long startTime = micros();
  bool timeoutFailure = false;
  char charBeingRead;
  int samplesCounted;

  //Wait for beginning of transmission; return failed if timeout exceeded
  while(!timeoutFailure && checkSensor(SENSOR_PIN)==0){
    timeoutFailure = micros()-startTime < WAIT_FOR_MSG_TIMEOUT;
  }
  
  if(timeoutFailure) return 0;

  
  while(charsRead < MSG_BUF_LEN && stopChar == 0){

    //The 4 here needs to be changed to accept a different PPM value and I couldn't think of a smart way of doing it in advance
    for(int i = 0; i < 4; i++){
      delayMicroseconds(highTime); //Let start pulse pass
      timeoutFailure = false;
      startTime = micros();
      samplesCounted = 0;
          
      while(samplesCounted < 100 && checkSensor(SENSOR_PIN)==0){
        samplesCounted += 1;
        delayMicroseconds(sampleTime);
      }

      Serial.print(micros());
      Serial.print(' ');
      Serial.println(samplesCounted);
       
      samplesCounted = samplesCountedToNibblet(samplesCounted);
      charBeingRead = (charBeingRead << N_BITS) | samplesCounted; //Moves existing bits to the left, inserts new bits on right
    }
    msgBuf[charsRead] = charBeingRead;
    Serial.println(charBeingRead);
  
    charsRead++;
  }

  Serial.println(msgBuf);
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
    Serial.println(data);
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
