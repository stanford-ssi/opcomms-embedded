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
  while(checkSensor(SENSOR_PIN)==0 && !Serial.available()){} //Note: this hangs the board until a message arrives or other input received, whatever comes first
  digitalWrite(LED, LOW);

  if(Serial.available()){
    waitMode = false;
    return -1; //Time out if input receiced
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

void query(){
  query(false);
}

void query(bool dangerous){
  Serial.print(celestronGetPos(AZM, dangerous));
  Serial.print(' ');
  Serial.print(celestronGetPos(ALT, dangerous));
  Serial.print(' ');
  Serial.print(sampleSensor());
  Serial.print(' ');
  if(bnoEnabled && bnoVerbose) queryIMU();
  Serial.println();
}





//*******Below is the alternative transmit interrupt code******************
IntervalTimer transmit_timer;
static const int buffer_size = 256;
const int clock_offset = 14;
volatile int msg_buffer[buffer_size] = {0};
volatile int buffer_location = 0;
volatile int time_waited = 0;
volatile int msg_length = 0;
volatile bool laser_on = false;
volatile bool transmitting = false;
volatile unsigned long last_period = 0;
volatile unsigned long transmit_period = 300 + clock_offset;

void transmit_msg(char* msg, int m_length){
  noInterrupts();
  if(transmitting){
    Serial.println("Currently Transmitting, please try again");
    return;
  }
  if(m_length*4+6>buffer_size){
    msg_length = buffer_size-4; //-4 to account for EOF char, while still staying divisible by 4.
  }else{
    msg_length = m_length*4;
  }
  Serial.println(msg);
  for(int i=0; i<msg_length/4; i++){
       Serial.print(msg[i]);
       Serial.print(" encoded to: ");
      
       for(int j=3; j>=0; j--){
        msg_buffer[i*4+3-j] = ((msg[i]&(0b11<<j*2))>>j*2);
        Serial.print(msg_buffer[i*4+3-j]);
        //Bit mask each value in the ascii char, then shift it back to only two bits.
        //Place the values in the proper order in the buffer
       }
       Serial.println("");
  }
  msg_buffer[msg_length] = (0b1<<N_BITS)+1 +1; //Encodes the EOF wait time at end of buffer.
  Serial.print("Added eof: ");
  Serial.println((0b1<<N_BITS)+1);
  transmit_timer.begin(transmit_timer_tick, transmit_period);
  transmitting = true;
  interrupts(); //For some reason the interrupts() flag doesn't work. Used the above boolean instead.
}

void transmit_timer_tick(){
  noInterrupts();
  if(!transmitting) return;
  Serial.print("Time: ");
  Serial.print(micros()-last_period); 
  bool eom = buffer_location>msg_length-1;
  Serial.print(" buffer location: "); 
  Serial.print(buffer_location);
  Serial.print(" time waited: ");
  Serial.print(time_waited);
  Serial.println(laser_on);
  Serial.println(eom);

  if(laser_on){
    digitalWrite(LASER,LOW);
    laser_on = false;
  }else{
    digitalWrite(LASER,HIGH);
    laser_on = true;
  }
  
  digitalWrite(LASER,HIGH);
  delayMicroseconds(100);
  digitalWrite(LASER,LOW);
  
  last_period = micros();
  interrupts();
  return;
/*
  if(time_waited == 0){//Just got to this buffer_location, send starting pulse
    digitalWrite(LASER, HIGH);
    laser_on = true;
    time_waited++;
    
    last_period = micros();
    interrupts(); 
    return;
  }
  if(time_waited>0 & laser_on & !eom){//Finish the starting pulse, if EOM keep the laser on for the long pulse.
    digitalWrite(LASER,LOW);
    laser_on = false;
  }
  if(time_waited > msg_buffer[buffer_location]){//Have waited the appropriate pulse time, finish with this bit.
    time_waited = 0;
    buffer_location++;
    if(eom){//Finished EOM signal, time to wrap up all the transmition stuff.
      digitalWrite(LASER,LOW);
      laser_on=false;
      reset_buffer();
      transmitting = false;
      transmit_timer.end();
    }
    
    last_period = micros();
    interrupts(); 
    return;
  }
  //If we get to here that means, 1.Didn't just start. 2.Haven't waited long enough.
  time_waited++;
  last_period = micros();
  interrupts(); 

  */
}



void reset_buffer(){
  for( int i=0; i<buffer_size; i++){
    msg_buffer[i] = 0;
  }
  buffer_location = 0;
  time_waited = 0;
  msg_length = 0;
}

