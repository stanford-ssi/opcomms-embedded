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

//int listen_for_msg(){
//  int charsRead = 0;
//  bool stopChar = 0;
//  char charBeingRead;
//  int samplesCounted;
//
//
//noInterrupts();
//
//  elapsedMillis waiting;
//  
//  while(charsRead < MSG_BUF_LEN && stopChar == 0){
//
//    //The 4 here needs to be changed to accept a different PPM value and I couldn't think of a smart way of doing it in advance
//    for(int i = 0; i < 4; i++){     //done 4 times to assemble byte
//      delayMicroseconds(highTime); //Let start pulse pass (should be 100 microsecs)
//      samplesCounted = 0;           //resets every iteration (4 times/byte)
//
//      //  EOP check
//      while(checkSensor(SENSOR_PIN)==1){
//        samplesCounted++;
//      }
//      if(samplesCounted > 20){  //  may cost some time
//        stopChar = 1; //detect end of packet
//        //Serial.println("EOP");
//      }
//
//      if(!stopChar){
//        samplesCounted = 0;
//        while(samplesCounted < 100 && checkSensor(SENSOR_PIN)==0){
//          samplesCounted++;
//          delayMicroseconds(sampleTime);    //~drift may be caused by sampleTime 
//        }
//
//        if(samplesCounted >= 100) stopChar = 1; //Detect failure and terminate packet
//         
//        samplesCounted = samplesCountedToNibblet(samplesCounted);
//        charBeingRead = (charBeingRead << N_BITS) | samplesCounted; //Moves existing bits to the left, inserts new bits on right
//      }
//    }
//    
//    if(!stopChar){
//      msgBuf[charsRead] = charBeingRead;
//      charsRead++;
//    }
//    
//  }
//
//  interrupts();
//  return charsRead;
//
//}

///*Function: samplesCountedToNibblet
// * 
// * Postconditions:
// *      returns the actual bits of data represented by samplesCounted
// * 
// * Method of Operation:
// *      samplesCounted should equal the number of low intervals observed,
// *      leftshifted by sampleTimeShiftVal. This function rightshifts to
// *      correct for that and then subtracts 1 to make 1 interval correspond
// *      to 0, 2 intervals to 1, etc. 
// *      
// * This function is discrete only because this comment block would have
// * to be stupidly long in an already long function
// */
 
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





/*******Below is the alternative transmit interrupt code***************************************************************
**********************************************************************************************************************
**********************************************************************************************************************
**********************************************************************************************************************
**********************************************************************************************************************/


//Swapping to Timer1 library: http://www.pjrc.com/teensy/td_libs_TimerOne.html
#include <TimerOne.h>


//IntervalTimer transmit_timer;
static const int buffer_size = 256;
const int clock_offset = 0;
volatile int msg_buffer[buffer_size] = {0};
volatile int buffer_location = 0;
volatile int time_waited = 0;
volatile int msg_length = 0;
volatile bool laser_on = false;

volatile unsigned long last_period = 0;
volatile unsigned long time_since_ten = 0;
volatile int periods_since_ten = 0;
volatile unsigned long transmit_period = 400;

void transmit_msg(char* msg, int m_length){
  Timer1.stop();
  if(transmitting){
    Serial.println("Currently Transmitting, please try again");
    interrupts(); //For some reason the interrupts() flag doesn't work. Used the above boolean instead.
    sei();
    return;
  }
  if(m_length*4+4>buffer_size){
    msg_length = buffer_size-4; //-4 to account for EOF char, while still staying divisible by 4.
  }else{
    msg_length = m_length*4;
  }
  Serial.println(msg);
  for(int i=0; i<msg_length/4; i++){
       for(int j=3; j>=0; j--){
        //Bit mask each value in the ascii char, then shift it back to only two bits.
        //Place the values in the proper order in the buffer
        msg_buffer[i*4+3-j] = ((msg[i]&(0b11<<j*2))>>j*2);
       }
  }
  
  msg_buffer[msg_length] = (0b1<<N_BITS)+1 +1; //Encodes the EOF wait time at end of buffer.
  transmitting = true;
  Timer1.start();
  //interrupts(); //For some reason the interrupts() flag doesn't work. Used the above boolean instead.
  //sei();
}

void transmit_timer_tick(){
  noInterrupts();
  bool eom = buffer_location>msg_length-1;
  
  if(time_waited == 0){//Just got to this buffer_location, send starting pulse
    digitalWrite(LASER, HIGH);
    laser_on = true;
    time_waited++;
    last_period = micros();
    interrupts(); 
    return;
  }
  if(time_waited>0 && laser_on && !eom){//Finish the starting pulse, if EOM keep the laser on for the long pulse.
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
      Serial.println("Transmission Completed");
      Timer1.stop();
    }
    
    last_period = micros();
    interrupts(); 
    return;
  }
  //If we get to here that means, 1.Didn't just start. 2.Haven't waited long enough.
  time_waited++;
  last_period = micros();
  interrupts(); 
  return;
}



void reset_buffer(){
  for( int i=0; i<buffer_size; i++){
    msg_buffer[i] = 0;
  }
  buffer_location = 0;
  time_waited = 0;
  msg_length = 0;
}

/* ============================================================================
 * Hardware Interrupt Reciever Code With Correlator
 * M Taylor and E Millares
 * Aug 19th 2016
 * ============================================================================
 */
// CONSTANTS
static const int oversamplingRatio = 5;
//int decision_threshold = 0;
//int noise_threshold = 0;

// VOLATILE
volatile int sample_buffer[oversamplingRatio] = {0};
volatile int sample_buffer_counter = 0;
volatile int correlator_buffer[oversamplingRatio] = {0};
volatile int next_sample_counter = 0;
volatile int since_last_high = 0;
volatile bool msg_done = false;
volatile bool msg_error = false;
volatile int two_bit_count = 3;
volatile int charsRead = 0;
volatile int charBeingRead;
volatile bool msg_start = false;
volatile bool timing_lock = false;
volatile int onecount = 0;
volatile int zerocount = 0;


// Entire decode operation taken over by Timer 3 interrupt. No more calling functions with delays.
void receive_interrupt() {
// TEMP VARIABLES
// value to store this samples new correlator output
int correlator_value = 0;
int early = 0;
int late = 0;
int sample = 0;
int ELG_avg = 0;
  
  noInterrupts();

// CALCULATE DECISION THRESHOLD
// This can really be done in the calibration measurement function, but if we want to make it adaptive it could happen here too.
// TODO: ADD VARIANCE FROM SAMPLE TIMING UNCERTAINTY USING EARLY LATE GATE WITH FIXED SAMPLE INTERVALS (should be ~2xsample time /12 variance -> correlator slope)


// ACQUIRE NEW SAMPLE
// order of samples does not matter, only need to keep last (oversamplingRatio) worth of samples for correlator
sample_buffer[sample_buffer_counter] = analogRead(SENSOR_PIN);
//Serial.println(sample_buffer[sample_buffer_counter]);
sample_buffer_counter ++;
if (sample_buffer_counter == oversamplingRatio){
  sample_buffer_counter = 0;
}

// making space for new correlator value output
// keeping order of values
for (int i = 0 ; i < (oversamplingRatio-1) ; i++){
  correlator_buffer[i] = correlator_buffer[i+1];
}

// calculate new correlator value
for (int i = 0 ; i<oversamplingRatio ;i++){
  correlator_value = correlator_value + sample_buffer[i];
}
// normalize energy of correlator (divide by # of points)
correlator_value = correlator_value/oversamplingRatio;
//Serial.println(correlator_value);

correlator_buffer[oversamplingRatio-1] = correlator_value;
early = correlator_buffer[1];
late  = correlator_buffer[3];
sample = correlator_buffer[2];

ELG_avg = (early + late + sample )/3;
//Serial.println(ELG_avg);
// search for first correlator peak
// when the first sample before and first sample after are both lower than the middle one, you should be on the peak of the correlator triangle
// this should REALLY not be a one-shot estimation, it should be trying to find the peak over many pulses to gaurentee it is properly aligned
// but since we need it to start decoding RIGHT AWAY, this is a problem.
// the criteria for all three points not being noise is currently set to be that their average is at least 0.5* the expected decision threshold from measurements.
// This will proably need tweaking.
if ((timing_lock == false) && ( ((early < sample) && sample>decision_threshold ) || ((late<sample) && sample>decision_threshold)) && (correlator_buffer[0]<decision_threshold || correlator_buffer[4]<decision_threshold)){
  timing_lock = true;
  msg_start = true;
  next_sample_counter = 0;
  since_last_high = 0;
}

// EARLY LATE GATE and DECISION THRESHOLD
// chooses best sample for this pulse

// if you think you are looking at the right samples and are roughly symbol aligned
if(timing_lock == true){
  // if it's time to make a decision
  if(next_sample_counter == 0){
    
    // try to "balance" on the peak of the correlator curve
    if(early > sample && late < sample && early > decision_threshold){
      // if the sample before where you think the peak is turns out to be bigger than the peak you are sampling too slow, drop a sample to speed up
      next_sample_counter = oversamplingRatio -2;
      sample = early;
    } else if (late > sample && early < sample && late > decision_threshold){
      // if the sample after where you think the peak is turns out to be bigger than the peak you are sampling too fast, add a sample to slow down
      next_sample_counter = oversamplingRatio;  
      sample = late;    
    } else{
      // otherwise you are right on time
      next_sample_counter = oversamplingRatio-1;
    }

    // SYMBOL DECISION
    if(sample>decision_threshold){
      //Serial.println("1");
      // if the correlator value is above the threshold, you are estimating that a pulse was sent here
      if(since_last_high >=10){
        // if you see two high pulses back to back, the message is done
        msg_done = true;
        timing_lock = false;
        since_last_high = 0;
      } else if(since_last_high <5 && since_last_high >0){
        // DECODE 
        // if buffer is full stop and print what you have
        
        if(charsRead>=MSG_BUF_LEN){
          msg_done = true;
          timing_lock = false;
          msg_error = true;
          two_bit_count = 3;
          charBeingRead = 0;
          charsRead = 0;
        }
        
        //4 symbols to fill 1 char
        // 1024 max chars in buffer
        if(two_bit_count==0){
          charBeingRead = charBeingRead + (((since_last_high-1)&0b11) << (2*two_bit_count));
          msgBuf[charsRead] = charBeingRead;
          charsRead++;  
          charBeingRead = 0;
          two_bit_count = 3;
        } else {
            charBeingRead = charBeingRead + (((since_last_high-1)&0b11) << (2*two_bit_count));
            two_bit_count--;
        }
        since_last_high = 0;
      } 
    } else {
      // otherwise you are assuming there was no pulse
        //Serial.println("0");
      if(since_last_high >=10){
        // if you see two high pulses back to back, the message is done
        msg_done = true;
        timing_lock = false;
        since_last_high = 0;
      }        
        // if you are still in the message, keep counting blank spaces between high pulses
        since_last_high++;
    }   
  } else {
    // if you think you are counting things out right but its not time to make a decision, then wait until it is
    next_sample_counter --;
  }
}

// DISPLAY MESSAGE
if(msg_done){
  Serial.println("Message:");
  Serial.println(msgBuf);
  clearMsgBuf();
  charsRead = 0;
  charBeingRead = 0;
  two_bit_count = 3;
  msg_done = false;
  timing_lock = false;
}

  interrupts();
}
