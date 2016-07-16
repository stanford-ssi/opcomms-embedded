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
//  digitalWrite(LED, HIGH);
//  //while(checkSensor(SENSOR_PIN)==0){} //Note: this hangs the board
//  while(checkSensor(SENSOR_PIN)==0 && !Serial.available()){} //Note: this hangs the board until a message arrives or other input received, whatever comes first
//  digitalWrite(LED, LOW);
//
//  if(Serial.available()){
//    waitMode = false;
//    return -1; //Time out if input receiced
//  }
  
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





//*******Below is the alternative transmit interrupt code***************************************************************
//**********************************************************************************************************************
//**********************************************************************************************************************
//**********************************************************************************************************************
//**********************************************************************************************************************


//Swapping to Timer1 library: http://www.pjrc.com/teensy/td_libs_TimerOne.html



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

#include <TimerOne.h>
volatile unsigned long transmit_period = 400;
//volatile unsigned long transmit_period = 100;// + clock_offset;

//void hi_setup(){
//  Timer1.initialize(transmit_period);
//  Timer1.attachInterrupt(transmit_timer_tick);
//  Timer1.stop();
//}

void transmit_msg(char* msg, int m_length){
  cli();
  noInterrupts()
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
  //transmit_timer.begin(transmit_timer_tick, transmit_period);
  transmitting = true;
  Timer1.start();
  interrupts(); //For some reason the interrupts() flag doesn't work. Used the above boolean instead.
  sei();
}

void transmit_timer_tick(){
  noInterrupts();
  bool eom = buffer_location>msg_length-1;
//  Serial.print(" buffer location: "); 
//  Serial.print(buffer_location);
//  Serial.print(" time waited: ");
//  Serial.print(time_waited);
//  Serial.print(" ");
//  Serial.print(laser_on);
//  Serial.print(" ");
//  Serial.print(last_period);
//  last_period = micros();
  
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
 * Hardware Interrupt Reciever Code implemented with listen_for_msg();
 * ============================================================================
 */

// Her Majesty, The Interrupt Service Routine
// Runs at (currently) 25us intervals and processes message given a signal during that interval
// Of course there are some shenanigans with the timing that would make the court jester laugh but functional. 
void receive_interrupt() {
  noInterrupts();
  if(analogRead(SENSOR_PIN) > sensorThreshold){
   listen_for_msg();
   Serial.println(msgBuf);
   clearMsgBuf();
  }
  interrupts();
}


/* ============================================================================
 * Hardware Interrupt Reciever Code using Buffers
 * ! Now Defunct :( !
 * ============================================================================
 */

// Constants
const int msg_length_max = 64;                                // Maximum Size of A Given Message
const int analog_buffer_size = 2048;                          // Maximum Size of the Global Analog Buffer
const int pro_size = 256;                                     // Size of the buffer to be processed in each chunk. Designed for one char each time


// Volatiles
volatile int analog_buffer[analog_buffer_size] = {0};         // His Majesty, The Global Analog Buffer
volatile int glob_analog_buffer_index;                        // Refers to the "placing index" for the ISR
volatile int glob_analog_buffer_processing_index = 0;         // Refers to the index of the Global Analog Buffer where chunk processing begins
volatile int chars_read_so_far = 0;
volatile char received_message_buffer[msg_length_max] = {0};
volatile bool received_message_ready = false;                 // Flag indicating if a received message is available
//volatile bool start_msg_decode = false;

/*
 * Function: bool aboveThreshold(int i)
 * --------------------------------------------------
 * Predicate function returning if a value is above the threshold for activation
 *
 */
bool aboveThreshold(int i) {
  return (i > sensorThreshold);
}


/*
 * bool chunkAvailable(int main_buffer_index, int starting_index)
 * --------------------------------------------------
 * Predicate function returning if our chunk is large enough for processing
 *
 */
bool chunkAvailable(int main_buffer_index, int starting_index) {
  if ((main_buffer_index - starting_index) < pro_size && (main_buffer_index - starting_index + analog_buffer_size < pro_size)) return true;
  return false;
}



/*
 * void print_message_buffer()
 * --------------------------------------------------
 * Prints a received message!
 *
 */
void print_message_buffer() {
  noInterrupts();
  char loc_msg_buff[msg_length_max];                        // Initialize Local Message Buffer Array
  for (int j = 0; j < msg_length_max; j++) {                // Copies characters from volatile global
    loc_msg_buff[j] = received_message_buffer[j];
    received_message_buffer[j] = 0;                         // Clears buffer as its read
  }
  received_message_ready = false;                           // Toggle flag
  interrupts();
  Serial.println(loc_msg_buff);                             // Send message to serial
}

void print_buffer(){
  noInterrupts();
    for (int i = 0; i < glob_analog_buffer_index; i++){
      Serial.print(analog_buffer[i]);
      Serial.print(", ");
      Serial.println(" ");
    }
  interrupts();
}

/*
 * Function: void decode_msg_buffer()
 * --------------------------------------------------
 * Monitors the buffer that is filled with every ADC trigger. Must be placed in the main loop function.
 *
 */
void decode_msg_buffer() {
  Serial.println("Hello! I'm here at the beginning!");
  // Takes local copies of all global vars.
  noInterrupts();
  int starting_index = glob_analog_buffer_processing_index;                      // Starting_index refers to global position of chunk processing
  int an_buffer[pro_size];                                                       // Refers to local copy of analog buffer, hereby referred to as "the chunk"
  for (int j = 0; j < pro_size; j++) {                                           // Populate the chunk from the global buffer
    an_buffer[j] = analog_buffer[(j + starting_index) % analog_buffer_size];     // Wrap Around
  }
  int main_buffer_index = glob_analog_buffer_index;                              // Refers to the "placing index"
  interrupts();


  // Looks to see if a message is ready to be printed. If not, it parses some more.
  if (received_message_ready) {
    print_message_buffer();
    return;
  }



  //All the buffer alignment, index position checks, etc.
  if (chunkAvailable(main_buffer_index, starting_index)) return;         // Chunk to be processed too small - We're too close to the current sampling index. Try again later.
  int curr_index = 0;
  while (!aboveThreshold(an_buffer[curr_index])) {
    curr_index++;                                                       // Normalizes us so we always start the decoding (the for loop below) at the initial laser pulse
  }
  if (curr_index > 10) {                                               // Did not start close to a starting pulse; no messages to see or mean mis-aligned timing. Advances through periods of darkness.
    noInterrupts();
    glob_analog_buffer_processing_index = (starting_index + pro_size) % analog_buffer_size; // Advances the index where we would begin chunk processing.
    interrupts();
    return;
  }


  //Decoding the buffer now into an actual byte.
  char char_being_read;

  for (int k = 0; k < 4; k++) {
    Serial.println("Hello! I'm here!");
    while (an_buffer[curr_index] >= sensorThreshold) {
      curr_index++;                                                   // Moves past initial pulse
    }
    if (curr_index > 20) {                                            //Psyche, initial pulse was the EOF pulse. Message now ready to be sent.
      noInterrupts();
      received_message_ready = true;
      interrupts();
      return;
    }

    int samples_counted = 0;

    while (an_buffer[curr_index] < sensorThreshold) {       //Counts number of samples between initial pulse and following pulse.
      curr_index++;
      samples_counted++;
      if (curr_index >= pro_size) {               //ERROR: Finishing pulse not found.
        noInterrupts();
        glob_analog_buffer_processing_index = (starting_index + pro_size) % analog_buffer_size;
        interrupts();
        return;
      }
    }

    samples_counted = samplesCountedToNibblet(samples_counted);

    char_being_read = (char_being_read << 2) | samples_counted; //Moves existing bits to the left, inserts new bits on right. Magic number 2 comes from the number of bits decoded with each symbol.
  }//end for


  noInterrupts();
  received_message_buffer[chars_read_so_far] = char_being_read;
  chars_read_so_far++;
  glob_analog_buffer_processing_index = (starting_index + curr_index) % analog_buffer_size; //Possible Bug, may be starting one before the pulse window. Double check.
  interrupts();
  return;
}
