#define RX 2
#define TX 3
#define EN_PIN 4

#define BITTIME 45

#define POS '%' //37
#define NEG '$' //36
#define AZM 16
#define ALT 17
#define LEFT POS + (2*AZM) //69
#define RIGHT NEG + (2*AZM) //68
#define UP NEG + (2*ALT) //70
#define DOWN POS + (2*ALT) //71

void setup()
{
  Serial.begin(250000);
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(TX, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  digitalWrite(TX, HIGH);
  
  delay(1000);
  
  /*beginCmd();
  celestronWrite(';');
  celestronWrite(4);
  celestronWrite('\r');
  celestronWrite(16);
  celestronWrite('%');
  celestronWrite(9); //speed character
  celestronWrite(177);
  endCmd();*/
  
  /*celestron.print(';');
  celestron.print(4);
  celestron.print('\r');
  celestron.print(16);
  celestron.print('%');
  celestron.print('\t');
  celestron.print(177);*/
  
  
  /*beginCmd();
  celestronWrite(';');
  celestronWrite(4);
  celestronWrite('\r');
  celestronWrite(16);
  celestronWrite('%');
  celestronWrite(0); //speed character
  celestronWrite(186);
  endCmd();*/
}

void loop() // run over and over
{
  celestronMoveCmd(UP,9);
  delay(2000);
  celestronMoveCmd(UP,0);
  delay(1000);
  celestronMoveCmd(LEFT,9);
  delay(2000);
  celestronMoveCmd(LEFT,0);
  delay(1000);
  celestronMoveCmd(DOWN,9);
  delay(2000);
  celestronMoveCmd(DOWN,0);
  delay(1000);
  celestronMoveCmd(RIGHT,9);
  delay(2000);
  celestronMoveCmd(RIGHT,0);
  delay(1000);
}

void beginCmd(){
  pinMode(EN_PIN, OUTPUT);
  //pinMode(TX, OUTPUT);
  digitalWrite(EN_PIN,LOW);
  //digitalWrite(TX, HIGH);
}

void endCmd(){
  digitalWrite(EN_PIN,HIGH); //Charge pin before changing to input
  pinMode(EN_PIN, INPUT_PULLUP);
  //pinMode(TX, INPUT_PULLUP);
}

void celestronWrite(char c){
  digitalWrite(TX, LOW);
  delayMicroseconds(BITTIME);
  for(int i = 0; i < 8; i++){
    digitalWrite(TX,c & 0b1); //Mask to set pin to value of LSB
    c = c >> 1; //Move to next LSB
    delayMicroseconds(BITTIME);
  }
  digitalWrite(TX, HIGH);
  delayMicroseconds(BITTIME);
}

void celestronMoveCmd(char dir, int spd){
  char axis = (dir - NEG) / 2; //convert direction to axis variable
  char realdir = dir - (2 *axis); //convert "direction" constant to actual value needed by Celestron

  unsigned char lastChar = 239 - (realdir + axis + spd);
  /*Serial.println(axis,DEC);
  Serial.println(realdir,DEC);
  Serial.println(lastChar,DEC);*/
  
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

