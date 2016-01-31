#define LASER 5
#define N_BITS 2
#define N_BITMASK 0b11

int highTime = 1000;
int lowTime = 1000;

void setup() {
  pinMode(LASER, OUTPUT);

}

void loop() {
  ppm('Q');
  delay(1000);
}

void ppm(char toSend){
  int mask = N_BITMASK;
  for(int i = 0; i < N_BITS; i++){
    int bitsToSend = toSend & mask;
    bitsToSend >> (N_BITS*i); //align with the right side of the byte
    
    digitalWrite(LASER, HIGH);
    delayMicroseconds(highTime);
    digitalWrite(LASER, LOW);
    delayMicroseconds(lowTime * (1 + bitsToSend));

    mask << (N_BITS*i);
  }
  digitalWrite(LASER, HIGH);
  delayMicroseconds(highTime);
  digitalWrite(LASER, LOW);
}

