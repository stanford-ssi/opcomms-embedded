
//Default length blink for when you can't be bothered to specify a length
void blinkLED(){
  blinkLED(200);
}

//Turns LED on pin 13 on for a specified period of time
void blinkLED(int onTimeIn_ms){
  digitalWrite(LED, HIGH);
  delay(onTimeIn_ms);
  digitalWrite(LED, LOW);
}


//RGB LED color function
void ledColor(byte color){
  digitalWrite(LED_R, !(0b1 & color));
  digitalWrite(LED_G, !(0b10 & color));
  digitalWrite(LED_B, !(0b100 & color));
}


void ledParty(){
    ledColor(LED_RED);
    delay(100);
    ledColor(LED_YELLOW);
    delay(100);
    ledColor(LED_GREEN);
    delay(100);
    ledColor(LED_TEAL);
    delay(100);
    ledColor(LED_BLUE);
    delay(100);
    ledColor(LED_PURPLE);
    delay(100);
    ledColor(LED_WHITE);
    delay(100);
    ledColor(LED_OFF);
}

