#include "led.h"

void blink(int amt){
  for(int i=0;i<amt;i++){
    digitalWrite(LED_PIN,HIGH);
    delay(500);
    digitalWrite(LED_PIN,LOW);
    delay(500);
  }
}