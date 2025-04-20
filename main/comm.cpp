#include "comm.h"


volatile bool interrupted = false;
void setFlag(){
  stop_motors();
  interrupted = true;
}

void interruptFunc() {
  Serial.println("Interrupted!");
  stop_motors(); delay(500);
  forwardCm(30,8);
  stop_motors(); delay(250);
  // while (Serial1.available()) Serial1.read();  // clear buffer
  while (!Serial1.available()) Serial.println("waiting1");
  char side = (char)Serial1.read();
  while (!Serial1.available()) Serial.println("waiting2");
  char vic = (char)Serial1.read();
  Serial.print("Side: "); Serial.print(side); Serial.print(" Vic: "); Serial.println(vic);
  
  dropVictims(side, vic);
  blink();
  backwardCm(30,8);
  stop_motors(); delay(250);
  interrupted=false;
}

void commBegin() {
  Serial1.setTX(TX_PIN);
  Serial1.setRX(RX_PIN);
  Serial1.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), setFlag, RISING);
}