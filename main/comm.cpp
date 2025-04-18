#include "comm.h"

void interruptFunc() {
  Serial.println("Interrupted!");
  while (Serial1.available()) Serial1.read();  // clear buffer
  stop_motors();
  delay(10);
  while (!Serial1.available());
  char side = (char)Serial1.read();
  while (!Serial1.available());
  char vic = (char)Serial1.read();
  Serial.print("Side: "); Serial.print(side); Serial.print(" Vic: "); Serial.println(vic);
  
  drop(side, vic);
  blink();
}

void commBegin() {
  Serial1.setTX(TX_PIN);
  Serial1.setRX(RX_PIN);
  Serial1.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptFunc, RISING);
}