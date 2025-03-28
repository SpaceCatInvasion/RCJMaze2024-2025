#include "motor.h"

//Motor m(11,10);

Motor frontLeft(15,14);
Motor frontRight(12,13);
Motor backLeft(10,11);
Motor backRight(9,8);

volatile int enc = 0;
void enc_update(){
  if(digitalRead(ENC_PIN)==HIGH)
    enc++;
  else
    enc--;
}

void lmotors(int speed){
  frontLeft.speed(speed);
  backLeft.speed(-speed);
}
void rmotors(int speed){
  frontRight.speed(-speed);
  backRight.speed(speed);
}
void forward(int speed){
  lmotors(speed);
  rmotors(speed);
}
void backward(int speed){
  lmotors(-speed);
  rmotors(-speed);
}
void stop_motors(){
  lmotors(0);
  rmotors(0);
}

void forwardCm(int speed, int cm){
  enc=0;
  while(encToCm(enc)<cm){
    forward(speed);
  }
}
void backwardCm(int speed, int cm){
  enc = 0;
  while(encToCm(enc)>-cm){
    backward(speed);
  }
}

int cmToEnc(double cm){
  return cm*ENC_PER_ROT/(PI*WHEELDIA);
}
double encToCm(int enc){
  return enc*PI*WHEELDIA/ENC_PER_ROT;
}

void Motor::speed(int percent){
  int speed = abs(percent) * 255 / 100;
  if(percent>0){
    analogWrite(fpin,speed);
    analogWrite(rpin,0);
  }
  else{
    analogWrite(fpin,0);
    analogWrite(rpin,speed);
  }
}
Motor::Motor(int f, int r){
  fpin=f;
  rpin=r;
}