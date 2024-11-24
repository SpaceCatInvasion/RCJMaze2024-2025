#include "motor.h"

Motor m(11,10);

volatile int enc = 0;


void lmotor(int speed){

}
void rmotor(int speed){

}
void forward(int speed){

}
void backward(int speed){

}
void stop(){

}

inline int cmToEnc(double cm){
  return cm*ENC_PER_ROT/(PI*WHEELDIA);
}
inline double encToCm(int enc){
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