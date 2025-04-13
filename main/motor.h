#pragma once
#include "robot.h"

#define kP 5
#define kD 1

extern volatile int enc;
#define ENC_PIN_INTER 17
#define ENC_PIN 16
void enc_update();

void lmotors(int speed);
void rmotors(int speed);
void forward(int speed);
// void forward(int lspeed, int rspeed, bool x);
void backward(int speed);
void stop_motors();

void forwardCm(int speed, int cm);
void backwardCm(int speed, int cm);

 int cmToEnc(double cm);
 double encToCm(int enc);

class Motor {
public:
  uint8_t fpin;
  uint8_t rpin;
  void speed(int percent);
  Motor(int f, int r);
};

// extern Motor m;

extern Motor frontLeft;
extern Motor frontRight;
extern Motor backLeft;
extern Motor backRight;