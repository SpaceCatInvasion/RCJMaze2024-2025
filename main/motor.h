#pragma once
#include "robot.h"

#define kP 0.5
#define kD 0.001

extern volatile int enc;

void lmotor(int speed);
void rmotor(int speed);
void forward(int speed);
void backward(int speed);
void stop();

inline int cmToEnc(double cm);
inline double encToCm(int enc);

class Motor {
public:
  uint8_t fpin;
  uint8_t rpin;
  void speed(int percent);
  Motor(int f, int r);
};

extern Motor m;