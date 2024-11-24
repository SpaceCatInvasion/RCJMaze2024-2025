#pragma once
#include <Adafruit_BNO055.h>

extern Adafruit_BNO055 bno;

bool bnoInit();
int getBNO();
int roundAngle(int angle);
int getTilt();