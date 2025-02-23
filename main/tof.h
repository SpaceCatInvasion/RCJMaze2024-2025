#pragma once
#include "Adafruit_VL53L0X.h"
#include "Wire.h"

extern Adafruit_VL53L0X tof;

#define TCAADDR 0x70

#define FRONT_TOF 1
#define LEFT_TOF 2
#define RIGHT_TOF 0
// #define LEFTB_TOF 1
// #define RIGHTB_TOF 7
#define BACK_TOF 3

const int tofs[6] = {FRONT_TOF, LEFT_TOF, RIGHT_TOF, BACK_TOF};

void tofInit();
double readTOF(int num);