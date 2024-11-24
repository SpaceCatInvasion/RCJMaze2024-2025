#pragma once
#include "Adafruit_VL53L0X.h"
#include "Wire.h"

extern Adafruit_VL53L0X tof;

#define TCAADDR 0x70

#define FRONT_TOF 1
#define LEFT_TOF 0
#define RIGHT_TOF 0

const int tofs[1] = {FRONT_TOF};

void tofInit();
double readTOF(int num);