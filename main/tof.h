#pragma once
#include "Adafruit_VL53L0X.h"
#include "Wire.h"


extern Adafruit_VL53L0X tof;

#define TCAADDR 0x70

// ASR bot
// #define FRONT_TOF 2
// #define LEFT_TOF 3
// #define RIGHT_TOF 7
// #define BACK_TOF 1

#define FRONT_TOF 1
#define LEFT_TOF 7
#define RIGHT_TOF 0
#define BACK_TOF 4

const int tofs[6] = { RIGHT_TOF, FRONT_TOF, LEFT_TOF, BACK_TOF };

void tofInit();
double readTOF(int num);
void printTOFs();