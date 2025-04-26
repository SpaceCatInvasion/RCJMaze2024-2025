#pragma once
#include <utility>
#include "Wire.h"
#include "dropper.h"
#include "motor.h"
#include "led.h"
#include "pins.h"

extern volatile bool interrupted;
extern int restartPi;
void interruptFunc();
void commBegin();