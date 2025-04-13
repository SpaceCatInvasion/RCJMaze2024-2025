#pragma once
#include <utility>
#include "Wire.h"
#include "dropper.h"
#include "motor.h"

#define INTERRUPT_PIN 7
#define TX_PIN 28
#define RX_PIN 29

void interruptFunc();
void commBegin();