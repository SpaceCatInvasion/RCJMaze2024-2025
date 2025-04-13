#pragma once
#include <utility>
#include "Wire.h"

void commBegin();
bool checkSerial();
void clearSerial();
std::pair<char,char> readVictim();
