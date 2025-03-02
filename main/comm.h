#pragma once
#include <utility>
#include "Wire.h"

bool checkSerial();
void clearSerial();
std::pair<char,char> readVictim();
