#include "comm.h"

bool checkSerial(){
  return Serial2.available();
}

void clearSerial(){
  while(Serial2.available()) Serial2.read();
}

#ifdef CAM_ON
std::pair<char,char> readVictim(){
  std::pair<char,char> vic;
  vic.first = Serial2.read();
  vic.second = Serial2.read();
  return vic;
}
#else
std::pair<char,char> readVictim(){
  std::pair<char,char> vic;
  vic.first = -1;
  vic.second = -1;
  return vic;
}
#endif