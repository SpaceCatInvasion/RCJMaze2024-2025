#include "bno.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28,&Wire1);

bool bnoInit(){
  //bno = Adafruit_BNO055(55, 0x28, &Wire1);
  return bno.begin(OPERATION_MODE_IMUPLUS);
}

int getBNO(){
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

int roundAngle(int angle){
  if(angle<0) angle=(angle%360)+360;
  return angle%90>45?(angle+90)-(angle%90): angle-(angle%90);
}

int getTilt(){
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.y;
}