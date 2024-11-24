#include "tof.h"

Adafruit_VL53L0X tof = Adafruit_VL53L0X();

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission(); 
}

void tofInit(){
  //tof = Adafruit_VL53L0X();
  for(int i:tofs){
    tcaselect(i);
    tof.begin();
  }
}

double readTOF(int num){
 tcaselect(num);
 VL53L0X_RangingMeasurementData_t measure;
 tof.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
 if(measure.RangeStatus == 4){
   return 8196;
 }
 return measure.RangeMilliMeter*6.25/80;
}