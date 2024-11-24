#include "maze.h"
#include "Wire.h"
Robot robot;
Maze maze(&robot);




void setup(){
  Serial.begin(115200);
  delay(2000);
  while(!Serial);
  Serial.println("Serial Connected");
  
//   Wire.setSCL(17);
//   Wire.setSDA(16);
//   Wire.begin();
  
//   tofInit();

//   Wire1.setSCL(27);
//   Wire1.setSDA(26);
//   Wire1.begin();

//   Serial.println("Serial Connected");

//   if(!bnoInit()){
//  // if(!bno.begin(OPERATION_MODE_IMUPLUS)){
//     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//     while (1);
//   }
//   else {
//     Serial.println("HI");
//   }
  
  

}
void loop(){
  //Serial.print("Angle: "); Serial.print(getBNO()); Serial.print("; Tilt: "); Serial.print(getTilt()); Serial.print("; Rounded: "); Serial.println(roundAngle(getBNO()));
  //Serial.print("Distance: "); Serial.println(readTOF(FRONT_TOF));
  m.speed(100);
  delay(2000);
  m.speed(0);
  delay(200);
  m.speed(-100);
  delay(2000);
  m.speed(0);
  delay(200);
}
// void setup1(){

// }
// void loop1(){
  
// }