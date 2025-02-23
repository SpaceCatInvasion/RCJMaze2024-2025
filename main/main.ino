#include "maze.h"
#include "Wire.h"
Robot robot;
Maze maze(&robot);




void setup(){
  
  Serial.begin(115200);
  delay(2000);
  //while(!Serial);
  Serial.println("Serial Connected");

  pinMode(ENC_PIN,INPUT);
  pinMode(ENC_PIN_INTER,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_INTER), enc_update, RISING);

  Wire.setSCL(29);
  Wire.setSDA(28);
  Wire.begin();
  
  tofInit();

  // Wire1.setSCL(27);
  // Wire1.setSDA(26);
  // Wire1.begin();

  if(!bno.begin(OPERATION_MODE_IMUPLUS)){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  colorBegin();

  maze.updateTile();
  enc=0;
}
void loop(){


  std::vector<Direction> directions = {NORTH, NORTH, EAST, EAST, SOUTH, SOUTH, WEST, WEST}; 
  robot.moveDirections(directions);

  // switch(robot.status){
  //   case TRAVERSING:
  //     robot.moveDirections(maze.findNextMove());
  //     maze.updateTile();
  // }
}
// void setup1(){

// }
// void loop1(){
  
// }