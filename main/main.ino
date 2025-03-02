#include "maze.h"
#include "Wire.h"
Robot robot;
Maze maze(&robot);




void setup(){
  Serial.begin(115200);
  delay(2000);
  //while(!Serial);
  Serial.println("Serial Connected");
  pinMode(20, INPUT);
  while(digitalRead(20)==HIGH);
  

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

  commBegin();

  maze.updateTile();
  enc=0;
}



void loop(){


  // std::vector<Direction> directions = {NORTH, NORTH, EAST, EAST, SOUTH, SOUTH, WEST, WEST}; 
  // robot.moveDirections(directions);

  switch(robot.status){
    case TRAVERSING:
    case DANGERZONE:
      switch(robot.moveDirections(maze.findNextMove())){
        case NOMOVES:
          if(robot.status==DANGERZONE) robot.status = BACKTRACKING;
          break;
        case BLACKTILE:
          maze.maze[robot.pos].black = 1;
          robot.pos = nextPoint(robot.pos, (Direction)((robot.facing+2)%4)); // return robot's position
          break;
        case REDTILE:
          maze.maze[robot.pos].red = 1;
          robot.pos = nextPoint(robot.pos, (Direction)((robot.facing+2)%4)); // return robot's position
          break;
        case GOOD:
          maze.updateTile();
          break;
      }
      break;
    case BACKTRACKING:
      stop_motors(); delay(5000);
      robot.moveDirections(maze.findOrigin());
      robot.status = FINISH;
      break;
  }
}

// void setup1(){

// }
// void loop1(){
  
// }