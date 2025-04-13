#include "maze.h"
#include "Wire.h"
#include "comm.h"
Robot robot;
Maze maze(&robot);

#define RAMP_ON
#define CAM_OFF
//#define OBSTACLE_ON
// #define OLD_BOT
#define NEW_BOT


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

  Wire1.setSCL(27);
  Wire1.setSDA(26);
  Wire1.begin();

  Serial.println(" i2c ready");

  
  tofInit();
  Serial.println(" tof ready");

  if(!bno.begin(OPERATION_MODE_IMUPLUS)){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  colorBegin();
  commBegin();
  servoBegin();

  maze.updateTile();
  enc=0;
}



void loop(){


  // std::vector<Direction> directions = {NORTH, NORTH, EAST, EAST, SOUTH, SOUTH, WEST, WEST}; 
  // robot.moveDirections(directions);
  Serial.print("At point "); printPoint(robot.pos);
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
        case RAMP:
        {
          Point flatExit = nextPoint(robot.pos, robot.facing, rampTilesForward);
          if(incline) flatExit.z++;
          else flatExit.z--;
          Point finalRamp = nextPoint(flatExit, (Direction)((robot.facing+2)%4));
          maze.rampConnections[robot.pos] = flatExit;
          maze.rampConnections[finalRamp] = nextPoint(robot.pos, (Direction)((robot.facing+2)%4));
          maze.AddRamp(finalRamp,(Direction)((robot.facing+2)%4));
          maze.AddRamp(robot.pos, robot.facing);
          maze.AddWall(robot.pos, (Direction)((robot.facing+1)%4));
          maze.AddWall(robot.pos, (Direction)((robot.facing+3)%4));
          maze.AddWall(finalRamp, (Direction)((robot.facing+1)%4));
          maze.AddWall(finalRamp, (Direction)((robot.facing+3)%4));
          robot.pos = flatExit;
          maze.updateTile();
          break;
        }
        case GOOD:
          maze.updateTile();
          break;
      }
      break;
    case BACKTRACKING:
      Serial.println("Backtracking");
      stop_motors(); delay(5000);
      robot.moveDirections(maze.findOrigin());
      robot.status = FINISH;
      break;
    case FINISH:
      stop_motors();
  }
  Serial.print("Ended at point "); printPoint(robot.pos);

}

// void setup1(){

// }
// void loop1(){
  
// }