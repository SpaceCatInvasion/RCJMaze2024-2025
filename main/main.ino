#include "maze.h"
#include "Wire.h"
#include "comm.h"
#include "fileIO.h"
Robot robot;
Maze maze(&robot);

#define RAMP_ON
#define CAM_OFF
//#define OBSTACLE_ON
// #define OLD_BOT
#define NEW_BOT


void setup() {
  Serial.begin(9600);
  delay(2000);
  //while(!Serial);
  Serial.println("Serial Connected");
  pinMode(20, INPUT);
  while (digitalRead(20) == HIGH)
    ;


  pinMode(ENC_PIN, INPUT);
  pinMode(ENC_PIN_INTER, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_INTER), enc_update, RISING);

  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.begin();

  Wire1.setSCL(SCL_PIN1);
  Wire1.setSDA(SDA_PIN1);
  Wire1.begin();

  Serial.println(" i2c ready");

  colorBegin();
  Serial.println(" color ready");

  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  tofInit();
  Serial.println(" tof ready");

  commBegin();
  Serial.println(" comm ready");
  servoBegin();
  Serial.println(" servo ready");
  pinMode(LED_PIN, OUTPUT);
  Serial.println(" led ready");

  LittleFS.begin();
  Serial.println(" file system ready");

  maze.updateTile();
  enc = 0;
  Serial.println("Ready to start");
}



void loop() {


  // std::vector<Direction> directions = {NORTH, NORTH, EAST, EAST, SOUTH, SOUTH, WEST, WEST};
  // robot.moveDirections(directions);
  // Serial.println(getBNO());
  // getColor();
  // printTOFs();
  // forward(50);
  


  // forward(100);
  // forward(60); delay(1000);
  // lmotors(30); delay(1000);
  // rmotors(30); delay(1000);
  // backward(60); delay(1000);

  

  Serial.print("At point ");
  printPoint(robot.pos);
  switch (robot.status) {
    case TRAVERSING:
    case DANGERZONE:
      switch (robot.moveDirections(maze.findNextMove())) {
        case NOMOVES:
          if (robot.status == DANGERZONE) robot.status = BACKTRACKING;
          break;
        case BLACKTILE:
          maze.maze[robot.pos] |= BLACK;
          robot.pos = nextPoint(robot.pos, (Direction)((robot.facing + 2) % 4));  // return robot's position
          break;
        case REDTILE:
          maze.maze[robot.pos] |= RED;
          robot.pos = nextPoint(robot.pos, (Direction)((robot.facing + 2) % 4));  // return robot's position
          break;
        case RAMP:
          {
            Point flatExit = nextPoint(robot.pos, robot.facing, rampTilesForward);
            if (incline) flatExit.z++;
            else flatExit.z--;
            Point finalRamp = nextPoint(flatExit, (Direction)((robot.facing + 2) % 4));
            maze.rampConnections[robot.pos] = flatExit;
            maze.rampConnections[finalRamp] = nextPoint(robot.pos, (Direction)((robot.facing + 2) % 4));
            maze.AddRamp(finalRamp, (Direction)((robot.facing + 2) % 4));
            maze.AddRamp(robot.pos, robot.facing);
            maze.AddWall(robot.pos, (Direction)((robot.facing + 1) % 4));
            maze.AddWall(robot.pos, (Direction)((robot.facing + 3) % 4));
            maze.AddWall(finalRamp, (Direction)((robot.facing + 1) % 4));
            maze.AddWall(finalRamp, (Direction)((robot.facing + 3) % 4));
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
      stop_motors();
      delay(5000);
      robot.moveDirections(maze.findOrigin());
      robot.status = FINISH;
      break;
    case FINISH:
      stop_motors();
  }
  Serial.print("Ended at point ");
  printPoint(robot.pos);
}

// void setup1(){

// }
// void loop1(){

// }