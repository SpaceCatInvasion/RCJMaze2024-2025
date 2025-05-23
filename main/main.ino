#include "maze.h"
#include "Wire.h"
#include "fileIO.h"
Robot robot;
Maze maze(&robot);

#define RAMP_ON
#define CAM_ON
#define OBSTACLE_ON
// #define OLD_BOT
#define NEW_BOT



void setup() {

  //limit switch init
  pinMode(RIGHT_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LEFT_LIMIT_SWITCH_PIN, INPUT_PULLUP);


  Serial.begin(9600);
  Serial1.begin(9600);
  delay(500);
  Serial.println("Serial Connected");
  pinMode(START_BUTTON, INPUT);
  pinMode(CLEAR_BUTTON, INPUT);

  //file io init
  LittleFS.begin();
  Serial.println(" file system ready");

  //buzzer init
  pinMode(BUZZER, OUTPUT);
  tone(BUZZER, 400, 200);
  Serial.println(" buzzer ready");

  while (digitalRead(START_BUTTON) == HIGH) {
    if (digitalRead(CLEAR_BUTTON) != HIGH) {
      clearFile();
      break;
    }
  }
  printMaze(&maze);

  //comm init + start pi code
  commBegin();  //serial init
  Serial.println(" comm ready");

  //enc init
  pinMode(ENC_PIN, INPUT);
  pinMode(ENC_PIN_INTER, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_INTER), enc_update, RISING);

  //i2c init
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.begin();

  Wire1.setSCL(SCL_PIN1);
  Wire1.setSDA(SDA_PIN1);
  Wire1.begin();

  Serial.println(" i2c ready");

  //color sensor init
  colorBegin();
  Serial.println(" color ready");

  //imu init
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  Serial.println(" bno ready");

  //tof init
  tofInit();
  Serial.println(" tof ready");

  //servo init
  servo.attach(SERVO_PIN);
  Serial.println(" servo ready");

  //stepper init
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  Serial.println(" stepper ready");

  //led init
  ledInit();
  Serial.println(" led ready");

  tone(BUZZER, 400, 200);

  downloadMaze(&maze);
  printMaze(&maze);
  robot.print();
  maze.updateTile();
  enc = 0;
  Serial.println("Ready to start");
}


int iter = 0;
void loop() {

  // std::vector<Direction> directions = {NORTH, NORTH, EAST, EAST, SOUTH, SOUTH, WEST, WEST};
  // robot.moveDirections(directions);
  // Serial.println(getBNO());
  //Serial.println(getColor());
  // printTOFs();
  // forward(50);
  //printColorSensorData();


  // forward(100);
  // forward(60); delay(1000);
  // int x = getBNO();
  // while(abs(getBNO) < x + 90) {
  //  lmotors(70);
  //  rmotors(0);
  //  delay(150);
  //  stop_motors();
  //  delay(300);
  //  rmotors(-55);
  //  lmotors(0);
  //  delay(150);
  //  stop_motors();
  //  delay(300);
  // }
  // stop_motors();
  // delay(1000);
  // backward(60); delay(1000);
  //   Serial.println("foward...");
  //   forward(30);
  //   // delay(250);


  // drop(true);
  // drop(false);
  // int temp;
  // while(1){
  //   switch((getColor())){
  //     case WHITE: Serial.println(" WHITE"); break;
  //     case BLACK: Serial.println(" BLACK"); break;
  //     case BLUE: Serial.println(" BLUE"); break;
  //     case RED: Serial.println(" RED"); break;
  //     case SILVER: Serial.println(" SILVER"); break;
  //     case UNK: Serial.println(" UNKNOWN"); break;
  //     // default: Serial.print(" confusion "); Serial.println(temp);
  //   }
  // }



#ifdef CAM_ON
  if (interrupted) {
    interruptFunc();
  }
#endif

  // printTOFs();



  Serial.print("\nAt point ");
  printPoint(robot.pos);
  switch (robot.status) {
    case TRAVERSING:
    case DANGERZONE:
      switch (robot.moveDirections(maze.findNextMove())) {
        case NOMOVES:
          if (robot.status == DANGERZONE) robot.status = BACKTRACKING;
          Serial.println("Can't find moves");
          break;
        case BLACKTILE:
          maze.maze[robot.pos] |= BLACKPOINT;
          robot.pos = nextPoint(robot.pos, (Direction)((robot.facing + 2) % 4));  // return robot's position
          break;
        case REDTILE:
          maze.maze[robot.pos] |= REDPOINT;
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
        case SILVERTILE:
          maze.updateTile();
          Serial.println("\nUploading...");
          printMaze(&maze);
          uploadMaze(&maze);
          break;
        case GOOD:
          maze.updateTile();
          break;
      }
      break;
    case BACKTRACKING:
      Serial.println("Backtracking");
      stop_motors();
      tone(BUZZER, 700, 1000);
      delay(2000);
      robot.moveDirections(maze.findOrigin());
      robot.status = FINISH;
      break;
    case FINISH:
      stop_motors();
      blink(5, 1000);
      robot.status = END;
    case END: break;
  }
  Serial.print("Ended at point ");
  printPoint(robot.pos);



  // printColorSensorData();

  //blink();
  //testForward();
  // delay(175);
  // stop_motors();
  // delay(100);
  // rmotors(75);
  // delay(175);
  // stop_motors();
  // forward(60);
  // delay(20);
  // stop_motors();
  // delay(300);

  // lmotors(100);
  // rmotors(100);

  //printEncs();
}

// void setup1(){

// }
// void loop1(){

// }
