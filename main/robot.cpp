#include "robot.h"

bool samePoint(Point p1, Point p2){
  return (p1.x==p2.x&&p1.y==p2.y);
}
Point nextPoint(Point p, Direction d){
  Point temp;
  switch(d){  
    case NORTH:
      temp.x=p.x;
      temp.y=p.y+1;
      return temp;
    case SOUTH:
      temp.x=p.x;
      temp.y=p.y-1;
      return temp;
    case EAST:
      temp.x=p.x+1;
      temp.y=p.y;
      return temp;
    case WEST:
      temp.x=p.x-1;
      temp.y=p.y;
      return temp;
  }
  Point err;
  err.x=-100;
  err.y=-100;
  return err;
}

int directionAngle(Direction d){
  switch(d){
    case NORTH:
      return 0;
    case SOUTH:
      return 180;
    case EAST:
      return 90;
    case WEST:
      return 270;
  }
  return -1;
}

// Don't pay attention to this function it doesn't work well
ReturnError Robot::wallTrace(int cm, int speed){
  enc=0;
  double prevErr = 0, dist = 0, err = 0;
  double leftDist, rightDist;
  while(encToCm(enc)<cm){
    leftDist = readTOF(LEFT_TOF);
    rightDist = readTOF(RIGHT_TOF);
    Serial.print("Left: "); Serial.print(leftDist); Serial.print("; Right: "); Serial.print(rightDist);
    if(leftDist<15)
      dist=leftDist;
    else if(rightDist<15)
      dist=30-WIDTH-rightDist;
    else{
      forward(speed);
      continue;
    }
    
    err = (30-WIDTH)/2 - dist;
    lmotors(speed+err*kP+(err-prevErr)*kD);
    rmotors(speed-err*kP-(err-prevErr)*kD);
    prevErr = err;
  }
  return GOOD;
}

// Radians to Angle
double rToA(double r){
  return r*180/PI;
}
// Angle to Radians
double aToR(double a){
  return a*PI/180;
}

/*
 * Find angle to turn to in order to be aligned with the next tile
 *
 * @param None
 * @return Acute angle from the y-axis
 */
double Robot::sideAlignment(){
  double dist=readTOF(LEFT_TOF)+2; //bias - positive = goes closer to wall
  //dist*=dist/3; //exagerates movement - (dist/b)^a decrease a to exagerate less, b is focus of exageration
  dist=dist<0?0.2:dist;
  if(dist < 15 && dist > 0.1){
    double theta = atan(60/(2*dist-30+TOF_WIDTH))>=0?rToA(atan(60/(2*dist-30+TOF_WIDTH))):rToA(atan(60/(2*dist-30+TOF_WIDTH)))+180; // math
    turn_to(roundAngle(getBNO())-90+theta);
    stop_motors(); delay(10);
    return theta;
  }
  dist=readTOF(RIGHT_TOF)+1; //bias - positive = goes closer to wall
  //dist*=dist/3; //exagerates movement - (dist/b)^a decrease a to exagerate less, b is focus of exageration
  dist=dist<0?0.2:dist;
  if(dist < 15 && dist > 0.1){
    double theta = atan(60/(2*dist-30+TOF_WIDTH))>=0?rToA(atan(60/(2*dist-30+TOF_WIDTH))):rToA(atan(60/(2*dist-30+TOF_WIDTH)))+180; //math
    turn_to(roundAngle(getBNO())+90-theta);
    stop_motors(); delay(10);
    return theta;
  }
  return 90;
}

/*
 * Move the robot forward while checking color sensor
 *
 * @param None
 * @return Whether or not the robot finds a special case (e.g. ramp, black tile)
 */
ReturnError Robot::robotForward(double cm){
  Serial.print("CM:");Serial.println(cm);
  enc=0;
  while(enc<cmToEnc(cm)){
    if(checkSerial()){ // camera check - replace later with interrupt
      clearSerial();
      stop_motors(); delay(500);
      if(checkSerial()) {
        drop(readVictim());
      }
    }

    // Add color check
    forward(30);
  }
  stop_motors(); delay(10);
  return GOOD;
}

/*
 * Align the robot with the front TOF
 *
 * @param None
 * @return None
 */
void Robot::frontAlign(){
  int dist = readTOF(FRONT_TOF);
  while(dist<15&&dist>(30-FRONTBACKTOF)/2){
    forward(20);
    dist=readTOF(FRONT_TOF);
  }
  while(dist<15&&dist<(30-FRONTBACKTOF)/2){
    backward(20);
    dist=readTOF(FRONT_TOF);
  }
}
/*
 * Align the robot with the back TOF
 *
 * @param None
 * @return None
 */
void Robot::backAlign(){
  int dist = readTOF(BACK_TOF);
  while(dist<15&&dist>(30-FRONTBACKTOF)/2){
    backward(20);
    dist=readTOF(BACK_TOF);
  }
  while(dist<15&&dist<(30-FRONTBACKTOF)/2){
    forward(20);
    dist=readTOF(BACK_TOF);
  }
}


Robot::Robot(){
  pos.x = 0;
  pos.y = 0;
  facing = NORTH;
  status = TRAVERSING;
}

/*
 * Move the robot in the direction dir
 *
 * @param Direction dir to move towards
 * @return None
 */
void Robot::moveRobot(Direction dir){
  facing = dir;
  stop_motors(); delay(200);
  turn_to(directionAngle(dir));
  stop_motors(); delay(1000);
  frontAlign();
  backAlign();
  turn_to(directionAngle(dir));
  stop_motors(); delay(1000);
  switch(robotForward(TILE_MOVE_DIST/sin(aToR(sideAlignment())))){
    case RAMP:
      break;
    case GOOD:
      stop_motors(); delay(200);
      turn_to(directionAngle(dir));
      stop_motors(); delay(1000);
      frontAlign();
      backAlign();
      break;
  }
}
/*
 * Move robot on the path denoted by the vector directions
 *
 * @param Vector of the directions to move in
 * @return If there was somewhere to move to
 */
bool Robot::moveDirections(std::vector<Direction> directions){
  if(directions.empty()) return false;
  for(Direction d : directions){
    moveRobot(d);
    stop_motors(); delay(1000);
    pos = nextPoint(pos, d);
  }
  return true;
}

/*
 * PID turn the robot to specified degrees clockwise from North
 *
 * @param Degrees clockwise from North to turn to
 * @return None
 */
void Robot::turn_to(int deg){
  deg %= 360;
  if(deg<0) deg+=360;
  double err = deg-getBNO();
  if(err>180) err-=360;
  if(err<-180) err+=360;
  while(err>2||err<-2){
    lmotors(err*TURNKP+(err>0?BASE_TURN_SPEED:-BASE_TURN_SPEED));
    rmotors(-1*err*TURNKP+(err>0?-BASE_TURN_SPEED:BASE_TURN_SPEED));
    err = deg-getBNO();
    if(err>180) err-=360;
    if(err<-180) err+=360;
    //Serial.print("Error: "); Serial.println(err);
  }
  //Serial.println("DONE");
}

/*
 * Turn specified degrees clockwise from current angle
 *
 * @param Degrees to turn clockwise
 * @return None
 */
void Robot::turn(int deg){
  turn_to(deg + getBNO());
}