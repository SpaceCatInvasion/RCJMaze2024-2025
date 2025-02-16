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


Robot::Robot(){
  pos.x = 0;
  pos.y = 0;
  facing = NORTH;
  status = TRAVERSING;
}
void Robot::moveRobot(Direction dir){
  facing = dir;
  turn_to(directionAngle(dir));
  switch(wallTrace(30,80)){
    case RAMP:
      break;
  }
}
void Robot::moveDirections(std::vector<Direction> directions){
  for(Direction d : directions){
    moveRobot(d);
    pos = nextPoint(pos, d);
  }
}

void turn_to(int deg){
  deg %= 360;
  if(deg<0) deg+=360;
  double err = deg-getBNO();
  if(err>180) err-=360;
  if(err<-180) err+=360;
  while(err>3||err<-3){
    lmotors(err*TURNKP+(err>0?BASE_TURN_SPEED:-BASE_TURN_SPEED));
    rmotors(-1*err*TURNKP+(err>0?-BASE_TURN_SPEED:BASE_TURN_SPEED));
    err = deg-getBNO();
    if(err>180) err-=360;
    if(err<-180) err+=360;
    Serial.print("Error: "); Serial.println(err);
  }
  Serial.println("DONE");
}

void turn(int deg){
  turn_to(deg + getBNO());
}

void turnRounded(int deg){
  turn_to(roundAngle(deg+getBNO()));
}