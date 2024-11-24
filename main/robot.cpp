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

void wallTrace(int cm, int speed){
  enc=0;
  double prevErr = 0, dist = 0, err = 0;
  while(encToCm(enc)<cm){
    dist = readTOF(LEFT_TOF) < readTOF(RIGHT_TOF) ? readTOF(LEFT_TOF) : 30 - WIDTH - readTOF(RIGHT_TOF); // dist from left wall
    err = (30-WIDTH)/2 - dist;
    lmotor(speed+err*kP+(err-prevErr)*kD);
    rmotor(speed-err*kP-(err-prevErr)*kD);
    prevErr = err;
  }
}


Robot::Robot(){
  pos.x = 0;
  pos.y = 0;
  facing = NORTH;
  status = TRAVERSING;
}
void Robot::moveRobot(Direction dir){
  facing = dir;
}
void Robot::moveDirections(std::vector<Direction> directions){
  for(Direction d : directions){
    moveRobot(d);
    pos = nextPoint(pos, d);
  }
}

void turn_to(int deg){
  deg %= 360;
  if(deg<360) deg+=360;
  double err = deg-getBNO();
  if(err>180) err-=360;
  if(err<-180) err+=360;
  while(abs(err)>3){
    lmotor(err*TURNKP);
    rmotor(-1*err*TURNKP);
  }
}

void turn(int deg){
  turn_to(deg + getBNO());
}

void turnRounded(int deg){
  turn_to(roundAngle(deg+getBNO()));
}