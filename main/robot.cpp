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
double rToA(double r){
  return r*180/PI;
}
double aToR(double a){
  return a*PI/180;
}
double Robot::sideAlignment(){
  double dist=readTOF(LEFT_TOF)+1; //bias - positive = goes closer to wall
  //dist*=dist/3; //exagerates movement - (dist/b)^a decrease a to exagerate less, b is focus of exageration
  dist=dist<0?0.2:dist;
  if(dist < 15 && dist > 0.1){
    double theta = atan(60/(2*dist-30+TOF_WIDTH))>=0?rToA(atan(60/(2*dist-30+TOF_WIDTH))):rToA(atan(60/(2*dist-30+TOF_WIDTH)))+180;
    turn_to(roundAngle(getBNO())-90+theta);
    stop_motors(); delay(10);
    return theta;
  }
  dist=readTOF(RIGHT_TOF)+1; //bias - positive = goes closer to wall
  //dist*=dist/3; //exagerates movement - (dist/b)^a decrease a to exagerate less, b is focus of exageration
  dist=dist<0?0.2:dist;
  if(dist < 15 && dist > 0.1){
    double theta = atan(60/(2*dist-30+TOF_WIDTH))>=0?rToA(atan(60/(2*dist-30+TOF_WIDTH))):rToA(atan(60/(2*dist-30+TOF_WIDTH)))+180;
    turn_to(roundAngle(getBNO())+90-theta);
    stop_motors(); delay(10);
    return theta;
  }
  return 0;
}
ReturnError Robot::robotForward(double cm){
  enc=0;
  while(enc<cmToEnc(cm)){
    forward(60);
  }
  stop_motors(); delay(10);
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
  switch(robotForward(30/sin(aToR(sideAlignment())))){
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

void Robot::turn_to(int deg){
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

void Robot::turn(int deg){
  turn_to(deg + getBNO());
}

void Robot::turnRounded(int deg){
  turn_to(roundAngle(deg+getBNO()));
}