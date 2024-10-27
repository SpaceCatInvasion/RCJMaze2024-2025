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

