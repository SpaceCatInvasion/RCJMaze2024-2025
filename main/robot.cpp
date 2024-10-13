#include "robot.h"

Robot::Robot(){
  pos.x = 0;
  pos.y = 0;
  dir = NORTH;
  status = TRAVERSING;
}

void Robot::move(std::vector<Direction> directions){

}

bool samePoint(Point p1, Point p2){
  return (p1.x==p2.x&&p1.y==p2.y);
}
Point nextPoint(Point p, Direction d){
  switch(d){
    Point temp;
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