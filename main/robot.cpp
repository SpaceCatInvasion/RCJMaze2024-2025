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