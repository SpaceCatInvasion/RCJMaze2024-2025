#pragma once
#include <vector>
#include "bno.h"
#include "tof.h"
#include "motor.h"
#include <math.h>

#define WIDTH 15.2
#define ENC_PER_ROT 580
#define WHEELDIA 7.25

struct Point {
  int x;
  int y;
};
struct PointCmp {
  bool operator()(const Point &lhs, const Point &rhs) const {
    return lhs.x==rhs.x?lhs.y>rhs.y:lhs.x>rhs.x;
  }
};
enum Direction {
  NORTH=0,
  SOUTH=2,
  EAST=1,
  WEST=3
};
enum Status {
  TRAVERSING=0,
  BACKTRACKING=1,
  FINISH=2,
  END=3
};

class Robot {
private:
  void moveRobot(Direction dir);
public:
  Point pos;
  Direction facing;
  Status status;
  Robot();
  void moveDirections(std::vector<Direction> directions);
};
bool samePoint(Point p1, Point p2);
Point nextPoint(Point p, Direction d);

void wallTrace(int cm, int speed);

#define TURNKP 2
void turn_to(int deg);
void turn(int deg);
void turnRounded(int deg);