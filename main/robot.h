#pragma once
#include <vector>
#include "bno.h"
#include "tof.h"
#include "motor.h"
#include <math.h>
#include "color.h"

#define WIDTH 15.6
#define ENC_PER_ROT 220
#define WHEELDIA 6
#define TOF_WIDTH 17.3

struct Point {
  int x;
  int y;
};
struct PointCmp {
  bool operator()(const Point &lhs, const Point &rhs) const {
    return lhs.x == rhs.x ? lhs.y > rhs.y : lhs.x > rhs.x;
  }
};
enum Direction {
  NORTH = 0,
  SOUTH = 2,
  EAST = 1,
  WEST = 3
};
enum Status {
  TRAVERSING = 0,
  DANGERZONE = 1,
  BACKTRACKING = 2,
  FINISH = 3,
  END = 4
};
enum ReturnError {
  GOOD = 0,
  BLACKTILE = 1,
  REDTILE = 2,
  RAMP = 3,
};

#define BASE_TURN_SPEED 25
#define TURNKP 0.5

class Robot {
private:
  
public:
  Point pos;
  Direction facing;
  Status status;
  Robot();
  void moveDirections(std::vector<Direction> directions);
  void moveRobot(Direction dir);
  double sideAlignment();
  ReturnError robotForward(double cm);
  ReturnError wallTrace(int cm, int speed);
  void turn_to(int deg);
  void turn(int deg);
  void turnRounded(int deg);
};
bool samePoint(Point p1, Point p2);
Point nextPoint(Point p, Direction d);
int directionAngle(Direction d);

void wallTrace(int cm, int speed);


