#pragma once
#include <vector>
struct Point {
  int x;
  int y;
};
struct PointCmp {
  bool operator()(const Point &lhs, const Point &rhs) const {
    return lhs.x>lhs.y;
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
public:
  Point pos;
  Direction dir;
  Status status;
  Robot();
  void move(std::vector<Direction> directions);
};
bool samePoint(Point p1, Point p2);
Point nextPoint(Point p, Direction d);