#include <map>
#include <queue>
#include "robot.h"

struct Tile {
  bool NWall : 1;
  bool SWall : 1;
  bool EWall : 1;
  bool WWall : 1;
  bool NVic : 1;
  bool SVic : 1;
  bool EVic : 1;
  bool WVic : 1;
  bool visited : 1;
  bool blue : 1;
  bool black : 1;
  bool NRamp : 1; //meaning robot faces north when entering ramp
  bool SRamp : 1;
  bool ERamp : 1;
  bool WRamp : 1;
  bool filler : 1;
};

class Maze {
private:

public:
  std::map<Point,Tile> maze;
  std::map<Point,Tile> floor2;
  Robot* robot;
  Maze(Robot* r);
  std::vector<Direction> findNextMove();
};