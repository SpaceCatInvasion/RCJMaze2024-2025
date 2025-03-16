#include <map>
#include <queue>
#include <stack>
#include "robot.h"

#define MIN_DIST 15

struct Tile {
  bool NWall : 1;
  bool SWall : 1;
  bool EWall : 1;
  bool WWall : 1;
  bool visited : 1;
  bool blue : 1;
  bool black : 1;
  bool red : 1;
  bool NRamp : 1; //meaning robot faces north when going up ramp
  bool SRamp : 1;
  bool ERamp : 1;
  bool WRamp : 1;
  bool filler : 4;
};

class Maze {
private:
  bool hasWall(Point p, Direction d);
  bool hasRamp(Point p, Direction d);
public:
  std::map<Point,Tile,PointCmp> maze;
  std::map<Point,Point,PointCmp> rampConnections; // connects first ramp tile to exit flat tile
  Robot* robot;
  Maze(Robot* r);
  std::vector<Direction> findNextMove();
  std::vector<Direction> findOrigin();
  void updateTile(); 
};
void clearTile(Tile*t);