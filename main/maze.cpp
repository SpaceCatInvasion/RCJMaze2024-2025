#include "maze.h"

/*
 * Return if there is a wall in between the tile at point p and tile in direction d
 *
 * @param Point p
 * @return Vector of the cardinal directions needed to get to the next unvisited tile
 */
bool Maze::hasWall(Point p, Direction d){
  switch(d){
    case NORTH:
      return maze[p].NWall||maze[nextPoint(p,d)].SWall;
    case SOUTH:
      return maze[p].SWall||maze[nextPoint(p,d)].NWall;
    case EAST:
      return maze[p].EWall||maze[nextPoint(p,d)].WWall;
    case WEST:
      return maze[p].WWall||maze[nextPoint(p,d)].EWall;
  }
  return 1;
}
Maze::Maze(Robot* r){
  robot = r;
}

/*
 * Find the next closest unvisited tile and return the directions to it
 *
 * @param None
 * @return Vector of the cardinal directions needed to get to the next unvisited tile
 */
std::vector<Direction> Maze::findNextMove(){
  std::map<Point,Point,PointCmp> paths; // Paths back to original position from any point
  std::map<Point,bool,PointCmp> bfsVisited;
  bfsVisited[robot->pos] = 1;
  std::queue<Point> q; // BFS queue
  q.push(robot->pos);
  std::vector<Direction> pathBack(0); // Returned path
  while(!q.empty()){
    Point p = q.front();
    q.pop();
    if(!maze[p].visited){ // Found closest unvisited tile - find path back
      Point prev = p;
      Point trace = paths[p];
      std::stack<Direction> s;
      for(;;){
        if(prev.y>trace.y) s.push(NORTH);
        else if(prev.y<trace.y) s.push(SOUTH);
        else if(prev.x>trace.x) s.push(EAST);
        else if(prev.x<trace.x) s.push(WEST);
        if(samePoint(trace,robot->pos)) break;
        prev = trace;
        trace = paths[trace];
      }
      while(!s.empty()){
        pathBack.push_back(s.top());
        s.pop();
      }
      break;
    }
    Point next = nextPoint(p,robot->facing);
    if(!hasWall(p,robot->facing)&&!bfsVisited[next]){ // Check if the tile infront is valid
      if(!(robot->status==TRAVERSING&&maze[p].red)){ // Check if tile is in dangerzone
        q.push(next);
        if(paths.count(next)==0) paths[next] = p;
        bfsVisited[next] = 1;
      }
    } 
    next = nextPoint(p,(Direction)((robot->facing+1)%4));
    if(!hasWall(p,(Direction)((robot->facing+1)%4))&&!bfsVisited[next]) {
      if(!(robot->status==TRAVERSING&&maze[p].red)){
        q.push(next);
        if(paths.count(next)==0) paths[next] = p;
        bfsVisited[next] = 1;
      }
    }
    next = nextPoint(p,(Direction)((robot->facing+3)%4));
    if(!hasWall(p,(Direction)((robot->facing+3)%4))&&!bfsVisited[next]) {
      if(!(robot->status==TRAVERSING&&maze[p].red)){
        q.push(next);
        if(paths.count(next)==0) paths[next] = p;
        bfsVisited[next] = 1;
      }
    }
    next = nextPoint(p,(Direction)((robot->facing+2)%4));
    if(!hasWall(p,(Direction)((robot->facing+2)%4))&&!bfsVisited[next]) {
      if(!(robot->status==TRAVERSING&&maze[p].red)){
        q.push(next);
        if(paths.count(next)==0) paths[next] = p;
        bfsVisited[next] = 1;
      }
    }
  }
  if(pathBack.empty()) {
    if(robot->status==TRAVERSING){
      robot->status=DANGERZONE; // No more tiles outside danger zone - start traversing danger zone
      return findNextMove(); 
    }
    else
      robot->status=BACKTRACKING; // No more tiles to traverse in entire maze - start backtracking
  }
  return pathBack;
}

/*
 * Update the walls of the current tile the robot is on
 *
 * @param None
 * @return None
 */
void Maze::updateTile(){
  bool wallN, wallS, wallE, wallW;
  switch(robot->facing){
    case NORTH:
      wallN = readTOF(FRONT_TOF)<MIN_DIST;
      wallS = readTOF(BACK_TOF)<MIN_DIST;
      wallE = readTOF(RIGHT_TOF)<MIN_DIST;
      wallW = readTOF(LEFT_TOF)<MIN_DIST;
      break;
    case SOUTH:
      wallS = readTOF(FRONT_TOF)<MIN_DIST;
      wallN = readTOF(BACK_TOF)<MIN_DIST;
      wallW = readTOF(RIGHT_TOF)<MIN_DIST;
      wallE = readTOF(LEFT_TOF)<MIN_DIST;
      break;
    case EAST:
      wallE = readTOF(FRONT_TOF)<MIN_DIST;
      wallW = readTOF(BACK_TOF)<MIN_DIST;
      wallS = readTOF(RIGHT_TOF)<MIN_DIST;
      wallN = readTOF(LEFT_TOF)<MIN_DIST;
      break;
    case WEST:
      wallW = readTOF(FRONT_TOF)<MIN_DIST;
      wallE = readTOF(BACK_TOF)<MIN_DIST;
      wallN = readTOF(RIGHT_TOF)<MIN_DIST;
      wallS = readTOF(LEFT_TOF)<MIN_DIST;
      break;
  }
  if(wallN){
    maze[robot->pos].NWall=1;
    maze[nextPoint(robot->pos,NORTH)].SWall=1;
  }
  if(wallS){
    maze[robot->pos].SWall=1;
    maze[nextPoint(robot->pos,SOUTH)].NWall=1;
  }
  if(wallW){
    maze[robot->pos].WWall=1;
    maze[nextPoint(robot->pos,WEST)].EWall=1;
  }
  if(wallE){
    maze[robot->pos].EWall=1;
    maze[nextPoint(robot->pos,EAST)].WWall=1;
  }
  maze[robot->pos].visited=1;
}