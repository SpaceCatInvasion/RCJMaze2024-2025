#include "maze.h"

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
  //r->m=self;
}
std::vector<Direction> Maze::findNextMove(){
  std::map<Point,Point,PointCmp> paths;
  std::map<Point,bool,PointCmp> bfsVisited;
  bfsVisited[robot->pos] = 1;
  std::queue<Point> q;
  q.push(robot->pos);
  std::vector<Direction> pathBack(0);
  while(!q.empty()){
    Point p = q.front();
    q.pop();
    if(!maze[p].visited){ //find path back
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
    if(!hasWall(p,robot->facing)&&!bfsVisited[next]){
      q.push(next);
      if(paths.count(next)==0) paths[next] = p;
      bfsVisited[next] = 1;
    } 
    next = nextPoint(p,(Direction)((robot->facing+1)%4));
    if(!hasWall(p,(Direction)((robot->facing+1)%4))&&!bfsVisited[next]) {
      q.push(next);
      if(paths.count(next)==0) paths[next] = p;
      bfsVisited[next] = 1;
    }
    next = nextPoint(p,(Direction)((robot->facing+3)%4));
    if(!hasWall(p,(Direction)((robot->facing+3)%4))&&!bfsVisited[next]) {
      q.push(next);
      if(paths.count(next)==0) paths[next] = p;
      bfsVisited[next] = 1;
    }
    next = nextPoint(p,(Direction)((robot->facing+2)%4));
    if(!hasWall(p,(Direction)((robot->facing+2)%4))&&!bfsVisited[next]) {
      q.push(next);
      if(paths.count(next)==0) paths[next] = p;
      bfsVisited[next] = 1;
    }
  }
  if(pathBack.empty()) {
    robot->status=BACKTRACKING;
  }
  return pathBack;
}
void Maze::updateTile(){
  
}