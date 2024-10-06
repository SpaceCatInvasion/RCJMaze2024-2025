#include "maze.h"

Maze::Maze(Robot* r){
  robot = r;
}
std::vector<Direction> Maze::findNextMove(){
  std::map<Point,Point> paths;
  std::map<Point,bool> bfsVisited;
  bfsVisited[robot->pos] = 1;
  std::queue<Point> q;
  q.push(robot->pos);
  std::vector<Direction> pathBack(0);
  while(!q.empty()){
    Point p = q.front();
    q.pop();
    if(!maze[p].visited){
      for(Point trace = p;!samePoint(trace,robot->pos);trace = paths[trace]){
//        pathBack.push_back()
      }
    }
  }
  return std::vector<Direction>(0);
}