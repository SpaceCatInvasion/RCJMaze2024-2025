#include "maze.h"
Robot robot;
Maze maze(&robot);
void setup(){
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial Connected");
}
std::vector<Direction> nextMove;
void loop(){
  switch(robot.status){
    case TRAVERSING:
      //Serial.println("Traversing");
      nextMove = maze.findNextMove();
      robot.moveDirections(nextMove);
      break;
    case BACKTRACKING:
      Serial.println("Backtracking");
      break;
    case FINISH:
      Serial.println("Finishing");
      robot.status = END;
    case END:
      break;
  }
}
void setup1(){

}
void loop1(){
  
}