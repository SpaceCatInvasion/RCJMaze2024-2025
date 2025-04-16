#include "fileIO.h"

void uploadMaze(Maze m) {
  File outfile = LittleFS.open(filename, "w");
  if (!outfile) {                         // if failed - maybe forgot to set the Filesystem in IDE
    Serial.println("fail open outfile");  // or didn't call LittleFS.begin()
    while (1)
      ;
  }
  Serial.println("Writing to file...");
  // print robot
  char robot[5];  // x,y,z,facing,status
  robot[0] = (char)(m.robot->pos.x);
  robot[1] = (char)(m.robot->pos.y);
  robot[2] = (char)(m.robot->pos.z);

  robot[3] = (char)(m.robot->facing);
  robot[4] = (char)(m.robot->status);
  outfile.write(robot, strlen(robot));
  for (std::pair<Point, Tile> tiles : m.maze) {  // print maze
    char ch[5];                                  // x,y,z,Tile high byte,Tile low byte
    ch[0] = (char)(tiles.first.x);
    ch[1] = (char)(tiles.first.y);
    ch[2] = (char)(tiles.first.z);

    ch[3] = (char)(tiles.second >> 8);
    ch[4] = (char)(tiles.second);
    outfile.write(ch, strlen(ch));
  }
  outfile.close();
}

void downloadMaze(Maze *m) {
  File infile = LittleFS.open(filename, "r");
  if (!infile) {  // failed - file not there or forgot to create it
    return;
  }
  Serial.println("Reading from file...");
  if (!infile.available()) return;
}