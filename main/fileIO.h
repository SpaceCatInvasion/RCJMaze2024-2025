#include <LittleFS.h>
#include "maze.h"

const char* filename = "/maze.txt";

void uploadMaze(Maze m);
void downloadMaze(Maze* m);