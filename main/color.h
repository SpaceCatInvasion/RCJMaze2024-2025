#include "Adafruit_APDS9960.h"

extern Adafruit_APDS9960 apds;

enum TileColor {
  WHITE = 0,
  RED = 1,
  BLACK = 2,
};

void colorBegin();
TileColor getColor();