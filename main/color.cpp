#include "color.h"

Adafruit_APDS9960 apds;

void colorBegin() {
  if (!apds.begin()) {
    Serial.println("failed to initialize color sensor! Please check your wiring.");
  } else Serial.println("Device initialized!");
  apds.enableColor(true);
}

/*
 * Find color of tile
 *
 * @param None
 * @return The tile color
 */
TileColor getColor() {
  // while(!apds.colorDataReady()) Serial.println("Waiting on color...");
  uint16_t r, g, b, c;
  apds.getColorData(&r, &g, &b, &c);
  // Serial.print("red: ");
  // Serial.print(r);

  // Serial.print(" green: ");
  // Serial.print(g);

  // Serial.print(" blue: ");
  // Serial.print(b);

  // Serial.print(" clear: ");
  // Serial.println(c);

  // if (c > 220) return SILVER;
  // if (r > 60 && g > 60 && b > 50) return WHITE;
  // if (r > 35&&g<35&&b<35) return RED;
  // if (b > 15) return BLUE;
  // return BLACK;
  if (r > 50 && g > 50 && b > 50) return WHITE;
  if (b > 30) return BLUE;
  return BLACK;
  // return UNK;
}


void printColorSensorData() {
  uint16_t r, g, b, c;
  apds.getColorData(&r, &g, &b, &c);
  Serial.print("red: ");
  Serial.print(r);

  Serial.print(" green: ");
  Serial.print(g);

  Serial.print(" blue: ");
  Serial.print(b);

  Serial.print(" clear: ");
  Serial.println(c);
}