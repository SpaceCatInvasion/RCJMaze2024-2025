#include "color.h"

Adafruit_APDS9960 apds;

void colorBegin(){
  if(!apds.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");
  apds.enableColor(true);
}

/*
 * Find color of tile
 *
 * @param None
 * @return The tile color
 */
TileColor getColor(){
  while(!apds.colorDataReady()) Serial.println("Waiting on color...");
  uint16_t r,g,b,c;
  apds.getColorData(&r, &g, &b, &c);
  Serial.print("red: ");
  Serial.print(r);
  
  Serial.print(" green: ");
  Serial.print(g);
  
  Serial.print(" blue: ");
  Serial.print(b);
  
  Serial.print(" clear: ");
  Serial.print(c);

  if(c<40)
    return BLACK;
  else if (c>500)
    return SILVER;
  else if ((float)b/r>1.8&&(float)b/g>1.2)
    return BLUE;
  else if ((float)r/b>1.5&&(float)r/g>1.5)
    return RED;
  else if (c>100)
    return WHITE;
  else
    return UNK;
}