#include "color.h"

Adafruit_APDS9960 apds;

void colorBegin(){
  if(!apds.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");
  apds.enableColor(true);
}

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
  Serial.println(c);

  if(r<5&&g<5&&b<5) //placeholder
    return BLACK;
  else if (r>40&&g>40&&b>40)
    return WHITE;
  else
    return RED;
}