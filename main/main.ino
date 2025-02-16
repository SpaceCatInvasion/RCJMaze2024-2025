#include "maze.h"
#include "Wire.h"
Robot robot;
Maze maze(&robot);




void setup(){
  enc=0;
  Serial.begin(115200);
  delay(2000);
  while(!Serial);
  Serial.println("Serial Connected");
  pinMode(ENC_PIN,INPUT);
  pinMode(ENC_PIN_INTER,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_INTER), enc_update, RISING);
  Wire.setSCL(17);
  Wire.setSDA(16);
  Wire.begin();
  
  tofInit();

  Wire1.setSCL(27);
  Wire1.setSDA(26);
  Wire1.begin();

//   Serial.println("Serial Connected");

  if(!bnoInit()){
 // if(!bno.begin(OPERATION_MODE_IMUPLUS)){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
//   else {
//     Serial.println("HI");
//   }
  colorBegin();

  maze.updateTile();

}
int i=0;
int iter=1;
void loop(){
  //Serial.print("Angle: "); Serial.print(getBNO()); Serial.print("; Tilt: "); Serial.print(getTilt()); Serial.print("; Rounded: "); Serial.println(roundAngle(getBNO()));
  // Serial.print("Front: "); Serial.print(readTOF(FRONT_TOF)); 
  // Serial.print("; Back: "); Serial.print(readTOF(BACK_TOF)); 
  // Serial.print("; Left: "); Serial.print(readTOF(LEFT_TOF));
  // Serial.print("; LeftB: "); Serial.print(readTOF(LEFTB_TOF));
  // Serial.print("; Right: "); Serial.print(readTOF(RIGHT_TOF));
  // Serial.print("; RightB: "); Serial.println(readTOF(RIGHTB_TOF));
  // wallTrace(30,40);
  // stop();
  // delay(2500);

  // for(int i=0;i<4;i++){
  //   turn_to(90*i);
  //   stop();
  //   delay(1000);
  // }

  // m.speed(i);
  // delay(50);
  // i+=iter;
  // if(i>=100)iter=-1;
  // if(i<=-100)iter=1;
  // Serial.print("Encoders: "); Serial.println(enc);




  
  // switch(robot.status){
  //   case TRAVERSING:
  //     robot.moveDirections(maze.findNextMove());
  //     maze.updateTile();
  // }
  // Color c;
  // getColor(&c);
  // if(c.r==-1) Serial.println("Sadge...");
  // else{
  // Serial.print("red: ");
  // Serial.print(c.r);
  
  // Serial.print(" green: ");
  // Serial.print(c.g);
  
  // Serial.print(" blue: ");
  // Serial.print(c.b);
  
  // Serial.print(" clear: ");
  // Serial.println(c.c);
  // }

  // while(!apds.colorDataReady()){
  //   delay(5);
  // }

  //get the data and print the different channels
  // uint16_t r,g,b,c;
  // apds.getColorData(&r, &g, &b, &c);
  // Serial.print("red: ");
  // Serial.print(r);
  
  // Serial.print(" green: ");
  // Serial.print(g);
  
  // Serial.print(" blue: ");
  // Serial.print(b);
  
  // Serial.print(" clear: ");
  // Serial.println(c);
 
}
// void setup1(){

// }
// void loop1(){
  
// }