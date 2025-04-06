
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.setTX(28);
  Serial1.setRX(29);
  Serial1.begin(115200);
}
int iter = 0;
void loop() {
  // // put your main code here, to run repeatedly:
  if(Serial1.available()){
    Serial.print((char)Serial1.read()); 
    if(iter++&1)Serial.println();
  }
  // else{
  //   Serial.println("waiting...");
  //   delay(1000);
  // }
}
