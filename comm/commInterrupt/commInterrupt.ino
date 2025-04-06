void interFunc(){
  Serial.println("Interrupted!");
  while(!Serial1.available());
  Serial.print((char)Serial1.read());
  while(!Serial1.available());
  Serial.println((char)Serial1.read());
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.setTX(28);
  Serial1.setRX(29);
  Serial1.begin(115200);
  pinMode(7,INPUT);
  attachInterrupt(digitalPinToInterrupt(7), interFunc, RISING);
}
void loop() {

}
