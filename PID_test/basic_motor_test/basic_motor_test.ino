void setup() {
  // put your setup code here, to run once:
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(11,0);
  analogWrite(10,255);
}
