#define TICKPIN 19
#define DRIVEPIN 3
#define PHASEPIN 9

volatile int counter = 0;

void setup() {
 Serial.begin(9600);
 pinMode(TICKPIN, INPUT);
 attachInterrupt(digitalPinToInterrupt(TICKPIN), onTick, RISING);
 pinMode(PHASEPIN, OUTPUT);
 analogWrite(DRIVEPIN, 255); 

}

void loop() {
  digitalWrite(PHASEPIN, HIGH);
  delay(1000);
  digitalWrite(PHASEPIN, LOW);
  delay(1000);
  Serial.print(counter);
}

void onTick(){
  counter++;
 
}

