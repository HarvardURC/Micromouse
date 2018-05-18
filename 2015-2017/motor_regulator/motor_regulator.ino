#define TICKPIN 3
#define DRIVEPIN 9
#define TICKSPERSEC 400 // set speed here
#define MEGA 1000000 // microseconds in a second

volatile short stepCounter;
long dt;
long prevTime;
float powCoeff = 0.68 - (((float)TICKSPERSEC) / 2000);

void setup() {
  Serial.begin(9600);
  pinMode(TICKPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(TICKPIN), onTick, RISING);
  analogWrite(DRIVEPIN, powCoeff * TICKSPERSEC);
  prevTime = micros();
}

void loop() {
  stepCounter = 0;

  delay(100);
  
  Serial.print(stepCounter);
  Serial.print("\t");
  
  dt = -prevTime;
  prevTime = micros();
  dt += prevTime;
  Serial.print(dt);
  Serial.print("\t");

  powCoeff *= TICKSPERSEC * dt;
  powCoeff /= stepCounter * MEGA;
  if (powCoeff * TICKSPERSEC > 255)
  {
    powCoeff = (float)255 / TICKSPERSEC;
  }
  Serial.print(powCoeff, 4);
  Serial.print("\n");

  analogWrite(DRIVEPIN, powCoeff * TICKSPERSEC);
}

void onTick() {
  stepCounter++;
}

