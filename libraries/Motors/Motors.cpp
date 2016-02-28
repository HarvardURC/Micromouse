
void Motors::setup() {
 Serial.begin(9600);
 pinMode(TICKPIN, INPUT);
 pinMode(SECONDTICKPIN, INPUT);
 attachInterrupt(digitalPinToInterrupt(TICKPIN), onTick, RISING);
 attachInterrupt(digitalPinToInterrupt(SECONDTICKPIN), onTickTwo, RISING);
 pinMode(PHASEPIN, OUTPUT);
 pinMode(SECONDPHASEPIN, OUTPUT);
 analogWrite(DRIVEPIN, 255); 
 analogWrite(DRIVEPINTWO, 255);

}


void Motors::anything(int pwm, int tickDelta) {

  counter1 = 0;
  analogWrite(DRIVEPIN, pwm);
  while (counterL < tickdelta) {
    delay(1);
    }
  analogWrite(DRIVEPIN, 0);
  Serial.println(counterL);
}

void Motors::othermotor(int pwm, int tickDelta) {
  
  counter1 = 0; 
  counter2 = 0;
  
  while (counterR < tickDelta || counterL < tickDelta) {
 
    analogWrite(DRIVEPIN, pwm);
    analogWrite(DRIVEPINTWO, pwm);
      
      if (counterL > counterR) 
      {
         analogWrite(DRIVEPIN, 0);
      }
      else if (counterL < counterR)
      {
        analogWrite(DRIVEPINTWO, 0);
      }
        
 
  }
  analogWrite(DRIVEPIN, 0);
  analogWrite(DRIVEPINTWO, 0);
  
}



void Motors::onTick() {
  
  counterL++;
 
}

void Motors::onTickTwo() {
  counterR++;
}