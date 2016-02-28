
 Motors::Motors(int drivepinL, int drivepinR, int tickpinL, 
           int tickpinR, int phasepinL, int phasepinR) {
 
 pinMode(tickpinL, INPUT);
 pinMode(tickpinR, INPUT);
 attachInterrupt(digitalPinToInterrupt(tickpinL), onTick, RISING);
 attachInterrupt(digitalPinToInterrupt(tickpinR), onTickTwo, RISING);
 pinMode(phasepinL, OUTPUT);
 pinMode(phasepinR, OUTPUT); 

 _drivepinL = drivepinL;
 _drivepinR = drivepinR;
 _phasepinL = phasepinL;
 _phasepinR = phasepinR;

}


void Motors::anything(int pwm, int tickDelta) {

  counter1 = 0;
  analogWrite(_drivepinL, pwm);
  while (counterL < tickdelta) {
    delay(1);
    }
  analogWrite(L, 0);
  Serial.println(counterL);
}

void Motors::othermotor(int pwm, int tickDelta) {
  
  counter1 = 0; 
  counter2 = 0;
  
  while (counterR < tickDelta || counterL < tickDelta) {
 
    analogWrite(_drivepinL, pwm);
    analogWrite(_drivepinR, pwm);
      
      if (counterL > counterR) 
      {
         analogWrite(_drivepinL, 0);
      }
      else if (counterL < counterR)
      {
        analogWrite(_drivepinR, 0);
      }
        
 
  }
  analogWrite(_drivepinL, 0);
  analogWrite(_drivepinR, 0);
  
}



void Motors::onTick() {
  
  counterL++;
 
}

void Motors::onTickTwo() {
  counterR++;
}