#define TICKPIN 7
#define DRIVEPIN 3
#define PHASEPIN 9
#define SECONDTICKPIN 2
#define DRIVEPINTWO 10
#define SECONDPHASEPIN A2


void anything(int pwm, int tickdelta);

volatile int counter1 = 0;
volatile int counter2 = 0;
int x = 0;

void setup() {
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

void loop() {
  /*
  digitalWrite(PHASEPIN, HIGH);
  analogWrite(DRIVEPIN, 255-x);
  delay(5000);
  //digitalWrite(PHASEPIN, LOW);
 // delay(1000);
 Serial.print(255-x);
 Serial.print("\t");
 Serial.println(counter);
 counter = 0;
 x+=10;
 */
 othermotor(255, 3000);
 
 delay(1000);

 
  
}

void anything(int pwm, int tickdelta) {

  counter1 = 0;
  analogWrite(DRIVEPIN, pwm);
  while (counter1 < tickdelta) {
    delay(1);
    }
  analogWrite(DRIVEPIN, 0);
  Serial.println(counter1);
}

void othermotor(int pwm, int tickdelta) {
  
  counter1 = 0; 
  counter2 = 0;
  
  while (counter2 < tickdelta || counter1 < tickdelta) {
 
    analogWrite(DRIVEPIN, pwm);
    analogWrite(DRIVEPINTWO, pwm);
      
      if (counter1 > counter2) 
      {
         analogWrite(DRIVEPIN, 0);
      }
      else if (counter1 < counter2)
      {
        analogWrite(DRIVEPINTWO, 0);
      }
        
 
  }
  analogWrite(DRIVEPIN, 0);
  analogWrite(DRIVEPINTWO, 0);
  
}





void onTick(){
  
  counter1++;
 
}

void onTickTwo(){
  counter2++;
}

