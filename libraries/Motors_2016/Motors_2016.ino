#include "Motors_2016.h"

void setup() {
  Serial.begin(9600);
  delay(5000);
  // initialize motors object
  Serial.print("trying...");
  Motors_2016 motors (9, 8, 10,
                      11, 14, 15,
                      16, 17, 
                      1, 1, 100, 1, 1);
  Serial.print("object initialized");
  // drive forward
  motors.forward();
  // turn right
  //motors.turnRight();
  // align with front wall
  //motors.front_align();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("looping\n");
  delay(500);
}
