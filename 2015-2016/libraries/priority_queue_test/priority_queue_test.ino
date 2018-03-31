#include "priority_queue.h"

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  priority_queue queue;
  queue.update(0,3);
  queue.update(3,2);
  delay(2000);
  for(int i=0; i<2; i++)
  {
    node popped = queue.pop();
    Serial.print(popped.state);
    Serial.print(", ");
    Serial.print(popped.value);
    Serial.print("\n");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
