#include "config.h"
#include "io.hh"
#include "motors.hh"
#include "sensors.hh"
#include "bluetooth.hh"

using namespace pins;

Driver* driver;
SensorArray* sensorArr;
Buzzer* buzz;
Button* backButt;
Button* frontButt;
int flag = 0;
int x = 60;
int y = 64;

void setup() {
  Serial.begin(9600);
  delay(500);

  // sensorArr = new SensorArray(
  //   tofLeftDiagS,
  //   tofRightDiagS,
  //   tofFrontS,
  //   tofFrontL,
  //   imuRST);

  driver = new Driver(
    motorPowerL,
    motorDirectionL,
    motorPowerR,
    motorDirectionR,
    motorMode,
    encoderL1,
    encoderL2,
    encoderR1,
    encoderR2,
    *sensorArr);

  buzz = new Buzzer(buzzer);
  backButt = new Button(backButton);
  frontButt = new Button(frontButton);

  pinMode(motorMode, OUTPUT);
  digitalWrite(motorMode, HIGH);
}

void loop() {
  if (flag == 0) {
    /* PID movement testing */
    // 400 -> 20cm
    // driver->go(0, -10, 0, 5.0);
    //driver->go(20, 0, 0, 5.0);
    // driver->go(20, 0, 3.14 / 2, 5.0);
    // driver->go(20, 20, 3.14 / 2, 5.0);
    
    // 180 degree turn
    driver->go(0, 0, 3.14, 5.0);
    
    flag = 1;
  }
    // if (backButt->read() == LOW) {
    //     y += 2;
    //     Serial.println("Increase right by 5\n");
    //     delay(1000);
    // }
    // if (frontButt->read() == LOW) {
    //     y -= 2;
    //     Serial.println("Decrease right by 5\n");
    //     delay(1000);
    // }
    // Serial.print("Left speed: ");
    // Serial.print(x);
    // Serial.print(" Right speed: ");
    // Serial.println(y);

    // driver->drive(x, y);
}
