/**
  A test of robot turning, using motor commands and IMU sensor.
  Attempts sensor-based tank turns of 90 degrees.
*/

#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <vector>
#include <config.h>

#define FORWARD 1
#define BACKWARD 0
#define STOP -1

struct motor {
    String name;
    int powerPin;
    int directionPin;
} leftMotor, rightMotor;

void move(std::vector<motor> motors, int direction);

// Constants for test
const int speed = 30;
int flag = 0;
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);
 
void setup(void) 
{
  Serial.begin(9600);
  delay(5000);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);

  // Initialize Motors
  leftMotor.name = "Left";
  leftMotor.powerPin = pins::motorPowerL;
  leftMotor.directionPin = pins::motorDirectionL;
  rightMotor.name = "Right";
  rightMotor.powerPin = pins::motorPowerR;
  rightMotor.directionPin = pins::motorDirectionR;

  pinMode(pins::motorPowerL, OUTPUT);
  pinMode(pins::motorDirectionL, OUTPUT);
  pinMode(pins::motorPowerR, OUTPUT);
  pinMode(pins::motorDirectionR, OUTPUT);
  pinMode(pins::motorMode, OUTPUT);
  digitalWrite(pins::motorMode, HIGH);
  Serial.println("Motors initialized.");

  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);

  float start_yaw = event.orientation.x;
  float current_yaw = event.orientation.x;
  delay(2000);
  move({rightMotor}, BACKWARD);
  move({leftMotor}, FORWARD);
  while (current_yaw - start_yaw < 90 || current_yaw - start_yaw > 355) {
    bno.getEvent(&event);
    current_yaw = event.orientation.x;
    Serial.print(current_yaw);
    Serial.println();
    delay(50);
  }
  move({leftMotor, rightMotor}, STOP);
  
}

void move(std::vector<motor> motors, int direction) {
    for (unsigned int i = 0; i < motors.size(); i++) {
        if (direction == STOP) {
            analogWrite(motors[i].powerPin, 0);
        }
        else {
            Serial.print(motors[i].name);
            Serial.print(" motor moving ");
            String stringDirection = direction ? "forwards" : "backwards";
            Serial.print(stringDirection);
            Serial.println("...");

            analogWrite(motors[i].powerPin, speed);
            digitalWrite(motors[i].directionPin, direction);
        }
    }
}

void loop() {
}


