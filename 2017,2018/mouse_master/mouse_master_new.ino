#include "config.h"
#include "io.hh"
#include "motors.hh"
#include "sensors.hh"
#include "bluetooth.hh"
#include "maze.hh"

using namespace pins;

Maze* maze;
Driver* driver;
SensorArray* sensorArr;
Buzzer* buzz;
Button* backButt;
Button* frontButt;
RGB_LED* backRgb;
RGB_LED* frontRgb;

void setup() {
    /* * * * * * * * * * * * * * * * *
    * MOUSE HARDWARE INTITALIZATION *
    * * * * * * * * * * * * * * * * **/
    if (debugMode)
    {
    Serial.begin(9600);
    }
    delay(500);

    pinMode(motorMode, OUTPUT);
    digitalWrite(motorMode, HIGH);

    maze = new Maze();

    sensorArr = new SensorArray(
      tofLeftDiagS,
      tofRightDiagS,
      tofFrontS,
      tofFrontL,
      imuRST);

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
    backRgb = new RGB_LED(backLedR, backLedG, backLedB);
    frontRgb = new RGB_LED(frontLedR, frontLedG, frontLedB);

    /* * * * * * * * * * * * * * *
    * FLOODFILL INITIALIZATION  *
    * * * * * * * * * * * * * * */
    // Initialize the maze
    maze.initializeMaze();
    // Set maze boundary walls
    maze.setBoundaryWalls();

    backRgb->flashLED(2);
    Serial.println("Setup done");

}

void loop() {
    Serial.println("Looping.");

    // run the flood-fill algorithm
    maze.floodMaze();
    Position next_move = maze.chooseNextCell();
}
