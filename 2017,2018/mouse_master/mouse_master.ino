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

void makeNextMove(Position next);
void waitButton();

void setup() {
    /* * * * * * * * * * * * * * * * *
    * MOUSE HARDWARE INTITALIZATION *
    * * * * * * * * * * * * * * * * **/
    Serial.begin(9600);
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
    maze->initializeMaze();
    // Set maze boundary walls
    maze->setBoundaryWalls();

    backRgb->flashLED(2);
    Serial.println("Setup done");

}

void loop() {
    Serial.println("Looping.");

    // run the flood-fill algorithm
    maze->floodMaze();

    Position next_move = maze->chooseNextCell();

    // makeNextMove(next_move);

    // update walls
    long tmp[3];
    memcpy(driver->shortTofWallReadings, tmp, sizeof(tmp));
    maze->addWalls(driver->curr_angle, tmp[0], tmp[1], tmp[2]);
}

// Unfinished
void moveNextMove(Position next) {
    Position curr = maze->currPos;
    // some combination of go, turn, forward
}


void waitButton() {
    while (1) {
        if (backButt->read() == LOW) {
            frontRgb->flashLED(1);
            delay(1000);
            driver->forward(18);
            frontRgb->flashLED(0);
            break;
        }
        if (frontButt->read() == LOW) {
            frontRgb->flashLED(1);
            delay(1000);
            driver->turnLeft(90);
            frontRgb->flashLED(0);
            break;
        }
    }
}
