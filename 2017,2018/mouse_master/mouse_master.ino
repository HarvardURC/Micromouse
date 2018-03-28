#include "config.h"
#include "io.hh"
#include "motors.hh"
#include "sensors.hh"
#include "bluetooth.hh"
#include "maze.hh"

using namespace pins;

const float cellSize = 18; // cm

Maze* maze;
Driver* driver;
SensorArray* sensorArr;
Buzzer* buzz;
Button* backButt;
Button* frontButt;
RGB_LED* backRgb;
RGB_LED* frontRgb;

int flag = 0;
int swap_flag = 0; // if true return to the start
const char init_command = 'a';
char command = init_command;
bool bluetooth = true;


void setup() {
    /* * * * * * * * * * * * * * * * *
    * MOUSE HARDWARE INTITALIZATION *
    * * * * * * * * * * * * * * * * **/

    Serial.begin(9600);
    delay(500);

    backRgb = new RGB_LED(backLedR, backLedG, backLedB);
    backRgb->flashLED(0);

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
    *sensorArr,
    false);

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
    if (bluetooth) {
        bluetoothInitialize();
    }

    backRgb->flashLED(1);
}

void loop() {
    if (maze->currPos == maze->goalPos ||
        (maze->currPos == maze->startPos && maze->counter % 2 == 1)) {
        maze->counter++;
        ble.println("Swapping goal....");

        if (maze->currPos == maze->startPos) {
            driver->resetState();
            flag = 0;
        }
    }
    if (flag >= 0) {
        ble.println("Waiting on command");
        // waitButton(backButt);
        waitCommand();
    }
    // run the flood-fill algorithm
    maze->floodMaze();

    // determine next cell to move to
    Position next_move = maze->chooseNextCell();
    ble.print("Next move -- ");
    next_move.print(bluetooth);

    // move to that cell
    makeNextMove(next_move);
    maze->updatePosition(next_move);

    // update walls
    maze->addWalls(
        driver->curr_angle,
        driver->shortTofWallReadings[0],
        driver->shortTofWallReadings[1],
        driver->shortTofWallReadings[2]);
    ble.print("Walls:");
    for (int i = 0; i < 3; i++) {
        ble.print(driver->shortTofWallReadings[i]);
        ble.print(" ");
    }
    ble.println(" ");
    driver->clearWallData();

    maze->printWallsCell(next_move);
    flag++;
}


/* Given the position of the next cell, converts the position to x, y
 * coordinates in centimeters and runs tankGo() to for precise movement
 * in the mapping phase.
 */
void makeNextMove(Position next) {
    Position diff = next - maze->currPos;
    ble.print("Diff direction: ");
    ble.println(diff.direction());

    driver->tankGo(next.col * cellSize, next.row * cellSize, diff.direction());
    frontRgb->flashLED(1);
}


/* Waits on a button press */
void waitButton(Button* but) {
    while (1) {
        if (but->read() == LOW) {
            frontRgb->flashLED(2);
            delay(1000);
            flag = -100;
            frontRgb->flashLED(1);
            break;
        }
    }
}


/* Waits on a command from bluetooth controller */
void waitCommand() {
    while (1) {
        while (ble.available()) {
            command = ble.read();
            break;
        }
        if (command != init_command) {
            switch(command) {
                // go to next movement
                case 'g':
                    frontRgb->flashLED(2);
                    delay(1000);
                    frontRgb->flashLED(1);
                    break;
                // continue without interruption
                case 'c':
                    flag = -100;
                    break;
                default:
                    break;
            }
            command = init_command;
            break;
        }
    }
}
