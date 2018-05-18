/* mouse_stripped.ino
 * Contains a barebones mouse code. Without all the bells and whistles of
 * mouse_master.ino
 *
 * After turning on, wait for the back LED to flash red, then press the
 * back button to start the maze run.
 */

#include <elapsedMillis.h>
#include <EEPROM.h>
#include "config.h"
#include "io.hh"
#include "maze.hh"
#include "motors.hh"
#include "sensors.hh"
#include "software_config.hh"

#define BUFSIZE 20

using namespace pins;

Maze* maze;
Driver* driver;
SensorArray* sensorArr;
Buzzer* buzz;
Button* backButt;
Button* frontButt;
RGB_LED* backRgb;
RGB_LED* frontRgb;

int command_flag = 0; // wait for a command or button press
int swap_flag = 0; // if true return to the start

void setup() {
    /* * * * * * * * * * * * * * * * *
    * MOUSE HARDWARE INTITALIZATION *
    * * * * * * * * * * * * * * * * **/
    pinMode(motorMode, OUTPUT);
    digitalWrite(motorMode, HIGH);

    Serial.begin(9600);
    delay(500);


    backRgb = new RGB_LED(backLedR, backLedG, backLedB);
    backRgb->flashLED(0);
    frontRgb = new RGB_LED(frontLedR, frontLedG, frontLedB);

    maze = new Maze();

    sensorArr = new SensorArray(
      tofDiagL,
      tofFrontL,
      tofFrontR,
      tofDiagR,
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
    *frontRgb);

    buzz = new Buzzer(buzzer);
    backButt = new Button(backButton);
    frontButt = new Button(frontButton);

    /* * * * * * * * * * * * * * *
    * FLOODFILL INITIALIZATION  *
    * * * * * * * * * * * * * * */
    // Initialize the maze
    maze->initializeMaze();
    // Set maze boundary walls
    maze->setBoundaryWalls();

    backRgb->flashLED(1);
}


void loop() {
    if (command_flag >= 0) {
        debug_println("Waiting on command");
        waitButton();
    }

    if ((maze->currPos == maze->goalPos && maze->counter % 2 == 0) ||
        (maze->currPos == maze->startPos && maze->counter % 2 == 1)) {
        maze->counter++;
        debug_println("Swapping goal....");
        if (maze->counter <= 2) {
            buzz->siren();
        }
        if (maze->currPos == maze->startPos) {
            driver->resetState();
            driver->cfgNum = min(maze->counter / 2, 2);
            command_flag = 1;
        }
        else if (maze->currPos == maze->goalPos) {
            celebrate();
        }
    }

    // run the flood-fill algorithm
    maze->floodMaze();

    // determine next cell to move to
    Position next_move = maze->chooseNextCell(maze->currPos, maze->counter >= 2);

    // move to that cell
    makeNextMove(next_move);
    maze->updatePosition(next_move);

    // update walls
    maze->addWalls(
        driver->curr_angle,
        driver->shortTofWallReadings[LEFTDIAG],
        driver->shortTofWallReadings[LEFTFRONT],
        driver->shortTofWallReadings[RIGHTDIAG]);

    // only print the walls on the speedrun
    if (maze->counter == 0) {
        debug_print("Walls:");
        for (int i = 0; i < 4; i++) {
            if (i == RIGHTFRONT) { continue; } // ignore right front tof
            debug_print(driver->shortTofWallReadings[i]);
            debug_print(" ");
        }
        debug_println(" ");
    }
    driver->clearWallData();

    maze->printWallsCell(next_move);
}


/* Given the position of the next cell, converts the position to x, y
 * coordinates in centimeters and runs tankGo() to for precise movement
 * in the mapping phase.
 */

void makeNextMove(Position next) {
    Position diff = next - maze->currPos;
    debug_print("Diff direction: ");
    debug_println(diff.direction());

    driver->tankGo(
        next.col * swconst::cellSize,
        next.row * swconst::cellSize);
    frontRgb->flashLED(1);
}


/* waitButton()
 * Waits on a back button press. When pressed it starts the run of the maze.
 */
void waitButton() {
    while (1) {
        if (backButt->read() == LOW) {
            backRgb->turnOn(0);
            delay(1000);
            backRgb->turnOn(1);
            command_flag = -1000;
            break;
        }
    }
}

/* celebrate()
 * Flashes the LEDs in celebration.
 */
void celebrate() {
    for (size_t j = 0; j < 4; j++) {
        for (size_t i = 0; i < 2; i++) {
            frontRgb->flashLED(i);
            backRgb->flashLED(i);
            buzz->siren();
            delay(50);
        }
    }
}
