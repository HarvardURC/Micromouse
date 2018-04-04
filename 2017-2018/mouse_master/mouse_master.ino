#include <elapsedMillis.h>
#include "bluetooth.hh"
#include "config.h"
#include "io.hh"
#include "maze.hh"
#include "motors.hh"
#include "sensors.hh"
#include "software_config.hh"

#define BUFSIZE 20

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
char command[BUFSIZE];
bool bluetooth = true;

bool commandIs(const char* cmd, bool firstchar=false);
bool commandIs(const char* token, const char* cmd, bool firstchar=false);


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
    if (bluetooth) {
        bluetoothInitialize();
        command[0] = '\0';
    }

    backRgb->flashLED(1);
}

void loop() {
    if ((maze->currPos == maze->goalPos && maze->counter % 2 == 0) ||
        (maze->currPos == maze->startPos && maze->counter % 2 == 1)) {
        maze->counter++;
        ble.println("Swapping goal....");
        if (maze->currPos == maze->startPos) {
            command[0] = '\0';
                driver->resetState();
            flag = 0;
        }
        else if (maze->currPos == maze->goalPos) {
            celebrate();
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
    Position next_move = maze->chooseNextCell(maze->currPos);
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

    // only print the walls on the speedrun
    if (maze->counter == 0) {
        ble.print("Walls:");
        for (int i = 0; i < 3; i++) {
            ble.print(driver->shortTofWallReadings[i]);
            ble.print(" ");
        }
        ble.println(" ");
    }
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
    // command interface menu vals
    int tuning = -1;
    float p = 0;
    float i = 0;
    float d = 0;
    PidController* pid;

    const int notifyTime = 8000;
    elapsedMillis timer = 0;

    while (1) {
        if (timer > notifyTime) {
            ble.println("Waiting on command...");
            timer = 0;
        }

        while (ble.available()) {
            ble.readline(command, BUFSIZE);
        }

        if (command[0] != '\0') {
            // reset maze
            if (commandIs("reset")) {
                flag = 0;
                driver->resetState();
                ble.println("Robot state reset. Ready for next run.");
            }
            else if (commandIs("fullreset")) {
                flag = 0;
                driver->resetState();
                maze->reset();
                ble.print("Maze reset.\n");
            }
            // go to next cell
            else if (commandIs("go")) {
                frontRgb->flashLED(2);
                delay(1000);
                frontRgb->flashLED(1);
                break;
            }
            // continue without interruption
            else if (commandIs("start", true)) {
                ble.println("Running maze.");
                flag = -100;
                break;
            }
            // move forward
            else if (commandIs("w")) {
                driver->forward(cellSize);
            }
            // turn left
            else if (commandIs("a")) {
                driver->turnLeft(90);
            }
            // turn right
            else if (commandIs("d")) {
                driver->turnRight(90);
            }
            else if (commandIs("celebrate")) {
                celebrate();
            }
            else if (commandIs("tune")) {
                tuning = 0;
                ble.println("Pick PID to tune. [linear, angular, front tof]");
            }
            else if (commandIs("quit")) {
                tuning = -1;
            }
            else if (tuning >= 0) {
                if (tuning > 0) {
                    p = pid->proportion;
                    i = pid->integral;
                    d = pid->derivative;

                    const char* token = strtok(command, " ");
                    if (commandIs(token, "proportion")) {
                        token = strtok(NULL, " ");
                        p = atof(token);
                    }
                    else if (commandIs(token, "integral")) {
                        token = strtok(NULL, " ");
                        i = atof(token);
                    }
                    else if (commandIs(token, "derivative")) {
                        token = strtok(NULL, " ");
                        d = atof(token);
                    }
                    pid->setTunings(p, i, d);
                }
                else if (tuning == 0) {
                    if (commandIs("linear")) {
                        tuning = 1;
                    }
                    else if (commandIs("angular")) {
                        tuning = 2;
                    }
                    else if (commandIs("front tof")) {
                        tuning = 3;
                    }
                }

                if (tuning > 0) {
                    switch(tuning) {
                        case 1: {
                            pid = &driver->_pid_x;
                            ble.print("linear: ");
                            break;
                        }
                        case 2: {
                            pid = &driver->_pid_a;
                            ble.print("angular: ");
                            break;
                        }
                        case 3: {
                            pid = &driver->_pid_front_tof;
                            ble.print("front tof: ");
                            break;
                        }
                    }
                    pid->printTunings();
                    ble.println(
                        "Pick var to tune. [proportion, integral, derivative]");
                        ble.println("Ex: 'proportion  10.5'");
                }
            }
            else if (commandIs("help")) {
                ble.println("Possible commands: "
                    "[go, start, reset, w, a, d, tune, celebrate, quit]");
            }
            else {
                ble.println(
                    "Invalid command. See the README for valid commands.");
            }

            timer = 0;
        }
        command[0] = '\0';
    }
    command[0] = '\0';
}

bool commandIs(const char* cmd, bool firstchar) {
    return !strcmp(command, cmd) || (firstchar && command[0] == cmd[0]);
}

bool commandIs(const char* token, const char* cmd, bool firstchar) {
    return !strcmp(token, cmd) || (firstchar && token[0] == cmd[0]);
}

void celebrate() {
    for (size_t j = 0; j < 4; j++) {
        for (size_t i = 0; i < 2; i++) {
            frontRgb->flashLED(i);
            backRgb->flashLED(i);
            delay(50);
        }
    }
}
