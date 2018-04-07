#include <elapsedMillis.h>
#include <EEPROM.h>
#include "bluetooth.hh"
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
char command[BUFSIZE]; // buffer to hold bluetooth commands
bool bluetooth = false; // activate bluetooth (and command system)
bool backupFlag = true;

bool commandIs(const char* token, const char* cmd, bool firstchar=false);

bool abort_run = 0;

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

    if (bluetooth) {
        backRgb->flashLED(2);
        bluetoothInitialize();
        command[0] = '\0';
    }
    backRgb->flashLED(1);

    driver->encoderOnlyFlag = true;

    // front button will kill any running process
    // attachInterrupt(frontButton, abort_isr, RISING);
}


void loop() {
    if (command_flag >= 0) {
        debug_println("Waiting on command");
        if (bluetooth) {
            waitCommand();
        } else {
            waitButton();
        }
    }

    if ((maze->currPos == maze->goalPos && maze->counter % 2 == 0) ||
        (maze->currPos == maze->startPos && maze->counter % 2 == 1)) {
        maze->counter++;
        debug_println("Swapping goal....");
        if (maze->counter <= 2) {
            buzz->siren();
            maze->writeEEPROM();
        }
        if (maze->currPos == maze->startPos) {
            command[0] = '\0';
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
    debug_print("Next move -- ");
    next_move.print(bluetooth);

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

    bool backupMode = backupFlag ? maze->wallBehind(diff.direction()) : false;

    driver->tankGo(
        next.col * swconst::cellSize,
        next.row * swconst::cellSize,
        false,
        // maze->wallsOnSides(driver->curr_angle),
        backupMode
        );
    frontRgb->flashLED(1);
}


void abort_isr() {
    abort_run = 1;
}


/* waitButton()
 * Waits on a button press. When pressed it starts the run of the maze.
 */
void waitButton() {
    while (1) {
        /* press back button for a run, for 3-6 seconds for an EEPROM read
         * and 6+ for EEPROM clear */
        if (backButt->read() == LOW) {
            elapsedMillis timer = 0;
            while(backButt->read() == LOW) {
                if (timer > 3000 && timer < 6000) {
                    backRgb->turnOn(1);
                } else if (timer >= 6000) {
                    backRgb->turnOn(2);
                } else {
                    backRgb->turnOn(0);
                }
            }
            backRgb->turnOff();

            if (timer < 3000) {
                driver->resetState();
                frontRgb->flashLED(2);
                delay(1000);
                frontRgb->flashLED(1);

                command_flag = -1000;
                break;
            } else if (timer < 6000) {
                maze->readEEPROM();
                buzz->siren();
            } else {
                buzz->siren();
                delay(50);
                buzz->siren();
                delay(50);
                buzz->siren();
                maze->clearEEPROM();
            }
        }
        // press front button to increment speed run counter
        else if (frontButt->read() == LOW) {
            elapsedMillis timer = 0;
            while(frontButt->read() == LOW) {
                if (timer > 3000 && timer < 6000) {
                    backRgb->turnOn(1);
                } else if (timer >= 6000) {
                    backRgb->turnOn(2);
                } else {
                    backRgb->turnOn(0);
                }
            }
            backRgb->turnOff();

            if (timer < 3000) {
                frontRgb->flashLED(1);
                if (maze->counter == 0) {
                    maze->counter++;
                } else {
                    maze->counter = min(maze->counter + 2, 4);
                }
            }
            else if (timer < 6000) {
                celebrate();
                backupFlag = !backupFlag;
            }
            else {
                buzz->siren();
                driver->encoderOnlyFlag = false;
            }
        }
    }
}


/* waitCommand()
 * Waits on a command from bluetooth controller.
 * See README.md for valid commands.
 */
void waitCommand() {
    // command interface menu vals
    int tuning = -1;
    float p = 0;
    float i = 0;
    float d = 0;
    PidController nullPid(0, 0, 0);
    PidController* pid = &nullPid;
    const int notifyTime = 8000;
    elapsedMillis timer = 0;

    while (1) {
        if (timer > notifyTime) {
            debug_println("Waiting on command...");
            timer = 0;
        }

        while (ble.available()) {
            ble.readline(command, BUFSIZE);
        }

        if (command[0] != '\0') {
            char* token = strtok(command, " ");

            // reset maze
            if (commandIs(token, "reset")) {
                driver->resetState();
                debug_println("Robot state reset. Ready for next run.");
            }
            else if (commandIs(token, "fullreset")) {
                driver->resetState();
                maze->reset();
                debug_print("Maze reset.\n");
            }
            // go to next cell
            else if (commandIs(token, "go")) {
                int numMoves = atoi(strtok(NULL, " "));
                command_flag = -1 * numMoves;

                frontRgb->flashLED(2);
                delay(1000);
                frontRgb->flashLED(1);
                break;
            }
            // continue without interruption
            else if (commandIs(token, "start")) {
                debug_println("Running maze.");
                driver->resetState();
                command_flag = -1000;
                break;
            }
            // move forward
            else if (commandIs(token, "w")) {
                driver->forward(swconst::cellSize);
            }
            // turn left
            else if (commandIs(token, "a")) {
                driver->turnLeft(90);
            }
            // turn right
            else if (commandIs(token, "d")) {
                driver->turnRight(90);
            }
            else if (commandIs(token, "celebrate")) {
                celebrate();
            }
            else if (commandIs(token, "setgoal")) {
                int row = atoi(strtok(NULL, " "));
                int col = atoi(strtok(NULL, " "));
                Position p(row, col);
                maze->setGoal(p);
            }
            else if (commandIs(token, "tune")) {
                tuning = 0;
                debug_println("Pick PID to tune. [linear, angular, fronttof]");
            }
            else if (commandIs(token, "quit")) {
                tuning = -1;
            }
            else if (tuning >= 0) {
                if (tuning > 0) {
                    p = pid->proportion;
                    i = pid->integral;
                    d = pid->derivative;

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
                    if (commandIs(token, "linear")) {
                        tuning = 1;
                    }
                    else if (commandIs(token, "angular")) {
                        tuning = 2;
                    }
                    else if (commandIs(token, "fronttof")) {
                        tuning = 3;
                    }
                }

                if (tuning > 0) {
                    switch(tuning) {
                        case 1: {
                            pid = &driver->_pid_x;
                            debug_print("linear: ");
                            break;
                        }
                        case 2: {
                            pid = &driver->_pid_a;
                            debug_print("angular: ");
                            break;
                        }
                        case 3: {
                            pid = &driver->_pid_front_tof;
                            debug_print("fronttof: ");
                            break;
                        }
                    }
                    pid->printTunings();
                    debug_println(
                        "Pick var to tune. [proportion, integral, derivative]");
                        debug_println("Ex: 'proportion  10.5'");
                }
            }
            else if (commandIs(token, "help")) {
                debug_println("Possible commands: "
                    "[go, start, reset, fullreset, w, a, d, tune,"
                    " celebrate, quit, setgoal]");
            }
            else {
                debug_println(
                    "Invalid command. See the README for valid commands.");
            }

            timer = 0;
        }
        command[0] = '\0';
    }
    command[0] = '\0';
}


/* commandIs()
 * Checks if the token is the same string as a command. If firstchar is
 * enabled, it passes if the first character is the same.
 */
bool commandIs(const char* token, const char* cmd, bool firstchar) {
    return !strcmp(token, cmd) || (firstchar && token[0] == cmd[0]);
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
