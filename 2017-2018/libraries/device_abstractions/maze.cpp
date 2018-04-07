#include <Arduino.h>
#include <EEPROM.h>
#include "maze.hh"
#include "bluetooth.hh"
#include "helpers.hh"
#include "software_config.hh"

using namespace swconst;

/* Global Constants */
// For Setting Wall bits in the wall array
#define NORTH 1
#define WEST  2
#define SOUTH 4
#define EAST  8

// Starting and ending position
#define STARTROW 0
#define STARTCOL 0
#define ENDROW 7
#define ENDCOL 7

// maps 0-3 direction to array offset, NORTH=0, WEST, SOUTH, EAST
int offsetMap[4] = {16, -1, -16, 1};


Position::Position(int r, int c) {
    row = r;
    col = c;
}


/* Converts the `Position` object's coordinates into an offset into
 * the maze maps */
int Position::offset() {
    return 16 * row + col;
}


/* Converts a map array offset into a `Position` object */
Position getPosition(int offset) {
    Position p(offset / 16, offset % 16);
    return p;
}


/* Gives the direction in radians of a relative position with 0 at NORTH.
 * A relative position is a point on the unit circle used for indicating
 * direction like those in this set {(0, 1), (1, 0), (-1, 0), (0, -1)}
 * > (-1, 0).direction() -> PI / 2 */
float Position::direction() {
    const float base = PI / 2;
    if (row) {
        return row > 0 ? 0 : base * 2;
    }
    else {
        return col > 0 ? base * -1 : base;
    }
}


void Position::print(int bluetooth) {
    debug_print("Position x=");
    debug_print(col);
    debug_print(" y=");
    debug_print(row);
    debug_println(".");
}


Maze::Maze()
    : currPos(STARTROW, STARTCOL),
      startPos(STARTROW, STARTCOL),
      goalPos(ENDROW, ENDCOL) {
    initializeMaze();
}


/* Initializes the maze's start position and wallMap to dummy values. */
void Maze::initializeMaze() {
    // set the robot's position to the start
    currPos.row = STARTROW;
    currPos.col = STARTCOL;

    // for each cell, set the wallMap value to 240, indicating unvisited
    for (int i = 0; i < 256; i++)
    {
        wallMap[i] = 240;
    }
    setBoundaryWalls();
}

/* Sets up wallMap with the boundary walls of the maze. */
void Maze::setBoundaryWalls ()
{
    // NORTH
    for (int i = 0; i < 16; i++) {
        wallMap[i] |= SOUTH;
    }
    // EAST
    for (int i = 15; i < 256; i += 16) {
        wallMap[i] |= EAST;
    }
    // SOUTH
    for (int i = 240; i < 256; i++) {
        wallMap[i] |= NORTH;
    }
    // WEST
    for (int i = 0; i < 241; i += 16) {
        wallMap[i] |= WEST;
    }

    // setup initial cell
    wallMap[0] |= EAST;
}

/* Runs the flood-fill algorithm, updating cellMap with distances. */
void Maze::floodMaze() {
    // reset the array of values
    for (int i = 0; i < 256; i++) {
        cellMap[i] = 255;
    }

    // char to store current distance from end
    unsigned char stepValue = 0;
    // array to act as stack
    unsigned char cellStack[256];
    // array to act as temporary storage
    unsigned char nextCellStack[256];
    // the index of the top of each stack; 0 means the stack is empty
    int stackPointer, nextStackPointer;

    // Initialize pointers to the top of each stack
    stackPointer = 0;
    nextStackPointer = 0;

    // if we're on an even run, set the destination to be the center of the maze
    if (counter % 2 == 0) {
        stackPointer = 4;
        cellStack[0] = (16 * ENDROW) + ENDCOL;
        cellStack[1] = (16 * (ENDROW + 1)) + ENDCOL;
        cellStack[2] = (16 * ENDROW) + (ENDCOL + 1);
        cellStack[3] = (16 * (ENDROW + 1)) + (ENDCOL + 1);
    }
    // otherwise, the destination is the start of the maze
    else {
        stackPointer = 1;
        cellStack[0] = (16 * startPos.row) + startPos.col;
    }

    // as long as the stack is non-empty
    while (stackPointer > 0) {
        // Stop flooding if the robot's cell has a value
        if (cellMap[16 * currPos.row + currPos.col] != 255) break;

        // Pop the cell off the stack
        unsigned char curCell = cellStack[stackPointer - 1];
        stackPointer--;

        // if the cell has not yet been assigned a distance value
        if (cellMap[curCell] == 255) {
            // Set the current cell value to the step path value
            cellMap[curCell] = stepValue;

            // Serial.print ("Flood Cell: %d\n", curCell);

            /* Add all unvisited, available uneighbors to the stack
             * for the next step */
            for (int i = 0; i < 4; i++) {
                unsigned char adjCell = curCell + offsetMap[i];
                if (adjCell >= 0 && adjCell < 256 && cellMap[adjCell] == 255 &&
                    (wallMap[curCell] & 1 << i) == 0)
                {
                    nextCellStack[nextStackPointer] = adjCell;
                    nextStackPointer++;
                }
            }
        }

        // if the stack is empty, move on to the next step value
        if (stackPointer == 0) {
            // move the next stack to the main stack
            for (int i = 0; i < nextStackPointer; i++) {
                cellStack[i] = nextCellStack[i];
            }

            stackPointer = nextStackPointer;

            // empty next stack
            nextStackPointer = 0;

            stepValue++;
        }
    }
}


/* prints what a wall cell looks like to bluetooth */
void Maze::printWallsCell(Position p) {
    unsigned char cell = wallMap[p.offset()];
    char tmp;

    debug_print(" ");
    tmp = cell & NORTH ? '_' : ' ';
    debug_println(tmp);
    tmp = cell & WEST ? '|' : ' ';
    debug_print(tmp);
    debug_print(" ");
    tmp = cell & EAST ? '|': ' ';
    debug_println(tmp);
    debug_print(" ");
    tmp = cell & SOUTH ? '-' : ' ';
    debug_println(tmp);
}


/* Debug function */
void Maze::printMaze() {
    for (int i = 0; i < 16; i++) {
        Serial.print ("---\t");
    }

    for (int i = 15; i >= 0; i--) {
        Serial.print ("\n");
        for (int j = 0; j < 16; j++) {
            if (currPos.row == i && currPos.col == j) {
                Serial.print("@");
            }
            Serial.print(cellMap[16 * i + j]);
            Serial.print("\t");
        }
        Serial.print("\n");
    }

    for (int i = 0; i < 16; i++) {
        Serial.print ("---\t");
    }
    Serial.print ("\n");
}


/* Resets the robot position in the maze to the start cell */
void Maze::reset() {
    currPos.row = STARTROW;
    currPos.col = STARTCOL;
    initializeMaze();
    counter = 0;
}


void Maze::updatePosition(Position p) {
    currPos = p;
}

void Maze::setGoal(Position p) {
    goalPos = p;
    debug_print("Goal set to ");
    p.print();
}


/* Converts angle in radians to relative direction
 * forward=0, left=1 back=2, right=3 */
int angleToDir(float angle) {
    while (angle < 0) {
        angle += 2 * PI;
    }
    return ((int)floor((angle + PI / 4) / (PI / 2))) % 4;
}


/* Takes in three short ToF sensor readings and the orientation of the
 * robot and determines where the walls are in the current cell and adds
 * them to the wallMap. */
void Maze::addWalls(float angle, long leftDiag, long front, long rightDiag) {
    long irReadings[4] = {front, leftDiag, 0, rightDiag};

    // if the current cell was marked as 240+ (unvisited), reduce it to <16
    if (wallMap[currPos.offset()] > 16) {
        wallMap[currPos.offset()] &= 15;
        int mouseDir = angleToDir(angle);

        // for each of the 4 directions
        for (int i = 0; i < 4; i++) {
            // but not backwards because there's no back sensor
            if (i != 2) {
                int dir = (mouseDir + i) % 4;

                /* the offset of adjacent cell in the direction
                 * of the current sensor */
                int oppositeCell = currPos.offset() + offsetMap[dir];

                // if IR threshold is exceeded
                if (irReadings[i] < irThresholds[i]) {
                    // set wall for current cell
                    wallMap[currPos.offset()] |= 1 << dir;

                    // set wall for opposite cell if valid
                    if (oppositeCell >= 0 && oppositeCell < 256)
                    {
                      wallMap[oppositeCell] |= 1 << ((dir + 2) % 4);
                    }
                }
            }
        }
    }
}


/* Chooses the next cell based on the flood-fill algorithm's determination
 * of the adjacent cell which is closest to the destination. */
Position Maze::chooseNextCell(Position pos, bool straights) {
    // stores the lowest adjacent distance from the destination
    unsigned char lowest = 255;
    int dir = 0;
    Position lowestPos = {0, 0};

    // Compare through all the neighbors
    for (int i = 0; i < 4; i++) {
        int test_offset = pos.offset() + offsetMap[i];

        /* if the there's no wall in the way, and the flood fill value is the
         * lowest set it as the tentative next cell */
        if (test_offset >= 0 && test_offset <= 255 &&
            !(wallMap[pos.offset()] & 1 << i) &&
            cellMap[test_offset] < lowest)
        {
            if (lowest == 255 || rand() % 2 == 1) {
                lowest = cellMap[test_offset];
                lowestPos = getPosition(test_offset);
                dir = i;
            }
        }
    }

    // long straight of ways
    if (counter > 0 && straights) {
        int n_cells = 1;
        while (n_cells < 16) {
            // test if next cell is valid, if it is set it to lowestPos
            int test_offset = pos.offset() + offsetMap[dir] * (n_cells + 1);
            int lowest_offset = pos.offset() + offsetMap[dir] * n_cells;

            if (test_offset >= 0 && test_offset <= 255 &&
                !(wallMap[lowest_offset] & 1 << dir) &&
                cellMap[test_offset] < lowest)
            {
                lowest = cellMap[test_offset];
                lowestPos = getPosition(test_offset);
                n_cells++;
                debug_print("Keeping ");
                lowestPos.print(1);
            }
            else {
                break;
            }
        }
    }

    return lowestPos;
}

bool Maze::wallBehind(float angle) {
    int direction = angleToDir(angle);
    return (wallMap[currPos.offset()] & 1 << ((direction + 2) % 4));
}

bool Maze::wallsOnSides(float angle) {
    int direction = angleToDir(angle);
    return ((wallMap[currPos.offset()] & 1 << ((direction + 1) % 4))
    && (wallMap[currPos.offset()] & 1 << ((direction + 3) % 4)));
}

void Maze::writeEEPROM() {
    for (int i = 0; i < 256; i++) {
        EEPROM.write(i, wallMap[i]);
    }
}

void Maze::readEEPROM() {
    for (int i = 0; i < 256; i++) {
        wallMap[i] = EEPROM.read(i);
    }
}

void Maze::clearEEPROM() {
    for (int i = 0 ; i < 256; i++) {
        EEPROM.write(i, 63);
    }
}
