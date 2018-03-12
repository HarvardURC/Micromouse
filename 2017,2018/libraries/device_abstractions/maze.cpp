#include <Arduino.h>
#include "maze.hh"

/* Global Constants */
// For Setting Wall bits in the wall array
#define NORTH 1
#define WEST  2
#define SOUTH 4
#define EAST  8

// Starting and ending position
#define STARTROW 0
#define STARTCOL 0
#define ENDROW 2
#define ENDCOL 2

// maps 0-3 direction to array offset
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


/* Gives the angle the robot needs to turn to go a position diff of p */
float Position::angFromRelPos() {
    const float base = PI / 2;
    if (row) {
        return row > 0 ? 0 : base * 2;
    }
    else {
        return col > 0 ? base * 3 : base;
    }
}


void Position::print() {
    Serial.print("Position x=");
    Serial.print(row);
    Serial.print(" y=");
    Serial.print(col);
    Serial.println(".");
}


Maze::Maze() : currPos(0, 0) {
    initializeMaze();
    setBoundaryWalls();
}


/* Initializes the maze. */
void Maze::initializeMaze() {
    // set the robot's position to the start
    currPos.row = STARTROW;
    currPos.col = STARTCOL;

    // for each cell, set the wallMap value to 240, indicating unvisited
    for (int i = 0; i < 256; i++)
    {
        wallMap[i] = 240;
    }
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
        cellStack[0] = (16 * STARTROW) + STARTCOL;
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

            // Add all unvisited, available uneighbors to the stack for the next step
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

        // Print stack for debug
        /*Serial.print ("Current Stack\n");
        for (int i = 0; i < stackPointer; i++)
        {
        Serial.print ("Stack Member: %d\n", cellStack[i]);
        }
        */
    }
}

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
void Maze::resetPosition() {
    currPos.row = STARTROW;
    currPos.col = STARTCOL;
    counter -= counter % 2;
}


void Maze::updatePosition(int row, int col) {
    currPos.row = row;
    currPos.col = col;
}


/* Converts angle in radians to direction, 0 forward */
int angleToDir(float angle) {
    return ((int)floor(angle / (PI / 2))) % 4;
}


void Maze::addWalls(float angle, long leftDiag, long front, long rightDiag) {
     // thresholds and readings for each of the 4 directions
    int irThresholds[4] = {250, 250, 0, 250};
    long irReadings[4] = {front, leftDiag, 0, rightDiag};

    // if the current cell was marked as 240+ (unvisited), reduce it to <16
    wallMap[currPos.offset()] &= 15;
    int mouseDir = angleToDir(angle);

    // for each of the 4 directions
    for (int i = 0; i < 4; i++) {
        // but not backwards because there's no back sensor
        if (i != 2) {
            int dir = (mouseDir + i) % 4;

            // the offset of adjacent cell in the direction of the current sensor
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


/* Chooses the next cell based on the flood-fill algorithm's determination
* of the adjacent cell which is closest to the destination. */
Position Maze::chooseNextCell() {
    // stores the lowest adjacent distance from the destination
    unsigned char lowest = 255;
    Position lowestPos = {0, 0};

    // Compare through all the neighbors
    for (int i = 0; i < 4; i++) {
        /* if the there's no wall in the way, and the flood fill value is the
         * lowest set it as the tentative next cell */
        if (!(wallMap[currPos.offset()] & 1 << i) &&
            cellMap[currPos.offset() + offsetMap[i]] < lowest)
        {
            int temp_offset = currPos.offset() + offsetMap[i];
            lowest = cellMap[temp_offset];
            lowestPos = getPosition(temp_offset);
        }
    }

    return lowestPos;
}
