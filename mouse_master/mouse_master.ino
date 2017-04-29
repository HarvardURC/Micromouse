#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Util.h>
#include "Motors_2016.h"
#include <VL6180X.h>
#include <Wire.h>

/* Global Constants */
// For Setting Wall bits in the wall array
#define NORTH 1
#define EAST  2
#define SOUTH 4
#define WEST  8

// Starting and ending position
#define STARTROW 0
#define STARTCOL 0
#define ENDROW 7
#define ENDCOL 7

int debugMode = 0;
int virtualMode = 0;

// 0 for North
// 1 for East
// 2 for South 
// 3 for West
int mouseDir = 0;

// maps 0-3 direction to array offset
int offsetMap[4] = {16, 1, -16, -1};

// define distance sensor pins
VL6180X *leftIR;
VL6180X *leftDiagIR;
VL6180X *frontIR;
VL6180X *rightDiagIR;
VL6180X *rightIR;

//extern TwoWire Wire1;

// initialize motors object pointer
Motors_2016* motors; 

// define push-button pin
int buttonPin = 7;

/* Global Variables */
// Global array to store the cell values
unsigned char cellMap[256];
// Global array to store the wall bits
unsigned char wallMap[256];
// Global array for virtual maze walls
unsigned char virtualWallMap[256];
/*
={0b1100,0b100,0b101,0b110,0b1100,0b110,0b1100,0b101,0b101,0b110,0b1000,0b101,0b101,0b110,0b1000,0b110,
  0b1010,0b1010,0b1110,0b1010,0b1011,0b1010,0b1010,0b1100,0b111,0b1010,0b1010,0b1100,0b110,0b1010,0b1010,0b1010,
  0b1010,0b1010,0b1000,0b1,0b101,0b10,0b1010,0b1010,0b1100,0b11,0b1010,0b1010,0b1010,0b1010,0b1010,0b1010,
  0b1010,0b1010,0b1011,0b1100,0b110,0b1010,0b1010,0b1010,0b1010,0b1100,0b11,0b1010,0b1010,0b1010,0b1010,0b1010,
  0b1010,0b1000,0b0101,0b0010,0b1000,0b0001,0b0001,0b0011,0b1010,0b1011,0b1100,0b0011,0b1010,0b1010,0b1010,0b1010,
  0b1010,0b1010,0b1110,0b1011,0b1010,0b1100,0b101,0b110,0b1001,0b101,0b0,0b101,0b10,0b1010,0b1010,0b1010,
  0b1010,0b1010,0b1000,0b110,0b1010,0b1010,0b1100,0b1,0b101,0b110,0b1010,0b1101,0b10,0b1010,0b1010,0b1010,
  0b1010,0b1010,0b1010,0b1011,0b1010,0b1010,0b1010,0b1100,0b110,0b1010,0b1010,0b1101,0b10,0b1010,0b1010,0b1010,
  0b1010,0b1011,0b1000,0b101,0b1,0b11,0b1010,0b1001,0b10,0b1010,0b1001,0b101,0b11,0b1010,0b1010,0b1010,
  0b1000,0b111,0b1010,0b1100,0b101,0b110,0b1001,0b111,0b1010,0b1000,0b101,0b101,0b101,0b10,0b1010,0b1010,
  0b1010,0b1110,0b1010,0b1010,0b1100,0b1,0b101,0b101,0b11,0b1010,0b1100,0b101,0b101,0b10,0b1010,0b1010,
  0b1010,0b1011,0b1010,0b1010,0b1010,0b1101,0b100,0b100,0b101,0b10,0b1010,0b1100,0b111,0b1010,0b1010,0b1010,
  0b1000,0b101,0b10,0b1010,0b1001,0b111,0b1010,0b1010,0b1110,0b1010,0b1010,0b1000,0b101,0b11,0b1110,0b1010,
  0b1010,0b1110,0b1010,0b1001,0b101,0b101,0b11,0b1010,0b1010,0b1011,0b1010,0b1001,0b101,0b101,0b11,0b1010,
  0b1010,0b1010,0b1000,0b101,0b101,0b101,0b111,0b1010,0b1001,0b101,0b11,0b1010,0b1101,0b101,0b101,0b10,
  0b1001,0b11,0b1001,0b101,0b101,0b101,0b1,0b101,0b101,0b101,0b101,0b1,0b101,0b101,0b101,0b11};*/
// Global ints for current position
int currentRow, currentCol;
// Global counter, keeps track of run number to set speed and destination cell
int counter = 0;
// For going fast: 1 if the last move was forward
int lastWasForward = 0;
// Flag to side-bump, which can be raised by the forward function
int sideCorrectFlag = 0;

/* Function Prototypes */
void initializeMaze();
void floodMaze();
void setBoundaryWalls ();
void setTestWalls();
int chooseNextDir(int currentCell, int _mouseDir);
void makeNextMove();
void turnRight();
void turnLeft();
int moveForward();
void makeNextMove();
void onButtonRelease();
void readCell();
void debugBlink(int times);
void printMaze();

void setup()
{
  /* * * * * * * * * * * * * * * * * 
   * MOUSE HARDWARE INTITALIZATION *
   * * * * * * * * * * * * * * * * **/
  if (debugMode)
  {
    Serial.begin(9600);
  }
  
  pinMode(13, OUTPUT); // onboard LED

  // Initialize sensors
  Wire.begin();
  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(17, OUTPUT);
  digitalWrite(23, HIGH);
  digitalWrite(22, LOW);
  digitalWrite(21, LOW);
  digitalWrite(20, LOW);
  digitalWrite(17, LOW);
  Serial.print("trying IR..."); 
  // left IR
  leftIR->init();
  leftIR->configureDefault();
  leftIR->setScaling(2);
  leftIR->setAddress(1);
  Serial.print("left IR Connected!");
  // leftDiag IR
  digitalWrite(22, HIGH); 
  leftDiagIR->init();
  leftDiagIR->configureDefault();
  leftDiagIR->setScaling(2);
  leftDiagIR->setAddress(2);
  Serial.print("leftDIag IR Connected!");
  // front IR
  digitalWrite(21, HIGH); 
  frontIR->init();
  frontIR->configureDefault();
  frontIR->setScaling(2);
  frontIR->setAddress(3);
  Serial.print("front IR Connected!");
  // right Diag IR
  digitalWrite(20, HIGH); 
  rightDiagIR->init();
  rightDiagIR->configureDefault();
  rightDiagIR->setScaling(2);
  rightDiagIR->setAddress(4);
  Serial.print("rightDIag IR Connected!");
  // right IR
  digitalWrite(17, HIGH); 
  rightIR->init();
  rightIR->configureDefault();
  rightIR->setScaling(2);
  rightIR->setAddress(5);
  Serial.print("All IR Connected!");
  // initilalize motors object w/ pins and sensors
  motors = new Motors_2016 (9, 10, 6, 12,
                            7, 8,  2, 1, 
                            frontIR, leftIR, 
                            rightIR, leftDiagIR, rightDiagIR);

  // set push-button pinmode, set it to trigger onButtonRelease on release
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), onButtonRelease, RISING);
  
  /* * * * * * * * * * * * * * *
   * FLOODFILL INITIALIZATION  *
   * * * * * * * * * * * * * * */
  // Initialize the maze
  initializeMaze ();
  // Set maze boundary walls
  setBoundaryWalls ();

  // if in virtual mode, do a countdown to allow time to open the serial monitor
  if (virtualMode)
  {
    for (int i = 5; i > 0; i--) {
      Serial.println(i);
      wait(1000);
    }
  }
}

void loop()
{
  // use the IR sensors to detect walls, store them in wallMap
  readCell();

  // if in debug mode, blink some information
  if (debugMode)
  {
    debugBlink(currentRow);
    wait(1000);
    debugBlink(currentCol);
    wait(1000);
    debugBlink(wallMap[16 * currentRow + currentCol]);
    wait(1000);
    debugBlink(mouseDir);
  }

  // run the flood-fill algorithm
  floodMaze();
  if (virtualMode)
  {
    printMaze();
  }

  // move to the next cell
  makeNextMove();

  /* if we've reached the destination (alternates between start and end of maze)
   * then increment the counter that keeps track of the number of runs */
  if(cellMap[(16*currentRow) + currentCol] == 0)
  {
    counter++;
  }

  if (virtualMode)
  {
    wait(1000);
  }

  // if the push-button was pressed, reset position to the start
  if (motors->releaseFlag)
  {
    currentRow = STARTROW;
    currentCol = STARTCOL;
    mouseDir = 0;
    counter -= counter % 2;
    motors->releaseFlag = 0;
    if (debugMode)
    {
      debugBlink(2);
    }
  }
}

/* Blinks the onboard LED a specified number of times. */
void debugBlink(int times) {
  for (int i = 0; i < times; i++)
  {
    digitalWrite(13, HIGH);
    wait(200);
    digitalWrite(13, LOW);
    wait(200);
  }
}

/* * * * * * * * * * * * * * * *
 *       HARDWARE CODE         *
 * * * * * * * * * * * * * * * */

/* Detects walls using the IR sensors and records them in wallMap. */
void readCell()
{
  // thresholds and readings for each of the 4 directions
  int irThresholds[4] = {270, 300, 1023, 300};
  
  int readings[4] = {frontIR -> readRangeSingleMillimeters(),
                     rightIR -> readRangeSingleMillimeters(),
                     0,
                     leftIR -> readRangeSingleMillimeters()};

  unsigned char currentCell = 16 * currentRow + currentCol;

  // if the current cell was marked as 240+ (unvisited), reduce it to <16
  wallMap[currentCell] &= 15;

  // for each of the 4 directions
  for (int i = 0; i < 4; i++)
  {
    // but not backwards because there's no back sensor
    if (i != 2)
    {
      int dir = (mouseDir + i) % 4;

      // the adjacent cell in the direction of the current sensor
      int oppositeCell = currentCell + offsetMap[dir];

      // if IR threshold is exceeded or virtual mode and reading from array
      if ((readings[i] < irThresholds[i] && !virtualMode) ||
          (virtualMode && virtualWallMap[currentCell] & 1 << dir))
      {
        // set wall for current cell
        wallMap[currentCell] |= 1 << dir;
  
        // set wall for opposite cell if valid
        if (oppositeCell >= 0 && oppositeCell < 256)
        {
          wallMap[oppositeCell] |= 1 << ((dir + 2) % 4);
        }
      }
      // unused code for erasing a wall if no wall is detected
      /*else
      {
        wallMap[currentCell] &= ~(1 << dir);
  
        if (oppositeCell >= 0 && oppositeCell < 256)
        {
          wallMap[oppositeCell] &= ~(1 << ((dir + 2) % 4));
        }
      }*/
    }
  }
}

/* Moves to the next cell. */
void makeNextMove ()
{
  unsigned char currentCell = 16 * currentRow + currentCol;
  int nextDir = chooseNextDir(currentCell, mouseDir);
  
  if (debugMode)
  {
    Serial.println(wallMap[currentCell]);
  }

  if (virtualMode)
  {
    switch (nextDir)
    {
      case 0:
        Serial.println("NORTH");
        break;
      case 1:
        Serial.println("EAST");
        break;
      case 2:
        Serial.println("SOUTH");
        break;
      case 3:
        Serial.println("WEST");
        break;
    }
  }
  else
  {
    // turn to face in the next direction
    makeTurn(nextDir);
    mouseDir = nextDir;
  }

  // move forward
  motors->forward();

  // record the new position
  currentRow += offsetMap[nextDir] / 16;
  currentCol += offsetMap[nextDir] % 16;
  mouseDir = nextDir;
}

/* Makes a turn to face in the specified next direction. */
void makeTurn(int nextDir)
{
  // angle to turn in units of 90 deg
  int angle = (4 + nextDir - mouseDir) % 4;

  // if no turn necessary, record that the last move was in the same direction
  if (angle == 0)
  {
    lastWasForward = 1;
    return;
  }
  // else the last move was in a different direction and wait to avoid coasting
  else
  {
    lastWasForward = 0;
    wait(400);
  }
  switch (angle)
  {
    case 1:
      motors->turnRight();
      break;
    case 2:
      // turn around left or right depending on which direction has more space
      if (leftIR -> readRangeSingleMillimeters() < rightIR -> readRangeSingleMillimeters())
      {
        motors->turnAroundLeft();
      }
      else
      {
        motors->turnAroundRight();
      }
      break;
    case 3:
      motors->turnLeft();
      break;
  }
}

/* Chooses the next direction based on the flood-fill algorithm's determination
 * of the adjacent cell which is closest to the destination.
 * Directions have precedence in this order: forward right back left */
int chooseNextDir(int currentCell, int _mouseDir)
{
  // stores the lowest adjacent distance from the destination
  unsigned char lowest = 255;

  // default
  int nextDir = 0;

  // Compare through all the neighbors
  for (int i = 0; i < 4; i++)
  {
    // current direction considered
    int curDir = (_mouseDir + i) % 4;

    /* if the current direction is the lowest so far and there's no wall in the
     * way, set it as the tentative next direction */
    if (cellMap[currentCell + offsetMap[curDir]] < lowest &&
        !(wallMap[currentCell] & 1 << curDir))
    {
      lowest = cellMap[currentCell + offsetMap[curDir]];
      nextDir = curDir;
    }
  }

  return nextDir;
}

/* Moves forward one cell. */
int moveForward()
{
  int nextCell = 16*currentRow + currentCol + offsetMap[mouseDir];
  /* checks if the next cell has been visited, and if so preloads to check if
   * the next action would be forward in the same direction */
  int forwardIsNext = wallMap[nextCell] < 240 &&
                      chooseNextDir(nextCell, mouseDir) == mouseDir &&
                      cellMap[nextCell] != 0;

  // default: accelerate from 60
  int start_pwm = 60;

  /* if we are going forward in the same direction as before, start from the
   * speed where the previous moveForward left off */
  if (lastWasForward)
  {
    start_pwm = 0;
  }

  /* if we are doing a speed run (counter >= 2 means we did at least 2 runs,
   * one to the finish and one back to the start), then accelerate up to max
   * speed, otherwise cap it at 150 */
  if (counter >= 2)
  {
    // nullified function
    return 0;
    //return motors.accForward(start_pwm, 255, 284, 1, !forwardIsNext);
  }
  else
  {
    // obselete function
    return 0;
    //return motors.accForward(start_pwm, 150, 284, 1, !forwardIsNext);
  }
}

/* Action taken when the push-button is released. */
void onButtonRelease()
{
  // set the release flag
  motors->releaseFlag = 1;
  if (debugMode)
  {
    Serial.println("button");
  }
  motors->stop();
  wait(5000);
}

/* * * * * * * * * * * * * * * *
 *          LOGIC CODE         * 
 * * * * * * * * * * * * * * * */

/* Initializes the maze. */
 void initializeMaze ()
{
  // set the robot's position to the start
  currentRow = STARTROW;
  currentCol = STARTCOL;

  // for each cell, set the wallMap value to 240, indicating unvisited
  for (int i = 0; i < 256; i++)
  {
    wallMap[i] = 240;
  }
}

/* Sets up wallMap with the boundary walls of the maze. */
void setBoundaryWalls ()
{
  // NORTH
  for (int i = 0; i < 16; i++)
  {
    wallMap[i] |= SOUTH;
  }
  // EAST
  for (int i = 15; i < 256; i += 16)
  {
    wallMap[i] |= EAST;
  }
  // SOUTH
  for (int i = 240; i < 256; i++)
  {
    wallMap[i] |= NORTH;
  }
  // WEST
  for (int i = 0; i < 241; i += 16)
  {
    wallMap[i] |= WEST;
  }
}

/* Runs the flood-fill algorithm, updating cellMap with distances. */
void floodMaze ()
{
  // reset the array of values
  for (int i = 0; i < 256; i++)
  {
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
  if (counter % 2 == 0)
  { 
    stackPointer = 4;
    cellStack[0] = (16 * ENDROW) + ENDCOL;
    cellStack[1] = (16 * (ENDROW + 1)) + ENDCOL;
    cellStack[2] = (16 * ENDROW) + (ENDCOL + 1);
    cellStack[3] = (16 * (ENDROW + 1)) + (ENDCOL + 1);
  }
  // otherwise, the destination is the start of the maze
  else
  {
    stackPointer = 1;
    cellStack[0] = (16 * STARTROW) + STARTCOL;
  }

  // as long as the stack is non-empty
  while (stackPointer > 0)
  {
    // Stop flooding if the robot's cell has a value
    if (cellMap[16 * currentRow + currentCol] != 255)
    {
      break;
    }

    // Pop the cell off the stack
    unsigned char curCell = cellStack[stackPointer - 1];
    stackPointer--;

    // if the cell has not yet been assigned a distance value
    if (cellMap[curCell] == 255)
    {
      // Set the current cell value to the step path value
      cellMap[curCell] = stepValue;
      
      // Serial.print ("Flood Cell: %d\n", curCell);
  
      // Add all unvisited, available uneighbors to the stack for the next step
      for (int i = 0; i < 4; i++)
      {
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
    if (stackPointer == 0)
    {
      // move the next stack to the main stack
      for (int i = 0; i < nextStackPointer; i++)
      {
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

    if (debugMode == 2)
    {
      printMaze();
    }
  }
}

/* Prints the maze. */
void printMaze()
{
  for (int i = 0; i < 16; i++)
  {
    Serial.print ("---\t");
  }
  for (int i = 15; i >= 0; i--)
  {
    Serial.print ("\n");
    for (int j = 0; j < 16; j++)
    {
      if (currentRow == i && currentCol == j)
      {
        Serial.print("@");
      }
      Serial.print(cellMap[16 * i + j]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }
  for (int i = 0; i < 16; i++)
  {
    Serial.print ("---\t");
  }
  Serial.print ("\n");
}
