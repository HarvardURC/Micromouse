#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Motors.h>

/* Global Constants */
// For Setting Wall bits in the wall array
#define NORTH 1
#define EAST  2
#define SOUTH 4
#define WEST  8

// Starting and ending position
#define STARTROW 0
#define STARTCOL 0
#define ENDROW 3
#define ENDCOL 3

int debugMode = 0;
int virtualMode = 0;

// Calibration for turns -- needs to change
int RIGHT_TURN_STEPS = 54;
int LEFT_TURN_STEPS = 53;
int MOVE_FORWARD_STEPS = 55;
int STEP_DELAY = 2;

//define motor pins
Motors motors (9, 11, 2, 3, 10, 12);

// 0 for North
// 1 for East
// 2 for South 
// 3 for West
int mouseDir = 0;

// maps 0-3 direction to array offset
int offsetMap[4] = {16, 1, -16, -1};

//Define Distance Sensor pins
int leftIRPin = A2;
int rightIRPin = A1;
int forwardIRPin = A0;

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
// Global counter to set destination cell
int counter = 0;
// For going fast: 1 if the last move was forward
int lastWasForward = 0;

/* Function Prototypes */
void initializeMaze();
void floodMaze();
void setBoundaryWalls ();
void setTestWalls();
void makeNextMove();
void turnRight();
void turnLeft();
void moveForward();
void makeNextMove();
int irReading(int pin);
void readCell();
void debugBlink(int times);
void printMaze();

void setup() 
{
  /* * * * * * * * * * * * * * * * * 
   * MOUSE HARDWARE INTITALIZATION *
   * * * * * * * * * * * * * * * * **/
  Serial.begin(9600);
  
  pinMode(13, OUTPUT);
  pinMode(forwardIRPin, INPUT);
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
  
  /* * * * * * * * * * * * * * *
   * FLOODFILL INITIALIZATION  *
   * * * * * * * * * * * * * * */
  // Initialize the maze
  initializeMaze ();
  // Set maze boundary walls
  setBoundaryWalls ();

  if (virtualMode)
  {
    for (int i = 5; i > 0; i--) {
      Serial.println(i);
      delay(1000);
    }
  }
}

void loop()
{

  readCell();

  if (debugMode)
  {
    debugBlink(currentRow);
    delay(1000);
    debugBlink(currentCol);
    delay(1000);
    debugBlink(wallMap[16 * currentRow + currentCol]);
    delay(1000);
    debugBlink(mouseDir);
  }

  floodMaze();
  if (virtualMode)
  {
    printMaze();
  }

  makeNextMove();

  if(cellMap[(16*currentRow) + currentCol] == 0)
  {
    counter++;
  }

  if (virtualMode)
  {
    delay(1000);
  }

  //delay(500);

}

void debugBlink(int times) {
  for (int i = 0; i < times; i++)
  {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }
}

/* * * * * * * * * * * * * * * *
 *       HARDWARE CODE         *
 * * * * * * * * * * * * * * * */

int irReading(int pin)
{
  int sum = 0;
  sum += analogRead(pin);
  wait(1);
  sum += analogRead(pin);
  wait(1);
  sum += analogRead(pin);
  return sum / 3;
}

void readCell()
{
  int irThresholds[4] = {240, 140, 1023, 140};
  
  int readings[4] = {irReading(forwardIRPin),
                     irReading(rightIRPin),
                     0,
                     irReading(leftIRPin)};

  unsigned char currentCell = 16 * currentRow + currentCol;

  for (int i = 0; i < 4; i++)
  {
    int dir = (mouseDir + i) % 4;
    
    if ((readings[i] > irThresholds[i] && !virtualMode) ||
        (virtualMode && virtualWallMap[currentCell] & 1 << dir))
    {
      
      // set wall for current cell
      wallMap[currentCell] |= 1 << dir;

      // set wall for opposite cell if valid
      int oppositeCell = currentCell + offsetMap[dir];
      if (oppositeCell >= 0 && oppositeCell < 256) {
        wallMap[oppositeCell] |= 1 << ((dir + 2) % 4);
 
      }
    }
  }
}

void makeNextMove ()
{ 
  int nextDir = 0;
  // Get the current cell
  unsigned char currentCell = 16 * currentRow + currentCol;
  
  if (debugMode)
  {
    Serial.println(wallMap[currentCell]);
  }

  if (!virtualMode)
  {
    int lowThreshold = 150;
    int highThreshold = 320;
  
    int rightReading = irReading(rightIRPin);
    int leftReading = irReading(leftIRPin);
  
    int isRightWall = wallMap[currentCell] & 1 << (mouseDir + 1) % 4;
    int isLeftWall = wallMap[currentCell] & 1 << (mouseDir + 3) % 4;
  
    // if too close to or too far from a side wall, bump it
    if (isRightWall && rightReading > highThreshold)
    {
      motors.turnLeft();
      motors.wallOrientateBkwd();
      motors.turnRight();
    }
    else if (isLeftWall && leftReading > highThreshold)
    {
      motors.turnRight();
      motors.wallOrientateBkwd();
      motors.turnLeft();
    }
    else if (isRightWall && rightReading < lowThreshold)
    {
      motors.turnRight();
      motors.wallOrientateFwd();
      motors.turnLeft();
    }
    else if(isLeftWall && leftReading < lowThreshold)
    {
      motors.turnLeft();
      motors.wallOrientateFwd();
      motors.turnRight();
    }
  }
  
  // Define a default, very high step value
  unsigned char lowest = 255;

  // Compare through all the neighbors
  for (int i = 0; i < 4; i++)
  {
    int curDir = (mouseDir + i) % 4;
    if (cellMap[currentCell + offsetMap[curDir]] < lowest && !(wallMap[currentCell] & 1 << curDir))
    {
      lowest = cellMap[currentCell + offsetMap[curDir]];
      nextDir = curDir;
    }
  }

  currentRow += offsetMap[nextDir] / 16;
  currentCol += offsetMap[nextDir] % 16;

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
    if (wallMap[currentCell] & 1 << mouseDir)
    {
      motors.wallOrientateFwd();
    }
    makeTurn(nextDir);
    if (wallMap[currentCell] & 1 << (nextDir + 2) % 4)
    {
      motors.wallOrientateBkwd();
    }
    moveForward();
  }
  mouseDir = nextDir;
}

void makeTurn(int nextDir)
{
  lastWasForward = 0;
  switch ((4 + nextDir - mouseDir) % 4)
  {
    case 0:
      lastWasForward = 1;
      break;
    case 1:
      motors.turnRight();
      break;
    case 2:
      if (irReading(leftIRPin) < irReading(rightIRPin))
      {
        motors.turnAroundLeft();
      }
      else
      {
        motors.turnAroundRight();
      }
      break;
    case 3:
      motors.turnLeft();
      break;
  }
}


void moveForward() {
  if (counter >= 2)
  {
    motors.accForward(100, 255, 284, forwardIRPin);
  }
  else
  {
    motors.forward(60, 284, forwardIRPin);
  }
}





/* * * * * * * * * * * * * * * *
 *          LOGIC CODE         * 
 * * * * * * * * * * * * * * * */


void initializeMaze ()
{
  // Initialize misc variables
  int stepValue = 0;

  currentRow = STARTROW;
  currentCol = STARTCOL;
  
  for (int i = 0; i < 255; i++)
  {
    cellMap[i] = 255;
    wallMap[i] = 0;
  }
}

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
  // int to serve as a pointer to the top of the stack
  // 0 means the stack is empty
  int stackPointer, nextStackPointer;
  
    // Initialize pointers to the top of each stack
  stackPointer = 0;
  nextStackPointer = 0;

if(counter % 2 == 0) { 
  //initial path to centre
    stackPointer = 4;
    cellStack[0] = (16 * ENDROW) + ENDCOL;
    cellStack[1] = (16 * (ENDROW + 1)) + ENDCOL;
    cellStack[2] = (16 * ENDROW) + (ENDCOL + 1);
    cellStack[3] = (16 * (ENDROW + 1)) + (ENDCOL + 1);
  }
  else {
  //path back to start
    stackPointer = 1;
    cellStack[0] = (16 * STARTROW) + STARTCOL;
   }
  
  while (stackPointer > 0)
  {
    
    // Stop flooding if our cell has a value
    if (cellMap[16 * currentRow + currentCol] != 255)
    {
      break;
    }

    // Pop the cell off the stack
    unsigned char curCell = cellStack[stackPointer - 1];
    stackPointer--;
    
    if (cellMap[curCell] == 255)
    {
      // Set the current cell value to the step path value
      cellMap[curCell] = stepValue;
      
      // Serial.print ("Flood Cell: %d\n", curCell);
  
      // Add all unvisited, available uneighbors to the stack for the next step
      for (int i = 0; i < 4; i++)
      {
        unsigned char adjCell = curCell + offsetMap[i];
        if (adjCell >= 0 && adjCell < 256 && cellMap[adjCell] == 255 && (wallMap[curCell] & 1 << i) == 0)
        {
          nextCellStack[nextStackPointer] = adjCell;
          nextStackPointer++;
        }
      }
    }
    
    // if the stack is empty, move on to the next step
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

// Print the maze
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
