#include <DistanceGP2Y0A41SK.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Motors.h>

/* Global Constants */
// For Setting Wall bits in the wall array
#define NORTH 1;
#define EAST  2;
#define SOUTH 4;
#define WEST  8;

// Starting and ending position
#define STARTROW 0;
#define STARTCOL 0;
#define ENDROW 7;
#define ENDCOL 7;

// Calibration for turns -- needs to change
int RIGHT_TURN_STEPS = 54;
int LEFT_TURN_STEPS = 53;
int MOVE_FORWARD_STEPS = 55;
int STEP_DELAY = 2;

int irThresholds[4] = {300, 200, 1023, 200};

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
int centerIRPin = A0;
// Initializes the sensor class
DistanceGP2Y0A41SK leftIR;
DistanceGP2Y0A41SK rightIR;
DistanceGP2Y0A41SK forwardIR;

/* Global Variables */
// Global array to store the cell values
unsigned char cellMap[256];
// Global array to store the wall bits
unsigned char wallMap[256];
// Global char to store current distance from end
unsigned char stepValue;
// Global array to act as stack
unsigned char cellStack[256];
// Global array to act as temporary storage
unsigned char tempCellStack[256];
// Global int to serve as a pointer to the top of the stack
// 0 means the stack is empty
unsigned int stackPointer, tempStackPointer;
// Global int to serve as the stack empty flag
unsigned int stackEmptyFlag, tempStackEmptyFlag;
// Global ints for current position
int currentRow, currentCol;
int startRow, startCol, endRow, endCol;

/* Function Prototypes */
void initializeMaze ();
void floodMaze ();
void setBoundaryWalls ();
void setTestWalls ();
void makeNextMove ();
void turnRight();
void turnLeft();
void moveForward();
void makeNextMove();

void debugBlink() {
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
  delay(200);
}

void setup() 
{
  pinMode(13, OUTPUT);
  
  /* * * * * * * * * * * * * * * * * 
   * MOUSE HARDWARE INTITALIZATION *
   * * * * * * * * * * * * * * * * **/
  Serial.begin(9600);
  
  //Start distance sensors
  leftIR.begin(leftIRPin);
  rightIR.begin(rightIRPin);
  forwardIR.begin(centerIRPin);
  
  /* * * * * * * * * * * * * * *
   * FLOODFILL INITIALIZATION  *
   * * * * * * * * * * * * * * */
  // Initialize the maze
  initializeMaze ();
  // Set maze boundary walls
  setBoundaryWalls ();
  
  // Test path length
  int pathLength = 0;
  
}

void loop()
{

	readCell();

  /*for (int i = 0; i < 16 * currentRow + currentCol; i++) {
    debugBlink();
  }

  delay(1000);

  for (int i = 0; i < wallMap[16 * currentRow + currentCol]; i++) {
    debugBlink();
  }*/

	floodMaze();

  Serial.println(cellMap[16 * currentRow + currentCol]);

	makeNextMove();

  delay(500);

}

/* * * * * * * * * * * * * * * *
 *       HARDWARE CODE         *
 * * * * * * * * * * * * * * * */

void readCell()
{
  int readings[4] = {forwardIR.getDistanceRaw(),
                     rightIR.getDistanceRaw(),
                     0,
                     leftIR.getDistanceRaw()};

  for (int i = 0; i < 4; i++)
  {
    if (readings[i] > irThresholds[i])
    {
      
      int dir = (mouseDir + i) % 4;
      
      // set wall for current cell
      wallMap[16 * currentRow + currentCol] |= 1 << dir;

      // set wall for opposite cell if valid
      int oppositeCell = 16 * currentRow + currentCol + offsetMap[dir];
      if (oppositeCell >= 0 && oppositeCell < 256) {
        wallMap[oppositeCell] |= 1 << ((dir + 2) % 4);
 
      }
/*
      if(i == 0)
      {
        motors.wallOrientate();
      } */
    }
    
  }
}

void makeNextMove ()
{
  int nextDir = 0;
	// Get the current cell
	unsigned char currentCell = 16 * currentRow + currentCol;
	
	// Store the current cell
	int tempCurrentRow = currentRow;
	int tempCurrentCol = currentCol;
	
	// Define a default, very high step value
	unsigned char lowest = 255;

	// Compare through all the neighbors
	// NORTH
	if (cellMap[currentCell + 16] < lowest && (tempCurrentRow + 1) < 16 /*&& wallMap[currentCell] & 1 == 0*/)
	{
		nextDir = NORTH;

		

		lowest = cellMap[currentCell + 16];
		currentRow = tempCurrentRow + 1;
		currentCol = tempCurrentCol;
	}
	// EAST
	if (cellMap[currentCell + 1] < lowest && (tempCurrentCol + 1) < 16)
	{
		nextDir = EAST;
		lowest = cellMap[currentCell + 1];
		currentRow = tempCurrentRow;
		currentCol = tempCurrentCol + 1;
	}
	// SOUTH
	if (cellMap[currentCell - 16] < lowest && (tempCurrentRow - 1) > 0)
	{
		nextDir = SOUTH;
		lowest = cellMap[currentCell - 16];
		currentRow = tempCurrentRow - 1;
		currentCol = tempCurrentCol;
	}
	// WEST
	if (cellMap[currentCell - 1] < lowest && (tempCurrentCol - 1) > 0)
	{
		nextDir = WEST;
		lowest = cellMap[currentCell - 1];
		currentRow = tempCurrentRow;
		currentCol = tempCurrentCol - 1;
	}

  makeTurn(nextDir);
  moveForward();
 
}

void makeTurn(int nextDir)
{
  switch((4 + nextDir - mouseDir) % 4)
  {
    case 1:
      motors.turnRight();
      break;
    case 2:
      motors.turnAround();
      break;
    case 3:
      motors.turnLeft();
      break;
    default:
      break;
  }
  mouseDir = nextDir;
}


void moveForward() {
  motors.forward(60, 284);
}





/* * * * * * * * * * * * * * * *
 *          LOGIC CODE         * 
 * * * * * * * * * * * * * * * */


void initializeMaze ()
{
  // Initialize misc variables
	stepValue = 0;

	startRow = STARTROW;
	startCol = STARTCOL;
	endRow = ENDROW;
	endCol = ENDCOL;
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
  // Set the bottom of each stack to garbage values
  cellStack[0] = 255;
  tempCellStack[0] = 255;

  // Initialize pointers to the top of each stack
  stackPointer = 1;
  tempStackPointer = 1;

  // Push the destination cell onto the stack
  cellStack[stackPointer] = (16 * endRow) + endCol;
  stackPointer++;
  // Signal that the stack is not empty
  stackEmptyFlag = 0;
  // Signal that the stack is indeed empty
  tempStackEmptyFlag = 0;

  while (stackEmptyFlag == 0)
  {
    // Stop flooding if our cell has a value
    if (cellMap[16 * currentRow + currentCol] != 255)
    {
      break;
    }

    // Pop the cell off the stack
    unsigned char floodCell = cellStack[stackPointer - 1];
    
    if (cellMap[floodCell] != 255 && stackPointer >= 1)
    {
		  stackPointer--;
	  }
	  else
  	{
  		// Set the current cell value to the step path value
  		cellMap[floodCell] = stepValue;
  		
  		// printf ("Flood Cell: %d\n", floodCell);
  
  		// Add all unvisited, available uneighbors to a temporary stack stack
  		
  		// Get all wall bits
  		unsigned char checkNorthWall = wallMap[floodCell] & NORTH;
  		unsigned char checkEastWall = wallMap[floodCell] & EAST;
  		unsigned char checkSouthWall = wallMap[floodCell] & SOUTH;
  		unsigned char checkWestWall = wallMap[floodCell] & WEST;
  		
  		// Check NORTH Cell
  		if (checkNorthWall == 0 && cellMap[floodCell + 16] == 255)
  		{
  		  tempCellStack[tempStackPointer] = floodCell + 16;
  		  // Update temp stack pointer
  		  tempStackPointer++;
  		  // Flag that it's no longer empty
  		  tempStackEmptyFlag = 0;
  		}
  		// EAST Cell
  		if (checkEastWall == 0 && cellMap[floodCell + 1] == 255)
  		{
  		  tempCellStack[tempStackPointer] = floodCell + 1;
  		  // Update temp stack pointer
  		  tempStackPointer++;
  		  // Flag that it's no longer empty
  		  tempStackEmptyFlag = 0;
  		}
  		// SOUTH Cell
  		if (checkSouthWall == 0 && cellMap[floodCell - 16] == 255)
  		{
  		  tempCellStack[tempStackPointer] = floodCell - 16;
  		  // Update temp stack pointer
  		  tempStackPointer++;
  		  // Flag that it's no longer empty
  		  tempStackEmptyFlag = 0;
  		}
  		// WEST Cell
  		if (checkWestWall == 0 && cellMap[floodCell - 1] == 255)
  		{
  		  tempCellStack[tempStackPointer] = floodCell - 1;
  		  // Update temp stack pointer
  		  tempStackPointer++;
  		  // Flag that it's no longer empty
  		  tempStackEmptyFlag = 0;
  		}
  	}
	

    // Check if this is the last pointer in the stack
    // If so, switch the empty flag on
    if (stackPointer == 1)
    {
      // Check if the secondary stack is empty
      if (tempStackEmptyFlag == 0)
      {
        // Replace primary stack with secondary stack
        for (int i = 0; i < tempStackPointer; i++)
        {
          cellStack[i] = tempCellStack[i];
        }

        stackPointer = tempStackPointer;

        // Reset secondary stack pointer and flag
        tempStackPointer = 1;
        tempStackEmptyFlag = 1;
        
        // Update the step value
        stepValue++;
      }
      // Otherwise, all stacks are empty
      else
      {
        stackEmptyFlag = 1;
      }

    }
    // If not, simply decrement the pointer value
    else
    {
      stackPointer--;
    }
    
    // Print stack for debug
    /*printf ("Current Stack\n");
    for (int i = 0; i < stackPointer; i++)
    {
		printf ("Stack Member: %d\n", cellStack[i]);
	}
	*/
	
	// system("clear");
  
  // Print the maze
  printf ("  ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---  \n");
	for (int i = 15; i >= 0; i--)
	{
		printf ("\n");
		for (int j = 0; j < 16; j++)
		{
			unsigned char current = cellMap[16* i + j];
			
			if (current < 10 && current < 100)
			{
				if (currentRow == i && currentCol == j)
				{
					printf(" @%d    ",cellMap[16* i + j]);
				}
				else
				{
					printf("  %d    ",cellMap[16* i + j]);
				}
			}
			else if (current < 100)
			{
				if (currentRow == i && currentCol == j)
				{
					printf(" @%d   ",cellMap[16* i + j]);
				}
				else
				{
					printf("  %d   ",cellMap[16* i + j]);
				}
			}
			else
			{
				if (currentRow == i && currentCol == j)
				{
					printf(" @%d  ",cellMap[16* i + j]);
				}
				else
				{
					printf("  %d  ",cellMap[16* i + j]);
				}
			}
		}
		printf("\n");
	}
	printf ("  ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---    ---  \n");
	printf ("\n");

	int j;
	int i;

	for (i=0;i<120000000;i++)
	{
		j=i;
	}
	
  }
}
