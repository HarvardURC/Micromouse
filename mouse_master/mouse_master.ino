#include <DistanceGP2Y0A21YK.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

int RIGHT_THRESHOLD = 1;
int LEFT_THRESHOLD = 1;
int FORWARD_THRESHOLD = 1;

int leftThreshold = 300;
int rightThreshold = 300;

//Define Stepper Pins
// Arduino pins -- will change
int dirpinLeft = 6;
int steppinLeft = 4;
int dirpinRight = 3;
int steppinRight = 2;

// mousePos = 16 * row + col
int mousePos = 0;

// 0 for North
// 1 for West
// 2 for East 
// 3 for South
int mouseDir = 0;

//Define Distance Sensor pins
int leftIRPin = A1;
int rightIRPin = A2;
int centerIRPin = A3;
// Initializes the sensor class
DistanceGP2Y0A21YK leftIR;
DistanceGP2Y0A21YK rightIR;
DistanceGP2Y0A21YK forwardIR;

/* Global Variables */
// Global array to store the cell values
unsigned char cellMap[255];
// Global array to store the wall bits
unsigned char wallMap[255];
// Global char to store current distance from end
unsigned char stepValue;
// Global array to act as stack
unsigned char cellStack[255];
// Global array to act as temporary storage
unsigned char tempCellStack[255];
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

void setup() 
{
  /* * * * * * * * * * * * * * * * * 
   * MOUSE HARDWARE INTITALIZATION *
   * * * * * * * * * * * * * * * * **/
  Serial.begin(9600);
  // initialize steppers to outputs
  pinMode(dirpinLeft, OUTPUT);
  pinMode(steppinLeft, OUTPUT);
  pinMode(dirpinRight, OUTPUT);
  pinMode(steppinRight, OUTPUT);
  
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

	floodMaze();

	makeNextMove();

}

/* * * * * * * * * * * * * * * *
 *       HARDWARE CODE         *
 * * * * * * * * * * * * * * * */

void readCell()
{

	int leftDistance = leftIR.getDistanceRaw();
	int rightDistance = rightIR.getDistanceRaw();
	int forwardDistance = forwardIR.getDistanceRaw();

	/* Bitshifting, but depends on direction
	wallMap[16 * endRow + endCol] |= NORTH;
	wallMap[16 * endRow + endCol] |= WEST;
	wallMap[16 * endRow + endCol] |= SOUTH;
	*/

	if (leftDistance > LEFT_THRESHOLD) 
	{
	   // Set the "left" wall depends on where mouse is facing
	}

	if (rightDistance > RIGHT_THRESHOLD) 
	{
          // Same
	}

	if (forwardDistance > FORWARD_THRESHOLD) 
	{
          // Same
	}  
}

void makeNextMove ()
{
	// Get the current cell
	unsigned char currentCell = 16 * currentRow + currentCol;
	
	// Store the current cell
	int tempCurrentRow = currentRow;
	int tempCurrentCol = currentCol;
	
	// Define a default, very high step value
	unsigned char lowest = 255;

	// Compare through all the neighbors
	// NORTH
	if (cellMap[currentCell + 16] < lowest && (tempCurrentRow + 1) < 16)
	{
		int nextDir = SOUTH;

		

		lowest = cellMap[currentCell + 16];
		currentRow = tempCurrentRow + 1;
		currentCol = tempCurrentCol;
	}
	// EAST
	if (cellMap[currentCell + 1] < lowest && (tempCurrentCol + 1) < 16)
	{
		double nextDir = EAST;
		lowest = cellMap[currentCell + 1];
		currentRow = tempCurrentRow;
		currentCol = tempCurrentCol + 1;
	}
	// SOUTH
	if (cellMap[currentCell - 16] < lowest && (tempCurrentRow - 1) > 0)
	{
		double nextDir = NORTH;
		lowest = cellMap[currentCell - 16];
		currentRow = tempCurrentRow - 1;
		currentCol = tempCurrentCol;
	}
	// WEST
	if (cellMap[currentCell - 1] < lowest && (tempCurrentCol - 1) > 0)
	{
		double nextDir = WEST;
		lowest = cellMap[currentCell - 1];
		currentRow = tempCurrentRow;
		currentCol = tempCurrentCol - 1;
	}
}

int getWallShift(int wallNum)
{
	/* WALL NUMS 1 - LEFT, 2 - FRONT, 3 - RIGHT */
	switch(mouseDir)
	{
		case 1:
			switch(wallNum)
			{
				case 1:
					return 8;
					break;
				case 2:
					return 1;
					break;
				case 3:
					return 2;
					break;
				default:
					break;
			}
			break;
		case 2:
			switch(wallNum)
			{
				case 1:
					return 1;
					break;
				case 2:
					return 2;
					break;
				case 3:
					return 4;
					break;
				default:
					break;
			}
			break;
		case 4:
			switch(wallNum)
			{
				case 1:
					return 2;
					break;
				case 2:
					return 4;
					break;
				case 3:
					return 8;
					break;
				default:
					break;
			}
			break;
		case 8:
			switch(wallNum)
			{
				case 1:
					return 4;
					break;
				case 2:
					return 8;
					break;
				case 3:
					return 1;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

void makeTurn(int nextDir)
{
	switch(mouseDir)
	{
		case 1:
			switch(nextDir)
			{
				case 2:
					turnRight();
					break;
				case 4:
					turnAround();
					break;
				case 8:
					turnLeft();
					break;
				default:
					break;
			}
			break;
		case 2:
			switch(nextDir)
			{
				case 1:
					turnLeft();
					break;
				case 4:
					turnRight();
					break;
				case 8:
					turnAround();
					break;
				default:
					break;
			}
			break;
		case 4:
			switch(nextDir)
			{
				case 1:
					turnAround();
					break;
				case 2:
					turnLeft();
					break;
				case 8:
					turnRight();
					break;
				default:
					break;
			}
			break;
		case 8:
			switch(nextDir)
			{
				case 1:
					turnRight();
					break;
				case 2:
					turnAround();
					break;
				case 4:
					turnLeft();
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

void turnRight()
{
	digitalWrite(dirpinLeft, LOW);
	digitalWrite(dirpinRight, LOW);

	for (int i; i < RIGHT_TURN_STEPS; i++)
	{
		digitalWrite(steppinLeft, HIGH);   
		delay(STEP_DELAY);             
		digitalWrite(steppinLeft, LOW);   
		delay(STEP_DELAY);              
		digitalWrite(steppinRight, HIGH);  
		delay(STEP_DELAY);             
		digitalWrite(steppinRight, LOW);    
		delay(STEP_DELAY); 
	}
}

void turnLeft()
{
	digitalWrite(dirpinLeft, HIGH);
	digitalWrite(dirpinRight, HIGH);

	for (int i; i < LEFT_TURN_STEPS; i++){
		digitalWrite(steppinLeft, HIGH);   
		delay(STEP_DELAY);             
		digitalWrite(steppinLeft, LOW);   
		delay(STEP_DELAY);              
		digitalWrite(steppinRight, HIGH);  
		delay(STEP_DELAY);             
		digitalWrite(steppinRight, LOW);    
		delay(STEP_DELAY); 
	}
}

void moveForward() {
	digitalWrite(dirpinLeft, LOW);
	digitalWrite(dirpinRight, HIGH);

	for (int i; i < MOVE_FORWARD_STEPS; i++){
		digitalWrite(steppinLeft, HIGH);   
		delay(STEP_DELAY);             
		digitalWrite(steppinLeft, LOW);   
		delay(STEP_DELAY);              
		digitalWrite(steppinRight, HIGH);  
		delay(STEP_DELAY);             
		digitalWrite(steppinRight, LOW);    
		delay(STEP_DELAY); 
	}
}

void turnAround() 
{
	turnRight();
	turnRight();
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
		wallMap[i] |= NORTH;
	}
	// EAST
	for (int i = 15; i < 256; i += 16)
	{
		wallMap[i] |= EAST;
	}
	// SOUTH
	for (int i = 240; i < 256; i++)
	{
		wallMap[i] |= SOUTH;
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
    
    if (cellMap[floodCell] != 255)
    {
		if(stackPointer == 1)
		{
		}
		else
		{
			stackPointer--;
		}
	}
	else
	{
		// Set the current cell value to the step path value
		cellMap[floodCell] = stepValue;
		
		// printf ("Flood Cell: %d\n", floodCell);

		// Add all unvisited, available uneighbors to a temporary stack stack
		
		// Get all wall bits
		unsigned char checkNorthWall = wallMap[floodCell] & (1 << 3);
		unsigned char checkEastWall = wallMap[floodCell] & (1 << 2);
		unsigned char checkSouthWall = wallMap[floodCell] & (1 << 1);
		unsigned char checkWestWall = wallMap[floodCell] & (1 << 0);
		
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
  
  /* Print the maze
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
	*/
  }
}
