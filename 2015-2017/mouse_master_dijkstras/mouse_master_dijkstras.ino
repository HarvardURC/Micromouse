#include "priority_queue.h"

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

/* Global Variables */
// Global array to store the cell values
// Each cell has up, right, down, left
unsigned char cellMap[1024];
// Global array to store the wall bits
unsigned char wallMap[256];
// Global ints for current position
int currentRow, currentCol, currentDir;
// Global ints for center of maze
int goal[8] = {480, 481, 482, 483, 544, 545, 546, 547};

void initializeMaze ();
void setBoundaryWalls ();
int* findAdjacent();

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  priority_queue queue;
  queue.update(0,3);
  queue.update(3,2);
  delay(2000);
  for(int i=0; i<2; i++)
  {
    node popped = queue.pop();
    Serial.print(popped.state);
    Serial.print(", ");
    Serial.print(popped.value);
    Serial.print("\n");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

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

/* Fills the temporary map with distance values given current position */
void floodMap (int pos)
{
  // Create local priority queue to store cells
  priority_queue queue;
  // maybe want to clear the map from last time here
  // thinking about editing priority queue's update function to only add non-visited cells, so we can add freely here
  // when tracing back in findMove (not written), just start at lowest destination cell (out of 4) and loop until the lowest adjacent is pos)
}

/* Determines what positions in the maze are accessible to given position */
/* CURRENTLY DYSFUNCTIONAL, THOUGHT TO TAKE PRIORITY QUEUE POINTER AS PARAMETER AND ADD THE
ADJACENT CELLS DIRECTLY INTO QUEUE, IF THAT IS ALL THIS FUNCTION IS USED FOR MIGHT AS WELL*/
int* findAdjacent (int pos)
{
  // may want to initialize third value to -1 to signal if wall blocks it
  int adjacent[3];
  int dir = pos % 4;
  // There will always be 2 adjacents (90 degree turn) for any given position
  // Third adjacent depends on if there is a wall to block a forward move
  if (dir == 0){
    adjacent[0] = pos + 3;
    adjacent[1] = pos + 1;
    if (!(wallMap[pos] & NORTH << dir)){
      adjacent[2] = pos - 62;
    }
  }
  else if (dir == 1){
    adjacent[0] = pos + 1;
    adjacent[1] = pos - 1;
    if (!(wallMap[pos] & EAST << dir)){
      adjacent[2] = pos + 4;
    }
  }
  else if (dir == 2){
    adjacent[0] = pos + 1;
    adjacent[1] = pos - 1;
    if (!(wallMap[pos] & SOUTH << dir)){
      adjacent[2] = pos + 62;
    }
  }
  else if (dir == 3){
    adjacent[0] = pos - 1;
    adjacent[1] = pos - 3;
    if (!(wallMap[pos] & WEST << dir)){
      adjacent[2] = pos - 4;
    }
  }
  return adjacent;
}

