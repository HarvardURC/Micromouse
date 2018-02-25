#include <QueueArray.h>

#include <config.h>

#define NORTH 8
#define EAST 4
#define SOUTH 2
#define WEST 1

#define CENTER_ROW 7
#define CENTER_COL 7

#define START_ROW 0
#define START_COL 0

bool findMinotaur = true;

int mouseRow = 0;
int mouseCol = 0;

struct Cell {
  int row;
  int column;
  int floodDistance;
  bool visited;
  unsigned char walls;
};

// CONVERT TO CELL MAP LATER
unsigned char virtualWalls[256];

Cell cellMap[16][16];

QueueArray<Cell> floodQueue;

void setBoundaryWalls();
void printVirtualMaze();
void printVirtualRow(int row, bool isPostRow);
void initializeCellMap();
void floodMaze();
bool checkWall(int row, int col, int dir);
void Janus();
void initializeFloodMaze();

// Mouse chooses cell to move to
void Janus() {
  if (findMinotaur && cellMap[mouseRow][mouseCol].floodDistance == 0) {
    Serial.println("we're here");
    return;
  }
  
  // Initialize with arbitrarily large values
  // floodDistance of surrounding cells
  // 0 NORTH
  // 1 EAST
  // 2 SOUTH
  // 3 WEST
  int choices[4] = {1000, 1000, 1000, 1000};

  // TODO check for walls
  // NORTH
  if (!checkWall(mouseRow, mouseCol, NORTH)) {
    choices[0] = cellMap[mouseRow - 1][mouseCol].floodDistance;
  }
  // EAST
  if (!checkWall(mouseRow, mouseCol, EAST)) {
    choices[1] = cellMap[mouseRow][mouseCol + 1].floodDistance;
  }
  // SOUTH
  if (!checkWall(mouseRow, mouseCol, SOUTH)) {
    choices[2] = cellMap[mouseRow + 1][mouseCol].floodDistance;
  }
  // WEST
  if (!checkWall(mouseRow, mouseCol, WEST)) {
    choices[3] = cellMap[mouseRow][mouseCol - 1].floodDistance;
  }

  // thePath stores the cell with lowest 
  int minDistance = 1000;
  int thePath = 0;
  for (int i = 0; i < 4; i++) {
    if (choices[i] < minDistance) {
      minDistance = choices[i];
      thePath = i;
    }
  }

  // Virtual movement
  if (thePath == 0) {
    mouseRow--;
  } 
  else if (thePath == 1) {
    mouseCol++;
  }
  else if (thePath == 2) {
    mouseRow++;
  }
  else if (thePath == 3) {
    mouseCol--;
  }
}

void floodMaze() {
  initializeFloodMaze();

  // While there are cells left in the queue
  while (!floodQueue.isEmpty()) {
    
    // Removes first cell from the queue
    Cell frontier = floodQueue.dequeue();
    
    int fRow = frontier.row;
    int fCol = frontier.column;
    int fDistance = frontier.floodDistance;
    
    // Check Northern Cell
    if (fRow > 0 && !cellMap[fRow - 1][fCol].visited && !checkWall(fRow, fCol, NORTH)) {
      if (fDistance + 1 < cellMap[fRow - 1][fCol].floodDistance) {
        cellMap[fRow - 1][fCol].floodDistance = fDistance + 1;
      }
        cellMap[fRow - 1][fCol].visited = true;
        floodQueue.enqueue(cellMap[fRow - 1][fCol]);
    }
    
    // Check Eastern Cell
    if (fCol < 15 && !cellMap[fRow][fCol + 1].visited && !checkWall(fRow, fCol, EAST)) {
      if (fDistance + 1 < cellMap[fRow][fCol + 1].floodDistance) {
        cellMap[fRow][fCol + 1].floodDistance = fDistance + 1;
      }
        cellMap[fRow][fCol + 1].visited = true;
        floodQueue.enqueue(cellMap[fRow][fCol + 1]);
    }
    
    // Check Southern Cell
    if (fRow < 15 && !cellMap[fRow + 1][fCol].visited && !checkWall(fRow, fCol, SOUTH)) {
      if (fDistance + 1 < cellMap[fRow + 1][fCol].floodDistance) {
        cellMap[fRow + 1][fCol].floodDistance = fDistance + 1;
      }
        cellMap[fRow + 1][fCol].visited = true;
        floodQueue.enqueue(cellMap[fRow + 1][fCol]);
    }
      
    // Check Western Cell
    if (fCol > 0 && !cellMap[fRow][fCol - 1].visited && !checkWall(fRow, fCol, WEST)) {
      if (fDistance + 1 < cellMap[fRow][fCol - 1].floodDistance) {
        cellMap[fRow][fCol - 1].floodDistance = fDistance + 1;
      }
        cellMap[fRow][fCol - 1].visited = true;
        floodQueue.enqueue(cellMap[fRow][fCol - 1]);
    }
  }
}

// Returns TRUE if wall exists, FALSE if not
bool checkWall(int row, int col, int dir) {
  if (dir == NORTH) {
    if (row > 0) {
      if ((cellMap[row - 1][col].walls & SOUTH) != SOUTH && (cellMap[row][col].walls & NORTH) != NORTH) {
        return false;
      }
    }
    return true;
  } else if (dir == EAST) {
    if (col < 15) {
      if ((cellMap[row][col + 1].walls & WEST) != WEST && (cellMap[row][col].walls & EAST) != EAST) {
        return false;
      }
    }
    return true;
  } else if (dir == SOUTH) {
    if (row < 15) {
      if ((cellMap[row + 1][col].walls & NORTH) != NORTH && (cellMap[row][col].walls & SOUTH) != SOUTH) {
        return false;
      }
    }
    return true;
  } else if (dir == WEST) {
    if (col > 0) {
      if ((cellMap[row][col - 1].walls & EAST) != EAST && (cellMap[row][col].walls & WEST) != WEST) {
        return false;
      }
    }
    return true;
  }

  return false;
}

void initializeFloodMaze() {
  // When we run the flood maze, none of the cells should be visited already
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      cellMap[i][j].visited = false;
    }
  }

  // Adham, in case you think this is a bad variable name.
  // Minotaur was at the center of the Labyrinth, so by finding the Minotaur we are finding the center of the maze 
  if (findMinotaur) {
    // Set center to 4 cells in the middle
    cellMap[CENTER_ROW][CENTER_COL].floodDistance = 0;
    cellMap[CENTER_ROW][CENTER_COL].visited = true;
    floodQueue.enqueue(cellMap[CENTER_ROW][CENTER_COL]);
  
    cellMap[CENTER_ROW + 1][CENTER_COL].floodDistance = 0;
    cellMap[CENTER_ROW + 1][CENTER_COL].visited = true;
    floodQueue.enqueue(cellMap[CENTER_ROW + 1][CENTER_COL]);
  
    cellMap[CENTER_ROW][CENTER_COL + 1].floodDistance = 0;
    cellMap[CENTER_ROW][CENTER_COL + 1].visited = true;
    floodQueue.enqueue(cellMap[CENTER_ROW][CENTER_COL + 1]);
  
    cellMap[CENTER_ROW + 1][CENTER_COL + 1].floodDistance = 0;
    cellMap[CENTER_ROW + 1][CENTER_COL + 1].visited = true;
    floodQueue.enqueue(cellMap[CENTER_ROW + 1][CENTER_COL + 1]);
  } else {
    // Set objective cell to start (for runs where we want to reset the robot's position)
    cellMap[START_ROW][START_COL].floodDistance = 0;
    cellMap[START_ROW][START_COL].visited = true;
    floodQueue.enqueue(cellMap[START_ROW][START_COL]);
  }
}

void initializeCellMap() {
  for (short i = 0; i < 16; i++) {
    for (short j = 0; j < 16; j++) {
      Cell temp;
      temp.row = i;
      temp.column = j;
      temp.floodDistance = 10000;
      temp.visited = false;
      temp.walls = 0;

      // CALL SET BOUNDARY WALLS FOR TEMP.WALLS
      cellMap[i][j] = temp;
    }
  }
}

// SETUP

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay(1000);
  Serial.println("We're setting up the map!");
  Serial.println("I'm a map");

  // Initialize Cell values in the CellMap
  initializeCellMap();

  // SET UP BOUNDARY WALLS!!!!! :)
  setBoundaryWalls();

  // I dont want to build a virtual maze generation algo
  cellMap[6][7].walls |= NORTH;
  cellMap[0][5].walls |= WEST;
  cellMap[1][5].walls |= WEST;
  cellMap[2][5].walls |= WEST;
  cellMap[3][5].walls |= WEST;
  cellMap[4][5].walls |= WEST;
  cellMap[5][5].walls |= WEST;
  cellMap[3][4].walls |= SOUTH;
}

// LOOP

void loop() {
  floodMaze();

  Janus();
  //mouseRow += 1;

  printVirtualMaze();

  delay(2000);
}

// REWRITE FOR 2 DIMENSIONAL
void setBoundaryWalls() {
  // NOTE: Top of the maze is set to the the 0th row
    for (int i = 0; i < 16; i++) {
      for (int j = 0; j < 16; j++) {
        if (i == 0) {
          cellMap[i][j].walls |= NORTH;
        }
        if (j == 15) {
          cellMap[i][j].walls |= EAST;
        }
        if (i == 15) {
          cellMap[i][j].walls |= SOUTH;
        }
        if (j == 0) {
          cellMap[i][j].walls |= WEST;
        }
      }
    }
}

void printVirtualMaze() {
  for (int row = 0; row < 16; row++) {
    printVirtualRow(row, true);
    printVirtualRow(row, false);
  }

  // print very bottom row
  Serial.println("+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+");
}

// isPost boolean variables flags whether we are printing a row with vertical walls or horizontal walls
void printVirtualRow(int row, bool isPostRow) {
  for (int col = 0; col < 16; col++) {
    
    if (isPostRow) {
      Serial.print("+");

      if (((cellMap[row][col].walls & NORTH) == NORTH) || (row > 0 && (cellMap[row - 1][col].walls & SOUTH) == SOUTH)) {
        Serial.print("---");
      } 
      else {
        Serial.print("   ");
      }
    }
    else {
      if (((cellMap[row][col].walls & WEST) == WEST) || (col > 0 && (cellMap[row][col - 1].walls & EAST) == EAST)) {
        Serial.print("|");
      } else {
        Serial.print(" ");
      }

      if (row == mouseRow && col == mouseCol) {
        Serial.print("@@@");
      }
      else {
        Serial.print(" ");
        Serial.print(cellMap[row][col].floodDistance);
        if (cellMap[row][col].floodDistance < 10) {
          Serial.print(" ");
        }
       
      }
    }
  }

  if (isPostRow) {
    Serial.print("+");
  } else {
    Serial.print("|");
  }

  Serial.println();
}








