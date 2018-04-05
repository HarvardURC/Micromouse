#include <QueueArray.h>

#include <VL6180X.h>
#include <config.h>
#include <emile_motors.h>
#include <i2c_t3.h>
#include <vector>

#define NORTH 8
#define EAST 4
#define SOUTH 2
#define WEST 1

#define CENTER_ROW 7
#define CENTER_COL 7

#define START_ROW 0
#define START_COL 0

#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16

int mouseRow = 0;
int mouseCol = 0;

bool mappingRun = true;

// 0 NORTH
// 1 EAST
// 2 SOUTH
// 3 WEST
int mouseDir = 2;

const int wallThreshold = 280;

struct Cell {
  int row;
  int column;
  int floodDistance;
  bool visited;
  unsigned char walls;
};

// {left, leftDiagonal, front, rightDiagonal, right}
std::vector<int> sensorReadings = {1000, 1000, 1000, 1000, 1000};

// CONVERT TO CELL MAP LATER
unsigned char virtualWalls[256];

Cell cellMap[MAZE_HEIGHT][MAZE_WIDTH];

QueueArray<Cell> floodQueue;

void setBoundaryWalls();
void printVirtualMaze();
void printVirtualRow(int row, bool isPostRow);
void initializeCellMap();
void floodMaze();
bool checkWall(int row, int col, int dir);
void senseWalls();
void Janus();
void initializeFloodMaze();
void initSensor(int pin, VL6180X *sensor, int address);
void turn(int desired);
void store(int nextMovement);

// initialize sensors
std::vector<int> sensor_pins = {pins::tofLeft, pins::tofLeftDiag, pins::tofFront, pins::tofRightDiag, pins::tofRight};
std::vector<VL6180X*> sensors = {new VL6180X, new VL6180X, new VL6180X, new VL6180X, new VL6180X};
std::vector<String> sensor_names = {"left", "leftDiag", "front", "rightDiag", "right"};

// initialize storage array and variables
int quickestPath [150];
int numMoves = 0;
int moveIndex = 0;

// Keep track of which run we are on
// EVEN = run to the center
// ODD = run to the start
int runCounter = 0;

// initialize motors object
emile_motors* motors = new emile_motors(sensors[0], sensors[4], sensors[2], sensors[1], sensors[3]);

// SETUP
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(57600);
  delay(1000);

  // Sets all sensors to low for initialization
  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
    pinMode(sensor_pins[i], OUTPUT);
    digitalWrite(sensor_pins[i], LOW);
  }

  // Initializes sensors
  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
    initSensor(sensor_pins[i], sensors[i], i + 1);
    Serial.println(i);
  }

  Serial.println("We're setting up the map!");
  Serial.println("I'm a map");

  // Initialize Cell values in the CellMap
  initializeCellMap();
  Serial.println("Made it past initializing cell map!");

  // SET UP BOUNDARY WALLS!!!!! :)
  setBoundaryWalls();
  Serial.println("Made it past set boundary walls!");

  // Start runCounter at 0: mapping run
  runCounter = 0;

  // No moves to start
  numMoves = 0;
}

// LOOP
void loop() {
  senseWalls();
  Serial.println("Made it past sense walls");

  // ONLY FLOOD ON THE FIRST RUN
  if (runCounter == 0) {
    floodMaze();
    Serial.println("Made it past floodmaze");
  }

  Janus();
  Serial.println("Made it past janus");

  printVirtualMaze();
  Serial.println("Made it past print virtual maze");

  delay(1000);
}

// Mouse chooses cell to move to
void Janus() {
  if ((runCounter == 0 && cellMap[mouseRow][mouseCol].floodDistance == 0) || (runCounter > 0 && moveIndex == numMoves - 1)) {
    if (runCounter % 2 == 0) {
      Serial.println("Arrived at CENTER");
    } else {
      Serial.println("Arrived at START");
    }
    runCounter++;
    
    // Reverse direction of quickest path
    for (int i = 0; i < numMoves; i++) {
      quickestPath[i] = (quickestPath[i] + 2) % 4;
    }

    // reset to beginning of quickest path
    moveIndex = 0;
  }

  // 0 NORTH
  // 1 EAST
  // 2 SOUTH
  // 3 WEST
  int thePath = 0;

  if (runCounter == 0) {
    int choices[4] = {1000, 1000, 1000, 1000};

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
    for (int i = 0; i < 4; i++) {
      if (choices[i] < minDistance) {
        minDistance = choices[i];
        thePath = i;
      }
    }

    // store direction in quickest path
    store(thePath);

    turn(thePath);
    motors->forward();
  }
  else {
    thePath = quickestPath[moveIndex];
    moveIndex++;

    // ADD QUICKER MOVEMENTS TODO, like moving two cells in 1 move.
    turn(thePath);
    motors->forward();


  }

  Serial.print("thePath: ");
  Serial.println(thePath);
  
  //   Virtual movement
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

void store(int nextMovement) {
    quickestPath[numMoves] = nextMovement;
    numMoves++;
}

void turn(int desired) {
  int difference = (desired - mouseDir + 4) % 4;

  if (difference == 1) {
     motors->turnRight();
  }
  else if (difference == 3) {
     motors->turnLeft();
  }
  else if (difference == 2) {
     motors->turnAroundRight();
  }

  mouseDir = desired;
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
    if (fCol < MAZE_WIDTH - 1 && !cellMap[fRow][fCol + 1].visited && !checkWall(fRow, fCol, EAST)) {
      if (fDistance + 1 < cellMap[fRow][fCol + 1].floodDistance) {
        cellMap[fRow][fCol + 1].floodDistance = fDistance + 1;
      }
        cellMap[fRow][fCol + 1].visited = true;
        floodQueue.enqueue(cellMap[fRow][fCol + 1]);
    }

    // Check Southern Cell
    if (fRow < MAZE_HEIGHT - 1 && !cellMap[fRow + 1][fCol].visited && !checkWall(fRow, fCol, SOUTH)) {
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
    if (col < MAZE_WIDTH - 1) {
      if ((cellMap[row][col + 1].walls & WEST) != WEST && (cellMap[row][col].walls & EAST) != EAST) {
        return false;
      }
    }
    return true;
  } else if (dir == SOUTH) {
    if (row < MAZE_HEIGHT - 1) {
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
  for (int i = 0; i < MAZE_HEIGHT; i++) {
    for (int j = 0; j < MAZE_WIDTH; j++) {
      cellMap[i][j].visited = false;
      cellMap[i][j].floodDistance = 1000;
    }
  }

  // // Adham, in case you think this is a bad variable name.
  // // Minotaur was at the center of the Labyrinth, so by finding the Minotaur we are finding the center of the maze
  // if (runCounter % 2 == 0) {
  //   // Set center to cells in the middle
  //   cellMap[CENTER_ROW][CENTER_COL].floodDistance = 0;
  //   cellMap[CENTER_ROW][CENTER_COL].visited = true;
  //   floodQueue.enqueue(cellMap[CENTER_ROW][CENTER_COL]);
  // } else {
  //   // Set objective cell to start (for runs where we want to reset the robot's position)
  //   cellMap[START_ROW][START_COL].floodDistance = 0;
  //   cellMap[START_ROW][START_COL].visited = true;
  //   floodQueue.enqueue(cellMap[START_ROW][START_COL]);
  // }

  // We only use flood fill on the first run
  cellMap[CENTER_ROW][CENTER_COL].floodDistance = 0;
}

void initializeCellMap() {
  for (short i = 0; i < MAZE_HEIGHT; i++) {
    for (short j = 0; j < MAZE_WIDTH; j++) {
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

// REWRITE FOR 2 DIMENSIONAL
void setBoundaryWalls() {
  // NOTE: Top of the maze is set to the the 0th row
    for (int i = 0; i < MAZE_HEIGHT; i++) {
      for (int j = 0; j < MAZE_WIDTH; j++) {
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
  for (int row = 0; row < MAZE_HEIGHT; row++) {
    printVirtualRow(row, true);
    printVirtualRow(row, false);
  }

  // print very bottom row
  Serial.println("+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+");
}

// isPost boolean variables flags whether we are printing a row with vertical walls or horizontal walls
void printVirtualRow(int row, bool isPostRow) {
  for (int col = 0; col < MAZE_WIDTH; col++) {

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

void senseWalls() {
  // get sensor readings
  for (int i = 0; i < sensors.size(); i++) {
    sensorReadings[i] = sensors[i]->readRangeContinuousMillimeters();
    Serial.println(sensorReadings[i]);
  }
  Serial.print("Mouse Direction: ");
  Serial.println(mouseDir);

  // compare LEFT with threshold
  if (sensorReadings[0] < wallThreshold) {
    // NORTH
    if (mouseDir == 0) {
      cellMap[mouseRow][mouseCol].walls |= WEST;
    }
    // EAST
    else if (mouseDir == 1) {
      cellMap[mouseRow][mouseCol].walls |= NORTH;
    }
    // SOUTH
    else if (mouseDir == 2) {
      cellMap[mouseRow][mouseCol].walls |= EAST;
    }
    // WEST
    else if (mouseDir == 3) {
      cellMap[mouseRow][mouseCol].walls |= SOUTH;
    }
  }

  // compare FRONT with threshold
  if (sensorReadings[2] < wallThreshold) {
    // NORTH
    if (mouseDir == 0) {
      cellMap[mouseRow][mouseCol].walls |= NORTH;
    }
    // EAST
    else if (mouseDir == 1) {
      cellMap[mouseRow][mouseCol].walls |= EAST;
    }
    // SOUTH
    else if (mouseDir == 2) {
      cellMap[mouseRow][mouseCol].walls |= SOUTH;
    }
    // WEST
    else if (mouseDir == 3) {
      cellMap[mouseRow][mouseCol].walls |= WEST;
    }
  }

  // compare RIGHT with threshold
  if (sensorReadings[4] < wallThreshold) {
    // NORTH
    if (mouseDir == 0) {
      cellMap[mouseRow][mouseCol].walls |= EAST;
    }
    // EAST
    else if (mouseDir == 1) {
      cellMap[mouseRow][mouseCol].walls |= SOUTH;
    }
    // SOUTH
    else if (mouseDir == 2) {
      cellMap[mouseRow][mouseCol].walls |= WEST;
    }
    // WEST
    else if (mouseDir == 3) {
      cellMap[mouseRow][mouseCol].walls |= NORTH;
    }
  }
}

void initSensor(int pin, VL6180X *sensor, int address) {
    digitalWrite(pin, HIGH);
    sensor->init();
    sensor->configureDefault();
    sensor->setScaling(2);
    sensor->setAddress(address);
    sensor->setTimeout(500);
    sensor->writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 20);
    sensor->stopContinuous();
    delay(300);
    sensor->startRangeContinuous(30);
    Serial.print(pin);
    Serial.println(" connected.");
}

