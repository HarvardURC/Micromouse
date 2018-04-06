#include <QueueArray.h>

#include <VL6180X.h>
#include <config.h>
#include <emile_motors.h>
#include <i2c_t3.h>
#include <vector>
#include <string>

#define NORTH 8
#define EAST 4
#define SOUTH 2
#define WEST 1

#define CENTER_ROW 2
#define CENTER_COL 0

#define START_ROW 0
#define START_COL 0

bool findMinotaur = true;

int mouseRow = 0;
int mouseCol = 0;

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

Cell cellMap[16][16];

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
void generateWall();
void initSensor(int pin, VL6180X *sensor, int address);
void turn(int desired);
void store(int nextMovement);

// initialize sensors
std::vector<int> sensor_pins = {pins::tofLeft, pins::tofLeftDiag, pins::tofFront, pins::tofRightDiag, pins::tofRight};
std::vector<VL6180X*> sensors = {new VL6180X, new VL6180X, new VL6180X, new VL6180X, new VL6180X};
std::vector<String> sensor_names = {"left", "leftDiag", "front", "rightDiag", "right"};

// initialize speed counter and speed run array
int spd = 0;
int prevSpd = 0;
String speedPathCheck [150];
int speedPath [150];
int speedIndex = 0;

// initialize storage array and variables
int quickestPath [150];
int moveIndex = 0;
bool backAtStart = false;

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
  // Initialize buttons
  pinMode(pins::buttonS8, INPUT);
  pinMode(pins::buttonS6, INPUT);
  Serial.println("We're setting up the map!");
  Serial.println("I'm a map");

  // Initialize Cell values in the CellMap
  initializeCellMap();
  Serial.println("Made it past initializing cell map!");

  // SET UP BOUNDARY WALLS!!!!! :)
  setBoundaryWalls();
  Serial.println("Made it past set boundary walls!");

  // randomSeed(17);
  // generateWall();
}

// LOOP
void loop() {
//  if (spd <= 0) {
    senseWalls();
    Serial.println("Made it past sense walls");
  
    floodMaze();
    Serial.println("Made it past floodmaze");
  
    Janus();
    Serial.println("Made it past janus");
  
    //mouseRow += 1;
  
    printVirtualMaze();
    Serial.println("Made it past print virtual maze");
  
    delay(1000);
    
//  }

    
//   else {
//     while (spd == prevSpd){
//        addSpdCount();
//     }
//     prevSpd = spd;
//     motors->forward();
//     // MODIFY TOP MOTOR SPEED RELATIVE TO SPD
//     //speedrun()
//   }
}

void generateWall() {
  // 0 = NORTH, 1 = EAST, 2 = SOUTH, 3 = WEST
  for (int i = 0; i < 16; i = i + 2) {
    for (int j = 0; j < 16; ++j) {
      int nWalls = random(0, 3);
      for (int k = 0; k < nWalls; ++k) {
        int d = random (0, 3);
        if (d == 0) {
          cellMap[i][j].walls |= NORTH;
        }
        else if (d == 1) {
          cellMap[i][j].walls |= EAST;
        }
        else if (d == 2) {
          cellMap[i][j].walls |= SOUTH;
        }
        else {
          cellMap[i][j].walls |= WEST;
        }
      }
    }
  }
}

// Mouse chooses cell to move to
void Janus() {
  if (findMinotaur && cellMap[mouseRow][mouseCol].floodDistance == 0) {
    Serial.println("we're here");
    for (int i = 0; i < moveIndex; i++) {
      quickestPath[i] = (quickestPath[i] + 2) % 4;
    }
    int beginningArray, endArray;
    int endingReference = moveIndex - 1;
    for (int i = 0; i < moveIndex/2; i++, endingReference--) {
      beginningArray = quickestPath[i];
      endArray = quickestPath[endingReference];
      quickestPath[endingReference] = beginningArray;
      quickestPath[i] = endArray;
      }
    Serial.println("the path back to the start: ");
    if (!backAtStart) {
      for (int i = 0; i < moveIndex; i++) {
        turn(quickestPath[i]);
        Serial.print(quickestPath[i]);
        motors->forward();
      }
      backAtStart = true;
      Serial.println("back!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
    spd++;
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

  // 0 NORTH
  // 1 EAST
  // 2 SOUTH
  // 3 WEST
  // We want thePath == mouseDir

  Serial.print("thePath: ");
  Serial.println(thePath);
  turn(thePath);
  motors->forward();
  store(thePath);

  // TODO update mouseRow and mouseCol in moveForward()

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

void turn(int desired) {
  int difference = (desired - mouseDir + 4) % 4;

  if (difference == 1) {
     motors->turnRight();
     Serial.println("turning right #######################");
  }
  else if (difference == 3) {
     motors->turnLeft();
     Serial.println("turning left @@@@@@@@@@@@@@@@@@@@@");
  }
  else if (difference == 2) {
     motors->turnAroundRight();
     Serial.println("turning around!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
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
      cellMap[i][j].floodDistance = 1000;
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

void store(int nextMovement) {
    quickestPath[moveIndex] = nextMovement;
    speedPath[speedIndex] = nextMovement;
    char buffer [200];
    speedPathCheck[speedIndex] = sprintf(buffer, "%d, %d", mouseRow, mouseCol);
    Serial.println(speedPathCheck[speedIndex]);
    speedIndex++;
    moveIndex++;
    if (speedPathCheck[speedIndex - 2] == speedPathCheck[speedIndex]) {
      speedPathCheck[speedIndex - 2] = speedPathCheck[speedIndex];
      speedPath[speedIndex - 2] = speedPath[speedIndex];
      speedIndex -= 2;
    }
}

void addSpdCount() {
    int addSpd = digitalRead(pins::buttonS8);
    int deleteSpd = -digitalRead(pins::buttonS6);
    spd += addSpd + deleteSpd;
}

void speedRun() {
    for (int i = 0; i < speedIndex; i++) {
      turn(speedPath[i]);
      motors->forward();
    }
    for (int i = 0; i < speedIndex; i++) {
      speedPath[i] = (speedPath[i] + 2) % 4;
    }
    int beginningArray, endArray;
    int endingReference = speedIndex - 1;
    for (int i = 0; i < speedIndex/2; i++, endingReference--) {
      beginningArray = speedPath[i];
      endArray = speedPath[endingReference];
      speedPath[endingReference] = beginningArray;
      speedPath[i] = endArray;
      }
    for (int i = 0; i < speedIndex; i++) {
      turn(speedPath[i]);
      Serial.print(speedPath[i]);
      motors->forward();
    }
}

