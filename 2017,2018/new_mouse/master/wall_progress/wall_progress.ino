#include <QueueArray.h>

#include <vector>
#include <VL6180X.h>
#include <i2c_t3.h>
#include <config.h>

#define NORTH 8
#define EAST 4
#define SOUTH 2
#define WEST 1

#define CENTER_ROW 7
#define CENTER_COL 7

std::vector<int> sensor_pins = {pins::tofLeft, pins::tofLeftDiag, pins::tofFront, pins::tofRightDiag, pins::tofRight};
std::vector<VL6180X*> sensors = {new VL6180X, new VL6180X, new VL6180X, new VL6180X, new VL6180X};
std::vector<String> sensor_names = {"left", "leftDiag", "front", "rightDiag", "right"};

const int threshold = 250;

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

void initSensor(int pin, VL6180X *sensor, int address);
void setBoundaryWalls();
void printVirtualMaze();
void initializeCellMap();
void floodMaze();
void initializeFloodMaze();

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
    if (fRow > 0) {
      // If the Northern Cell is not visited, and there is no wall between the frontier and the Northern Cell, enqueue the Northern Cell
      if (!cellMap[fRow - 1][fCol].visited && (cellMap[fRow - 1][fCol].walls & SOUTH) != SOUTH && (frontier.walls & NORTH) != NORTH) {
        if (fDistance + 1 < cellMap[fRow - 1][fCol].floodDistance) {
          cellMap[fRow - 1][fCol].floodDistance = fDistance + 1;
        }
        cellMap[fRow - 1][fCol].visited = true;
        floodQueue.enqueue(cellMap[fRow - 1][fCol]);
      }
    }

    // Check Eastern Cell
    if (fCol < 15) {
      if (!cellMap[fRow][fCol + 1].visited && (cellMap[fRow][fCol + 1].walls & WEST) != WEST && (frontier.walls & EAST) != EAST) {
        if (fDistance + 1 < cellMap[fRow][fCol + 1].floodDistance) {
          cellMap[fRow][fCol + 1].floodDistance = fDistance + 1;
        }
      cellMap[fRow][fCol + 1].visited = true;
      floodQueue.enqueue(cellMap[fRow][fCol + 1]);
      }
    }
    
    // Check Southern Cell
    if (fRow < 15) {
      // If the Southern Cell is not visited, and there is no wall between the frontier and the Southern Cell, enqueue the Southern Cell
      if (!cellMap[fRow + 1][fCol].visited && (cellMap[fRow + 1][fCol].walls & NORTH) != NORTH && (frontier.walls & SOUTH) != SOUTH) {
        if (fDistance + 1 < cellMap[fRow + 1][fCol].floodDistance) {
          cellMap[fRow + 1][fCol].floodDistance = fDistance + 1;
        }
        cellMap[fRow + 1][fCol].visited = true;
        floodQueue.enqueue(cellMap[fRow + 1][fCol]);
      }
    }

    // Check Western Cell
    if (fCol > 0) {
      if (!cellMap[fRow][fCol - 1].visited && (cellMap[fRow][fCol - 1].walls & EAST) != EAST && (frontier.walls & WEST) != WEST) {
        if (fDistance + 1 < cellMap[fRow][fCol - 1].floodDistance) {
          cellMap[fRow][fCol - 1].floodDistance = fDistance + 1;
        }
        cellMap[fRow][fCol - 1].visited = true;
        floodQueue.enqueue(cellMap[fRow][fCol - 1]);
      }
    }
  }
}

void initializeFloodMaze() {
  // When we run the flood maze, none of the cells should be visited already
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      cellMap[i][j].visited = false;
    }
  }
  
  // Set center to 4 cells in the middle
  cellMap[CENTER_ROW][CENTER_COLUMN].floodDistance = 0;
  cellMap[CENTER_ROW][CENTER_COLUMN].visited = true;
  floodQueue.enqueue(cellMap[CENTER_ROW][CENTER_COLUMN]);

  cellMap[CENTER_ROW + 1][CENTER_COLUMN].floodDistance = 0;
  cellMap[CENTER_ROW + 1][CENTER_COLUMN].visited = true;
  floodQueue.enqueue(cellMap[CENTER_ROW][CENTER_COLUMN]);

  cellMap[CENTER_ROW][CENTER_COLUMN + 1].floodDistance = 0;
  cellMap[CENTER_ROW][CENTER_COLUMN + 1].visited = true;
  floodQueue.enqueue(cellMap[CENTER_ROW][CENTER_COLUMN]);

  cellMap[CENTER_ROW + 1][CENTER_COLUMN + 1].floodDistance = 0;
  cellMap[CENTER_ROW + 1][CENTER_COLUMN + 1].visited = true;
  floodQueue.enqueue(cellMap[CENTER_ROW][CENTER_COLUMN]);
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

void setup() {
  // put your setup code here, to run once:
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 100000);
  Serial.begin(9600);
  delay(1000);
  Serial.println("Initializing:");

  // Sets all sensors to low for initialization
  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
    pinMode(sensor_pins[i], OUTPUT);
    digitalWrite(sensor_pins[i], LOW);
  }

  // Initializes sensors
  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
    initSensor(sensor_pins[i], sensors[i], i + 1);
  }

  initializeCellMap();

  // SET UP BOUNDARY WALLS!!!!! :)
  setBoundaryWalls();
}

void loop() {
  // put your main code here, to run repeatedly:  

  for (unsigned int i = 0; i < sensors.size(); i += 2) {
    if (sensors[i]->readRangeContinuousMillimeters() < threshold) {
      
      Serial.print(sensor_names[i]);
      Serial.print(" ");
    }
    if (sensors[i]->timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  }
  Serial.println();

  // index 0 = left
  // index 2 = front
  // index 4 = right
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

// REWRITE FOR 2 DIMENSIONAL
void setBoundaryWalls() {
  // NOTE: Top of the maze is set to the the 0th row
  
  // North
  for (int i = 0; i < 16; i++) {
    virtualWalls[i] |= NORTH;
  }

  // East
  for (int i = 15; i < 256; i += 16) {
    virtualWalls[i] |= EAST;
  }

  // South
  for (int i = 240; i < 256; i++) {
    virtualWalls[i] |= SOUTH;
  }

  // West
  for (int i = 0; i < 256; i += 16) {
    virtualWalls[i] |= WEST;
  }
}

void printVirtualMaze() {
  int check_north = 0;
  int check_south = 0;
 
  for (int row = 0; row < 16; row++) {
    printVirtualColumn(row, true);
    printVirtualColumn(row, false);
  }

  // print very bottom row
  //printVirtualColumn()
}

// isPost boolean variables flags whether we are printing a row with vertical walls or horizontal walls
void printVirtualColumn(int row, bool isPost) {
  int check_north_wall = 0;
  int check_south_wall = 0;
  int check_east_wall = 0;
  int check_west_wall = 0;

  for (int col = 0; col < 16; col++) {
    check_north_wall = 0;
    check_south_wall = 0;
    check_east_wall = 0;
    check_west_wall = 0;
    
    if (isPost) {
      Serial.print("+");
      // check north wall in here 
      check_north_wall = virtualWalls[row * 16 + col] & NORTH;

      // check south wall in here
      if (row > 0) {
        check_south_wall = virtualWalls[(row - 1) * 16 + col] & SOUTH;
      }
      
      if (check_north_wall == NORTH || check_south_wall == SOUTH) {
        Serial.print("---");
      } 
      else {
        Serial.print("   ");
      }
    }
    else {
      check_west_wall = virtualWalls[row * 16 + col] & WEST;

      if (col > 0) {
        check_east_wall = virtualWalls[row * 16 + col - 1] & EAST;
      } 

      if (check_west_wall == WEST || check_east_wall == EAST) {
        Serial.print("|");
      } else {
        Serial.print(" ");
      }
      
      Serial.print("   ");
    }
  }

  if (isPost) {
    Serial.print("+");
  } else {
    Serial.print("|");
  }

  Serial.println();
}








