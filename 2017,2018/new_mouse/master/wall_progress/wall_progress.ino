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

int mouseRow = 13;
int mouseColumn = 12;

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
    Serial.print("Row: ");
    Serial.print(frontier.row);
    Serial.print(" | Column: ");
    Serial.print(frontier.column);
    Serial.print(" | FloodDistance: ");
    Serial.println(frontier.floodDistance);

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
  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 100000);
  Serial.begin(9600);
  delay(1000);
  Serial.println("We're setting up the map!");
  Serial.println("I'm a map");
//  Serial.println("Initializing:");
//
//  // Sets all sensors to low for initialization
//  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
//    pinMode(sensor_pins[i], OUTPUT);
//    digitalWrite(sensor_pins[i], LOW);
//  }
//
//  // Initializes sensors
//  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
//    initSensor(sensor_pins[i], sensors[i], i + 1);
//  }

  initializeCellMap();

  // SET UP BOUNDARY WALLS!!!!! :)
  setBoundaryWalls();

  delay(2000);
  floodMaze();

  printVirtualMaze();
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

      if (row == mouseRow && col == mouseColumn) {
        Serial.print(" @ ");
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








