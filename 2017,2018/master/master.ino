#include <QueueArray.h>
#include <EEPROM.h>
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

#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16

#define CENTER_ROW 1
#define CENTER_COL 1

#define START_ROW 0
#define START_COL 0

#define SPEED_START 70
#define SPEED_INCREASE 30
#define SPEED_DECREASE 10

// VERY IMPORTANT: UPDATE STARTING_DIR each run

#define STARTING_DIR 1

#define LONG_PRESS_THRESHOLD 1000

bool mapping_run;
bool speed_run;

int mouseRow;
int mouseCol;

// 0 NORTH
// 1 EAST
// 2 SOUTH
// 3 WEST 
int mouseDir;

const int wallThreshold = 280;

int S8_timer;
bool S8_active;
bool S8_long_active;
bool S6_active;

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
int chooseDirection(int currentRow, int currentCol);
void initializeFloodMaze();
// void generateWall();
void initSensor(int pin, VL6180X *sensor, int address);
void turn(int desired);
// void store(int nextMovement);
// void addSpdCount();
void checkButtons();
void speedRun();

// initialize sensors
std::vector<int> sensor_pins = {pins::tofLeft, pins::tofLeftDiag, pins::tofFront, pins::tofRightDiag, pins::tofRight};
std::vector<VL6180X*> sensors = {new VL6180X, new VL6180X, new VL6180X, new VL6180X, new VL6180X};
std::vector<String> sensor_names = {"left", "leftDiag", "front", "rightDiag", "right"};

// initialize speed counter and speed run array
int speed;

//String pathToCenterCheck [150];
short pathToCenter[256];
short pathToStart[256];
short pathLength;

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
  pinMode(pins::buttonS8, INPUT_PULLUP);
  pinMode(pins::buttonS6, INPUT_PULLUP);

  // Initializes LED
  pinMode(pins::led, OUTPUT);

  // Initialize Cell values in the CellMap
  initializeCellMap();
  Serial.println("Made it past initializing cell map!");

  // SET UP BOUNDARY WALLS!!!!! :)
  setBoundaryWalls();
  Serial.println("Made it past set boundary walls!");

  // Set mouse position
  mouseRow = START_ROW;
  mouseCol = START_COL;

  // make sure to change STARTING_DIR according to maze
  mouseDir = STARTING_DIR;

  // initialize speed
  speed = SPEED_START;
  motors->MOTOR_SPEED = speed;

  // starting path is 0
  pathLength = 0;

  // Only start runs when buttons are pressed
  mapping_run = false;
  speed_run = false;

  // no buttons pressed
  S8_active = false;
  S8_long_active = false;
  S6_active = false;

  int eepromAddr = 0;
  eepromAddr += EEPROM_readAnything(eepromAddr, pathLength);
  
  for (short i = 0; i < 256; i++) {
    eepromAddr += EEPROM_readAnything(eepromAddr, pathToCenter[i]);
  }
  for (short i = 0; i < 256; i++) {
    eepromAddr += EEPROM_readAnything(eepromAddr, pathToStart[i]);
  }


}

// LOOP
void loop() {
  checkButtons();

  if (mapping_run) {
    senseWalls();
    Serial.println("Made it past sense walls");

    floodMaze();
    Serial.println("Made it past floodmaze");

    Janus();
    Serial.println("Made it past janus");

    printVirtualMaze();
    Serial.println("Made it past print virtual maze");

    delay(300);
  } 
  else if (speed_run) {
    speedRun();
  }
}

// Mouse chooses cell to move to
// Functionality for end of mapping run
void Janus() {
  if (cellMap[mouseRow][mouseCol].floodDistance == 0) {
    Serial.println("we're here");
    // for (int i = 0; i < pathLength; i++) {
    //   quickestPath[i] = pathToCenter[i];
    // }
    //    int beginningArray, endArray;
    //    int endingReference = moveIndex - 1;
    //    for (int i = 0; i < moveIndex/2; i++, endingReference--) {
    //      beginningArray = quickestPath[i];
    //      endArray = quickestPath[endingReference];
    //      quickestPath[endingReference] = beginningArray;
    //      quickestPath[i] = endArray;
    //      }

    // FLOOD THE MAZE
    // STORE SPEED PATH

    floodMaze();

    // Virtual cell placeholders
    int tempRow = START_ROW;
    int tempCol = START_COL;

    int tempDir = 0;
    pathLength = 0; 

    while (cellMap[tempRow][tempCol].floodDistance != 0) {
      tempDir = chooseDirection(tempRow, tempCol);

      pathToCenter[pathLength] = tempDir;

      if (tempDir == 0) {
        tempRow--;
      }
      else if (tempDir == 1) {
        tempCol++;
      }
      else if (tempDir == 2) {
        tempRow++;
      }
      else if (tempDir == 3) {
        tempCol--;
      }

      pathLength++;
    }

    // reverse pathToCenter for pathToStart
    int start = 0;
    int end = pathLength - 1;

    while (start <= end) {
      pathToStart[start] = (pathToCenter[end] + 2) % 4;
      pathToStart[end] = (pathToCenter[start] + 2) % 4; 
      start++;
      end--;
    }
    // Store map data to EEPROM:
    // FORMAT: Pathlength, pathToCenter, pathToStart
    int eepromAddr = 0;
    eepromAddr += EEPROM_writeAnything(eepromAddr, pathLength);

    for (int i = 0; i < pathLength; i++){
      eepromAddr += EEPROM_writeAnything(eepromAddr, pathToCenter[i]);
    }
    for (int i = 0; i < pathLength; i++){
      eepromAddr += EEPROM_writeAnything(eepromAddr, pathToStart[i]);
    }

    // Go back to start
    for (int i = 0; i < pathLength; i++) {
      turn(pathToStart[i]);
      motors->forward();
    }

    // Increment spd to go on to speed runs
//    spd++;

    // stop mapping run
    mapping_run = false;
  } //jdjdjdjdjdjdjdjdjdj
  else {
    int thePath = chooseDirection(mouseRow, mouseCol);

    Serial.print("thePath: ");
    Serial.println(thePath);

    // Move during first run
    turn(thePath);
    motors->forward();
    // store(thePath);

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
}

int chooseDirection(int currentRow, int currentCol) {
  int choices[4] = {1000, 1000, 1000, 1000};

  // 0 NORTH
  // 1 EAST
  // 2 SOUTH
  // 3 WEST
  // We want thePath == mouseDir

  // NORTH
  if (!checkWall(currentRow, currentCol, NORTH)) {
    choices[0] = cellMap[currentRow - 1][currentCol].floodDistance;
  }
  // EAST
  if (!checkWall(currentRow, currentCol, EAST)) {
    choices[1] = cellMap[currentRow][currentCol + 1].floodDistance;
  }
  // SOUTH
  if (!checkWall(currentRow, currentCol, SOUTH)) {
    choices[2] = cellMap[currentRow + 1][currentCol].floodDistance;
  }
  // WEST
  if (!checkWall(currentRow, currentCol, WEST)) {
    choices[3] = cellMap[currentRow][currentCol - 1].floodDistance;
  }
  // bestPath stores the cell with lowest
  int minDistance = 1000;
  int bestPath = 0;
  for (int i = 0; i < 4; i++) {
    if (choices[i] < minDistance) {
      minDistance = choices[i];
      bestPath = i;
    }
  }

  return bestPath;
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

  // Adham, in case you think this is a bad variable name.
  // Minotaur was at the center of the Labyrinth, so by finding the Minotaur we are finding the center of the maze
  // if (findMinotaur) {
    // Set center to 4 cells in the middle, with CENTER_ROW, CENTER_COL as the top left

  // x = CENTER_ROW, CENTER_COL
  // +---+---+
  // | x |   |
  // +---+---+
  // |   |   |
  // +---+---+
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
      if (j == MAZE_WIDTH - 1) {
        cellMap[i][j].walls |= EAST;
      }
      if (i == MAZE_HEIGHT - 1) {
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

// void addSpdCount() {
//   // S8 is the increment button
//   int addSpd = digitalRead(pins::buttonS8);
//   int deleteSpd = -digitalRead(pins::buttonS6);
//   spd += addSpd + deleteSpd;
// }

void checkButtons() {
  // S8 changes the speed, turns on speed_run
  // S6 overrides Teensy memory EEBROM, turns on mapping_run
  while (digitalRead(pins::buttonS6) == LOW) {
    S6_active = true;
  }

  if (S6_active) {
    mapping_run = true;
    S6_active = false;
    digitalWrite(pins::led, HIGH);
    delay(500);
    digitalWrite(pins::led, LOW);
    return;
  }

  S8_timer = millis();
  while (digitalRead(pins::buttonS8) == LOW) {
    S8_active = true;
  }
  if (millis() - S8_timer > LONG_PRESS_THRESHOLD && S8_active) {
    S8_long_active = true;
  }

  if (S8_active) {

    if (S8_long_active) {
      motors->MOTOR_SPEED -= SPEED_DECREASE;
        digitalWrite(pins::led, HIGH);
        delay(500);
        digitalWrite(pins::led, LOW);
        delay(500);
    } else {
      motors->MOTOR_SPEED += SPEED_INCREASE;
    }
    
    S8_active = false;
    S8_long_active = false;

    speed_run = true;
    digitalWrite(pins::led, HIGH);
    delay(500);
    digitalWrite(pins::led, LOW);
  }
}

void speedRun() {
  // Run through pathToCenter, turn and move robot forward according to direction at each index
  // motors->MOTOR_SPEED = 150;
  for (int i = 0; i < pathLength; i++) {
    turn(pathToCenter[i]);
    motors->forward();
  }
  Serial.println("REACHED THE CENTER!");
  for (int i = 0; i < pathLength; i++) {
    turn(pathToStart[i]);
    motors->forward();
  }
  Serial.println("REACHED THE START");

  // turn off speed run
  speed_run = false;
  // Reverse each of the directions once the robot has reached the center
  // for (int i = 0; i < pathLength; i++) {
  //   pathToCenter[i] = (pathToCenter[i] + 2) % 4;
  // }
  // // Reverse the array
  // int beginningArray, endArray;
  // int endingReference = pathLength - 1;
  // for (int i = 0; i < pathLength / 2; i++, endingReference--) {
  //   beginningArray = pathToCenter[i];
  //   endArray = pathToCenter[endingReference];
  //   pathToCenter[endingReference] = beginningArray;
  //   pathToCenter[i] = endArray;
  // }
  // // Return to the start
  // for (int i = 0; i < pathLength; i++) {
  //   turn(pathToCenter[i]);
  //   Serial.print(pathToCenter[i]);
  //   motors->forward();
  // }
}


template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
   const byte* p = (const byte*)(const void*)&value;
   int i;
   for (i = 0; i < sizeof(value); i++)
       EEPROM.write(ee++, *p++);
   return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
   byte* p = (byte*)(void*)&value;
   int i;
   for (i = 0; i < sizeof(value); i++)
       *p++ = EEPROM.read(ee++);
   return i;
}
