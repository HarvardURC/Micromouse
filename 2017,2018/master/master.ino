#include <QueueArray.h>
#include <EEPROM.h>
#include <VL6180X.h>
#include <config.h>
#include <emile_motors.h>
#include <i2c_t3.h>
#include <vector>
#include <string>


// Bitmap directions
#define NORTH 8
#define EAST 4
#define SOUTH 2
#define WEST 1


// VERY IMPORTANT: UPDATE MAZE CONSTANTS EACH RUN
#define MAZE_WIDTH 3
#define MAZE_HEIGHT 6
#define CENTER_ROW 2
#define CENTER_COL 1

// VERY IMPORTANT: UPDATE MOUSE STARTING CONSTANTS EACH RUN
// 0 NORTH
// 1 EAST
// 2 SOUTH
// 3 WEST 
#define STARTING_DIR 1
#define START_ROW 0
#define START_COL 0
int mouseDir;
int mouseRow;
int mouseCol;

// Two different runs, operated by buttons
bool mapping_run;
bool speed_run;
#define SPEED_START 70
#define SPEED_INCREASE 30
#define SPEED_DECREASE 10
// Variable storing mouse speed
int speed;

// Sensor constant for wall detection
const int wallThreshold = 280;
// {left, leftDiagonal, front, rightDiagonal, right}
std::vector<int> sensorReadings = {1000, 1000, 1000, 1000, 1000};
// initialize sensor vectors
std::vector<int> sensor_pins = {pins::tofLeft, pins::tofLeftDiag, pins::tofFront, pins::tofRightDiag, pins::tofRight};
std::vector<VL6180X*> sensors = {new VL6180X, new VL6180X, new VL6180X, new VL6180X, new VL6180X};
std::vector<String> sensor_names = {"left", "leftDiag", "front", "rightDiag", "right"};
// initialize motors object
emile_motors* motors = new emile_motors(sensors[0], sensors[4], sensors[2], sensors[1], sensors[3]);

// button constants
int S8_timer;
bool S8_active;
bool S8_long_active;
bool S6_active;

// Time to hold down button in ms
#define LONG_PRESS_THRESHOLD 1000

// Store 2d array of Cell structs with wall/floodfill information
struct Cell {
  int row;
  int column;
  int floodDistance;
  bool visited;
  unsigned char walls;
};
Cell cellMap[MAZE_HEIGHT][MAZE_WIDTH];

// Queue of cells for flood fill breadth first search
QueueArray<Cell> floodQueue;

// Arrays with ordered strings of direction stored during mapping runs
short pathToCenter[256];
short pathToStart[256];
short rawPath[256];
short pathLength;

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
void initSensor(int pin, VL6180X *sensor, int address);
void turn(int desired);
void checkButtons();
void speedRun();


/*
*	SETUP
*/
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

  // Set mouse position/direction
  mouseRow = START_ROW;
  mouseCol = START_COL;
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

  // TODO: Retrieving maze data from EEPROM memory

//  int eepromAddr = 0;
//  eepromAddr += EEPROM_readAnything(eepromAddr, pathLength);
//  
//  for (short i = 0; i < 256; i++) {
//    eepromAddr += EEPROM_readAnything(eepromAddr, pathToCenter[i]);
//  }
//  for (short i = 0; i < 256; i++) {
//    eepromAddr += EEPROM_readAnything(eepromAddr, pathToStart[i]);
//  }

  Serial.println("Finished setup");
}


/*
*	LOOP
*/
void loop() {
  // get button input
  checkButtons();
  Serial.println("Made it past checkButtons()");
  
  if (mapping_run) {
  	Serial.println("Mapping Run");

  	// Look for walls and update cellMap wall data
    senseWalls();
    Serial.println("Made it past sense walls");

    // Execute flood fill
    floodMaze();
    Serial.println("Made it past floodmaze");

    // Choose a direction for the mouse to go
    Janus();
    Serial.println("Made it past janus");

    // Display maze virtually
    printVirtualMaze();
    Serial.println("Made it past print virtual maze");

    // Change delay at will
    delay(150);
  } 
  else if (speed_run) {
  	Serial.println("Speed Run");

  	// Execute series of directions stored in pathToCenter and pathToStart arrays
    speedRun();
  }
}


/*
*	JANUS
*/
void Janus() {
  // Mouse chooses cell to move to
  // Functionality for end of mapping run

  if (cellMap[mouseRow][mouseCol].floodDistance == 0) {
    Serial.println("Reached the Center!");

    /*
	*	READ ME
	*	I, James Conant, was a huge dingus and forgot to add the dead end deletion code until 20 minutes for the competition
	*	It should work in principle, but I would test it thoroughly and make sure it works in every scenario (including cycles?)
	*	Have a wonderful day!
    */

    // We use the 'visited' attribute of each Cell struct to check for dead ends. 
    // Set every cells' visited value to false
    initializeFloodMaze();

    // CONVERT RawPath TO pathToCenter
    int tempRow = START_ROW;
    int tempCol = START_COL;
    cellMap[tempRow][tempCol].visited = true;
    
    for (int i = 0; i < pathLength; i++) {
  	  // Change temporary position virtually  
	  if (rawPath[i] == 0) {
	    tempRow--;
	  }
	  else if (rawPath[i] == 1) {
	    tempCol++;
	  }
	  else if (rawPath[i] == 2) {
	    tempRow++;
	  }
	  else if (rawPath[i] == 3) {
	    tempCol--;
	  }

      // Dead ends occur when we have reached a cell that has already been visited
      // 
      if (cellMap[tempRow][tempCol].visited) {
      	// Use temporary variables to not bungle outside loop
        int leftPair = i - 1;
        int rightPair = i;

        // Delete dead ends by recurrently deleting pairs of opposing directions
        while (rawPath[leftPair] == (rawPath[rightPair] + 2) % 4) {
          for (int j = rightPair + 1; j < pathLength; j++) {
            rawPath[j - 2] = rawPath[j];
          }
          pathLength -= 2;
          i--;

          leftPair--;
          rightPair--;

          if (rawPath[rightPair] == 0) {
            tempRow--;
          }
          else if (rawPath[rightPair] == 1) {
            tempCol++;
          }
          else if (rawPath[rightPair] == 2) {
            tempRow++;
          }
          else if (rawPath[rightPair] == 3) {
            tempCol--;
          }

          // Signal deletion of dead end directions
          digitalWrite(pins::led, HIGH);
          delay(500);
          digitalWrite(pins::led, LOW);
        }
      }

      cellMap[tempRow][tempCol].visited = true;
    }

    // Store rawPath in pathToCenter
    for (int i = 0; i < pathLength; i++) {
      pathToCenter[i] = rawPath[i];
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

    // Go back to start
    for (int i = 0; i < pathLength; i++) {
      turn(pathToStart[i]);
      motors->forward();
    }

    // stop mapping run
    mapping_run = false;

    // Activate speed_run with the buttons
  } 
  else {
  	// Choose cell with lowest floodDistance to move to 
    int thePath = chooseDirection(mouseRow, mouseCol);
    Serial.print("thePath: ");
    Serial.println(thePath);

    // Mapping_run movement
    turn(thePath);
    motors->forward();
    
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

    // store values in rawPath
    rawPath[pathLength] = thePath;
    pathLength++;
  }
}

/*
*	IMPORTANT TODO: NOT YET IMPLEMENTED
*	If the mouse reaches an intersection, don't choose the same direction every run
*	The mouse can get stuck always trying a tougher path
*	Add randomization or iteration for each equivalent direction choice at an intersection
*/
// Returns the direction to cell with lowest floodDistance out of 4 bordering cells
int chooseDirection(int currentRow, int currentCol) {
  int choices[4] = {1000, 1000, 1000, 1000};

  // 0 NORTH
  // 1 EAST
  // 2 SOUTH
  // 3 WEST

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
  // bestPath stores the direction of the cell with lowest floodDistance
  int minDistance = 1000;
  int bestPath = 0;
  // Add random/iterative choosing of direction here
  for (int i = 0; i < 4; i++) {
    if (choices[i] <= minDistance) {
      minDistance = choices[i];
      bestPath = i;
    }
  }

  return bestPath;
}


// Turn the mouse to face the desired direction
void turn(int desired) {
  // Get difference in angle - best understood with a diagram
  int difference = (desired - mouseDir + 4) % 4;

  // TURN RIGHT
  if (difference == 1) {
    motors->turnRight();
    Serial.println("turning right #######################");
  }
  // TURN LEFT
  else if (difference == 3) {
    motors->turnLeft();
    Serial.println("turning left @@@@@@@@@@@@@@@@@@@@@");
  }
  // TURN AROUND
  else if (difference == 2) {
    motors->turnAroundRight();
    Serial.println("turning around!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  // Virtual mouse direction
  mouseDir = desired;
}


/*
*	FLOOD FILL
*/
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


// Returns TRUE if wall exists in specified direction, FALSE if not
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


// Prepare virtual maze for floodFill
// Only changed 'visited' and 'floodDistance'
void initializeFloodMaze() {
  // When we run the flood maze, none of the cells should be visited already
  for (int i = 0; i < MAZE_HEIGHT; i++) {
    for (int j = 0; j < MAZE_WIDTH; j++) {
      cellMap[i][j].visited = false;
      cellMap[i][j].floodDistance = 1000;
    }
  }

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

//  cellMap[CENTER_ROW + 1][CENTER_COL].floodDistance = 0;
//  cellMap[CENTER_ROW + 1][CENTER_COL].visited = true;
//  floodQueue.enqueue(cellMap[CENTER_ROW + 1][CENTER_COL]);
//
//  cellMap[CENTER_ROW][CENTER_COL + 1].floodDistance = 0;
//  cellMap[CENTER_ROW][CENTER_COL + 1].visited = true;
//  floodQueue.enqueue(cellMap[CENTER_ROW][CENTER_COL + 1]);
//
//  cellMap[CENTER_ROW + 1][CENTER_COL + 1].floodDistance = 0;
//  cellMap[CENTER_ROW + 1][CENTER_COL + 1].visited = true;
//  floodQueue.enqueue(cellMap[CENTER_ROW + 1][CENTER_COL + 1]);
}



// Sense walls and store wall readings in cellMap
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


/*
*	Check Buttons
*/
void checkButtons() {
  // S8 changes the speed, turns on speed_run
  // S6 overrides Teensy memory EEBROM, turns on mapping_run
  while (digitalRead(pins::buttonS6) == LOW) {
    S6_active = true;
  }

  if (S6_active) {
    mapping_run = true;
    pathLength = 0;
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


/*
*	SPEED RUN
*/
void speedRun() {
  // Run through pathToCenter, turn and move robot forward according to direction at each index
  for (int i = 0; i < pathLength; i++) {
    turn(pathToCenter[i]);
    motors->forward();
  }
  Serial.println("REACHED THE CENTER!");
  // Run through pathToStart
  for (int i = 0; i < pathLength; i++) {
    turn(pathToStart[i]);
    motors->forward();
  }
  Serial.println("REACHED THE START");
}


/*
*	TODO: EEPROM Shenaniganry
*/

//template <class T> int EEPROM_writeAnything(int ee, const T& value)
//{
//   const byte* p = (const byte*)(const void*)&value;
//   int i;
//   for (i = 0; i < sizeof(value); i++)
//       EEPROM.write(ee++, *p++);
//   return i;
//}
//
//template <class T> int EEPROM_readAnything(int ee, T& value)
//{
//   byte* p = (byte*)(void*)&value;
//   int i;
//   for (i = 0; i < sizeof(value); i++)
//       *p++ = EEPROM.read(ee++);
//   return i;
//}

// Sensor intialization, should only called from setup()
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


// cellMap initialization, should only be called upon setup()
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


// Write boundary walls of the maze to cellMap
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


// Print out cellMap
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