#include <vector>
#include <VL6180X.h>
#include <i2c_t3.h>
#include <config.h>

#define NORTH 8
#define EAST 4
#define SOUTH 2
#define WEST 1

std::vector<int> sensor_pins = {pins::tofLeft, pins::tofLeftDiag, pins::tofFront, pins::tofRightDiag, pins::tofRight};
std::vector<VL6180X*> sensors = {new VL6180X, new VL6180X, new VL6180X, new VL6180X, new VL6180X};
std::vector<String> sensor_names = {"left", "leftDiag", "front", "rightDiag", "right"};

const int threshold = 250;

unsigned char virtualWalls[256];

void initSensor(int pin, VL6180X *sensor, int address);
void setBoundaryWalls();
void printVirtualMaze();

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
      Serial.print("+")
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






