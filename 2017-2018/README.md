# Robot Overview

## Code Structure
#### mouse_master
The master `.ino` file for mapping and speed running the maze.
#### mouse_testing
A series of test sets that can be run to verify the integrity and tune PIDs for different movements and compound movements.
#### tests/
A group of hardware or software specific tests for testing functionality.
#### libraries/
A folder containing all libraries used in the robot code.
#### libraries/device_abstractions/
A set of libraries we wrote for the different high level parts of the robot.
#### libraries/hadware_config/
Config files for the robot that are populated with Teensy pin numbers.

## Operation
### mouse_master
#### Setup
1. When the mouse turns on, the back LED should flash red.
2. Wait until the back LED flashes blue, then connect to the bluetooth module using the `Adafruit Bluetooth LE Connect` app.
3. After connecting, the back LED should flash green indicating it is ready for commands. 

#### Commands
- 'reset' : resets the internal state of the robot to start a new run without accumulated error from the previous run
- 'fullreset' : additionally resets the wall map of the maze to start a fresh mapping run
- 'go' : advances a single cell in the maze
- 'start' : runs the maze until completion and returns to the start
- Remote controls
    + 'w' : advances forward one cell
    + 'a' : turns left 90 degrees
    + 'd' : turns right 90 degrees
- 'celebrate' : flashes the LEDs in a celebratory pattern
- 'setgoal x y' : sets the goal of the maze to be row 'x' and column 'y'
- 'tune' : enters the tuning menu for the PIDs
- 'help' : shows available commands

### mouse_testing
#### Setup
1. The back LED will flash red at the beginning of the setup function.
2. The back LED will flash green when it's done doing setup.

If you want to start with a different test suite than the basic level 0 tests, press the front button once. The back LED will flash green and  increment the counter changing the test suite number.

#### Operation
- Press the back button once to run the next test. 
- Press the front button once to repeat a test that's already run.

