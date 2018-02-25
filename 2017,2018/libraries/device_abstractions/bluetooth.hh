#include <Adafruit_BluefruitLE_SPI.h>
#include <SPI.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>

extern Adafruit_BluefruitLE_SPI ble;

void bluetoothInitialize();

// methods include void print(char* blah), char* read(), and int available()