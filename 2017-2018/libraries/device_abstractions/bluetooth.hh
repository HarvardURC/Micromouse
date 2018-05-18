#include <Adafruit_BluefruitLE_SPI.h>
#include <SPI.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>

#ifndef bluetooth_hh
#define bluetooth_hh

extern Adafruit_BluefruitLE_SPI ble;

void bluetoothInitialize();
bool bleReady();

#endif
// methods include void print(char* blah), char* read(), and int available()
