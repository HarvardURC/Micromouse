/*
 * Provides BTLE communications between Teensy and connected computer
 * Uses DATA mode, which allows data passthrough via BTLE module
 * Alternatively, COMMAND mode allows sending control commands to BTLE module
 */

#include <bluetooth.hh>


/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
// Adafruit_BluefruitLE_SPI ble(10, 9, 8); // Teensy 3.2 test mappings


void setup() {
    bluetoothInitialize();
}


void loop() {
    // Check for user input
    char* output = "test\n";

    // Send data to host via Bluefruit
    ble.print(output);


  // Echo received data
  while ( ble.available() )
  {
    int c = ble.read();

    Serial.print((char)c);

    // Hex output too, helps w/debugging!
    Serial.print(" [0x");
    if (c <= 0xF) Serial.print(F("0"));
    Serial.print(c, HEX);
    Serial.print("] ");
  }
}