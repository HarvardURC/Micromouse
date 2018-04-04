#include "config.h"
#include "bluetooth.hh"

#define BUFSIZE                        	160   	// Size of the read buffer for incoming data
#define VERBOSE_MODE                   	true  	// If set to 'true' enables debug output
#define FACTORYRESET_ENABLE         	1		// Reset to forget all previous settings
#define MINIMUM_FIRMWARE_VERSION    	"0.6.6"
#define MODE_LED_BEHAVIOUR          	"MODE"

// error helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// TODO not sure if this is the best way to declare this
Adafruit_BluefruitLE_SPI ble(pins::CS, pins::bluetoothIRQ, pins::bluetoothRST);
// Adafruit_BluefruitLE_SPI ble(10, 9, 8); // Teensy 3.2 test mappings
bool bluetoothInitalized = false;

void bluetoothInitialize() {
    bluetoothInitalized = true;

	// Define pins
	SPI.setSCK(pins::SCK);	// using alternate SCK pin
    SPI.begin();

    Serial.begin(115200);
    Serial.println(F("Adafruit Bluefruit Command <-> Data Mode"));
    Serial.println(F("------------------------------------------------"));

    /* Initialise the module */
    Serial.print(F("Initialising the Bluefruit LE module: "));

    if ( !ble.begin(VERBOSE_MODE) )
    {
    	error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }
    Serial.println( F("OK!") );

    if ( FACTORYRESET_ENABLE )
    {
	    /* Perform a factory reset to make sure everything is in a known state */
	    Serial.println(F("Performing a factory reset: "));
	    if ( ! ble.factoryReset() ){
	    	error(F("Couldn't factory reset"));
	    }
    }

    /* Disable command echo from Bluefruit */
    ble.echo(false);

    Serial.println("Requesting Bluefruit info:");
    /* Print Bluefruit information */
    ble.info();

    Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
    Serial.println(F("Then Enter characters to send to Bluefruit"));
    Serial.println();

    ble.verbose(false);  // debug info is a little annoying after this point!

    /* Wait for connection */
    while (! ble.isConnected()) {
    	delay(500);
    }

    Serial.println(F("******************************"));

    // LED Activity command is only supported from 0.6.6
    if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
    {
	    // Change Mode LED Activity
	    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
	    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    }

    // Set module to DATA mode
    Serial.println( F("Switching to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);

    Serial.println(F("******************************"));
}

bool bleReady() {
    return bluetoothInitalized && ble.isConnected();
}
