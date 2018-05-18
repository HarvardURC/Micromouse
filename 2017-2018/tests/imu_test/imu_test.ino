/**
  An Inertial Measurement Unit (IMU) sensor test.
  Outputs to Serial console the readings from the imu sensor.
*/

#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <config.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);//pins::imuRST);
 
uint8_t sys;
uint8_t gyro;
uint8_t accel;
uint8_t mag;


void setup(void) 
{
  Serial.begin(9600);
  delay(5000);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  Serial.print("Calibration: ");
  Serial.print("sys: ");
  Serial.print(sys);
  Serial.print("gyro: ");
  Serial.print(gyro);
  Serial.print("accel: ");
  Serial.print(accel);
  Serial.print("mag: ");
  Serial.println(mag);
  
  delay(100);
}
