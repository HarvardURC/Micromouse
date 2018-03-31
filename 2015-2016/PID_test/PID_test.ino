#include <PID_v1.h>
#include <Encoder.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output, direction_pin, power_pin;
Encoder knobLeft(16, 17);

//Specify the links and initial tuning parameters
//1,.0017,.01
PID myPID(&Input, &Output, &Setpoint,1,0.0018,.01, DIRECT);
unsigned long time;

void setup()
{
  Serial.begin(9600);
  delay(5000);
  //set up motors
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  //initialize the variables we're linked to
  Input = 0;
  knobLeft.write(0);
  Setpoint = -2000;
  myPID.SetOutputLimits(-100, 100);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  time = millis();
  Input = knobLeft.read();
  myPID.Compute();
  Serial.print(Input);
  Serial.println();
  //Serial.print(power_pin);
  Serial.println();
  if (Output >= 0){
    direction_pin = LOW;
  }
  else {
    direction_pin = HIGH;
  }
  
  power_pin = abs(Output); 
  digitalWrite(11,direction_pin);
  //analogWrite(10,255);
  analogWrite(10,power_pin);
  myPID.SetTunings(1.5,0.01,0.01);
  //myPID.SetTunings(1.5,0.01,0.01);
  /*
  if (abs(Input - Setpoint) < 15){
    myPID.SetTunings(3,.0005,.007);
  }
  else if (abs(Input - Setpoint) < 30){
    myPID.SetTunings(1,.0017,.01);
  }
  else {
    myPID.SetTunings(.5,.0017,.015);
  }
  */
  while (millis() - time < 1){}
}

int map_output(double output)
{
  if (output < 30 && output > 10){
    return 30;
  }
  else return output;
}

