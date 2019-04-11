 /*
 This example code uses bogde's excellent library: https://github.com/bogde/HX711
 bogde's library is released under a GNU GENERAL PUBLIC LICENSE
 
 The HX711 does one thing well: read load cells. The breakout board is compatible with any wheat-stone bridge
 based load cell which should allow a user to measure everything from a few grams to tens of tons.

 Arduino pin 2 -> HX711 CLK
 3 -> DAT
 5V -> VCC
 GND -> GND
 
 The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power should be fine.
 
*/


//*****HX711 load cell*****// 
#include "HX711.h"
#define calibration_factor -7050.0 //This value is obtained using the SparkFun_HX711_Calibration sketch
#define DOUT  11
#define CLK  10
HX711 scale(DOUT, CLK);
float loadVal = 0.0;


//******opto photodetector for RPM counter******//
int IR_DETECTOR_PIN  = A0;
int sensorValue = 0;
int HIGH_VAL = 1000; // anything above this value is considered high value. 
int previousVal = sensorValue;
bool isCW = true; //signifies if the wheel is rotating CW 

//hardware
int numHoles = 20; //20 holes for the encoded wheels
//wheel diameter = 44.45 mm
const float distOfSignChanged = (44.45*3.14159/20)/2; // (2*pi*r/radius)/20 in mm 
float cLen = 0.0; // in mm


//************************start*****************************//

void setup() {
  Serial.begin(9600);

  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0

  sensorValue = analogRead(IR_DETECTOR_PIN);
  previousVal = sensorValue;
}

void loop() {
  
  // obtain latest scale reading
  loadVal = getScaleReading();

  
  //read the opto switch value
  sensorValue = analogRead(IR_DETECTOR_PIN);
  if (sensorValue >= HIGH_VAL && previousVal < HIGH_VAL)
  {
    previousVal = sensorValue;
    cLen = getCableLength(cLen,true);
  }
  else if(sensorValue < HIGH_VAL && previousVal >= HIGH_VAL)
  {
    previousVal = sensorValue;
    cLen = getCableLength(cLen,true);
  }
}




//****compute the cable length based on diameter of the wheel****//
float getCableLength(float cLen, bool isCW)
{
  if(isCW)
      return cLen + distOfSignChanged;
    else
      return cLen - distOfSignChanged; 
}


//****read the value from the load cell****// 
float getScaleReading()
{
  return scale.get_units();
}

