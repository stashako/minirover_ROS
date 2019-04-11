#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *lMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *rMotor = AFMS.getMotor(2);
// Motor for the cable reel- cable management system
Adafruit_DCMotor *cMotor = AFMS.getMotor(3);


int UD = 0;
int LR = 0;
int LRMID = 0;
int UDMID = 0;
float throttle = 0;
float direction = 0;
const byte PIN_BUTTON_UP = A3;
const byte PIN_BUTTON_DOWN = A4;

  /* Arduino Micro Input Pins */
int IUD=A2;
int ILR=A1;

/*
 * UD minmax [45-1023]
 * LR minmax [6-1023]
 */

 float ud_min = 45;
 float ud_max = 1023;
 float lr_min = 6;
 float lr_max = 1023;
 float max_percent = 0;
 float maxMotorScale = 0;
 float leftMotor = 0;
 float rightMotor = 0;
 float leftMotorScaled = 0;
 float rightMotorScaled = 0;
 float deadZone = 0.1; //10 percent of the full throttle

void setup() {

AFMS.begin();

//calabrate center
  LRMID = analogRead(ILR);
  UDMID = analogRead(IUD);

  // pull the pins high
  pinMode(PIN_BUTTON_UP, INPUT);  
  digitalWrite(PIN_BUTTON_UP, HIGH);
 
  pinMode(PIN_BUTTON_DOWN, INPUT);  
  digitalWrite(PIN_BUTTON_DOWN, HIGH);
}

void loop() {
  UD = analogRead(IUD);
  LR = analogRead(ILR);

  /* up or down */
  if(UD < UDMID)
  {
    throttle = (UDMID - UD)/(UDMID-ud_min)*-100.0;
  }
  else
  {
    throttle = (UD - UDMID)/(ud_max - UDMID)*100.0;
  }

  /* left or right */
  if(LR < LRMID)
  {
    direction = (LRMID - LR)/(LRMID-lr_min)*-100.0;
  }
  else
  {
    direction = (LR - LRMID)/(lr_max - LRMID)*100.0;
  }

  //mix throttle and direction
  leftMotor = throttle-direction;
  rightMotor = throttle+direction;
  
  //choose the max scale value if it is above 1
  maxMotorScale = max(leftMotor,rightMotor);
  maxMotorScale = max(100,maxMotorScale);

  //and apply it to the mixed values
  leftMotorScaled = constrain(leftMotor/maxMotorScale,-1,1);
  rightMotorScaled = constrain(rightMotor/maxMotorScale,-1,1);

  Serial.print("left = ");
  Serial.println(leftMotorScaled*100);

  if(abs(leftMotorScaled) > deadZone)
  {
    if(leftMotorScaled < 0)
    {
      lMotor->setSpeed(leftMotorScaled*-255);
      lMotor->run(BACKWARD);
    }
    else
    {
      lMotor->setSpeed(leftMotorScaled*255);
      lMotor->run(FORWARD);
    }
  }
  else
  {
    lMotor->run(RELEASE);
  }
    

  Serial.print("right = ");
  Serial.println(rightMotorScaled*100);
  if(abs(rightMotorScaled) > deadZone)
  {
    if(rightMotorScaled < 0)
    {
      rMotor->setSpeed(rightMotorScaled*-255);
      rMotor->run(BACKWARD);
    }
    else
    {
      rMotor->setSpeed(rightMotorScaled*255);
      rMotor->run(FORWARD);
    }
  }
  else
  {
    rMotor->run(RELEASE);
  }

  //handle the cable reel button
  if(digitalRead(PIN_BUTTON_UP) && !digitalRead(PIN_BUTTON_DOWN))
  {
      cMotor->setSpeed(255);
      cMotor->run(FORWARD);
  }
  else if(digitalRead(PIN_BUTTON_DOWN) && !digitalRead(PIN_BUTTON_UP))
  {
      cMotor->setSpeed(255);
      cMotor->run(BACKWARD);
  }
  else
  {
      cMotor->run(RELEASE);
  }
  
  delay(200);

}
