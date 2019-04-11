//**************************************************************************************
//**************************************************************************************
// *** cable length and load reading format (from adafruit to pc)
// "C|xxx.x,xxx.x" => cable length in meter, gauge load. 

// *** Command from Joy stick (from PC to adafruit)
// "O|Snum1|Snum2|Snum3" 
// SnumX => S-> pos/neg, numX-> two digits (033 equals to 0.33, with max=1.00)
// Snum1 = forward/backward of joy (forward -> pos)
// Snum2 = left/right of joy (right -> pos)
// Snum3 = in/out of cable (out -> pos)
//**************************************************************************************
//**************************************************************************************

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include "Q2HX711.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"


unsigned long curmillis = 0;


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Motor for the cable reel- cable management system
Adafruit_DCMotor *cMotor = AFMS.getMotor(3);
// Motor for the cable management system active control
Adafruit_DCMotor *cMotor_s = AFMS.getMotor(4);


//space holder for incoming command from the PC
#define INPUT_SIZE 16
char input[INPUT_SIZE];
char *token;

float throttle = 0;
float cableCmd = 0;
float m3m4_ratio_in = 0.5; //m4 = m3m4_ratio * m3
float m3m4_ratio_out = 0.8; //m4 = m3m4_ratio * m3



//**************************** Cable Management ****************************************//
//**************************************************************************************//
unsigned long previousMillis = 0; 
unsigned long interval = 1000;

//*************Opto Encoder****************//
int IR_DETECTOR_PIN  = 10;
int previousVal = 0;
int sensorValue = 0;
int numHoles = 20; //20 holes for the encoded wheels
int wheelDiameter = 44.45; //mm
//const float distOfSignChanged = (44.45*3.14159/20)/2; // (2*pi*r/radius)/20 in mm 
const float pulsesPermm = 73.199/20.00;
float cLen; // in mm
float disp;

//*****HX711 load cell*****// 
#define DOUT  6 //12
#define CLK  5 //11
Q2HX711 hx711(DOUT, CLK);
float initVal = 0.0;
float loadVal = 0.0;


//**********************************setup********************//
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("loading ...");

  //read the first value of the opto encoder
  previousVal  = digitalRead(IR_DETECTOR_PIN);

  AFMS.begin();
}

//*****************************loop**************************//
void loop() {
  //update the current time in milli seconds, Do not remove it !!
  curmillis = millis();

  //perform motor control
  motorControl();

  //read and send cable and load update
  getCableLength();
  getLoadCell();
  sendLoadLength();
  
}

void sendLoadLength()
{
  if(curmillis - previousMillis > interval) {
    // save the last time
    previousMillis = curmillis;   

    char str[10];
    sprintf(str,"C|%4.1f,%4.1f",cLen,loadVal);
    Serial1.print(str); // to control computer
    //Serial.println(str); // to terminal
  }
}

void getLoadCell()
{
  if (!hx711.readyToSend()) return;
  if (initVal == 0.0)
    initVal = hx711.read();
  loadVal = (hx711.read()-initVal)/100.0;
  //Serial.println(loadVal);
}

void getCableLength()
{
  sensorValue = digitalRead(IR_DETECTOR_PIN);
 // if  (sensorValue == 0 && sensorValue != previousVal)  //Beam is Seen
  if  ( sensorValue != previousVal)
  {
    previousVal = sensorValue;
    if(cableCmd == 1 || throttle < 0)
    {
      cLen = cLen - pulsesPermm;
    }
    else if(cableCmd == -1 || throttle > 0)
    {
      cLen = cLen + pulsesPermm; 
    }
    
    //Serial.print(sensorValue);
    //Serial.print("length = ");
    //Serial.println(cLen);
  } 
}

void motorControl()
{
  throttle = 0;
  
  if (Serial1.available())
  {
    memset(input, 0, sizeof(input));
    Serial1.readBytes(input, INPUT_SIZE);
    //Serial.println(input);
    processCommand(input);

    //handle the cable reel button
    if(cableCmd == 1) /////// retrieve cable
    {     
      // main motor
       cMotor->setSpeed(255);
       cMotor->run(FORWARD);
       //active control motor
       cMotor_s->setSpeed((int)(m3m4_ratio_in * 255));
       cMotor_s->run(FORWARD);
    }
    else if(cableCmd == -1) ////// release cable
    {
      //main motor
       cMotor->setSpeed(255);
       cMotor->run(BACKWARD);
       //active control motor
       cMotor_s->setSpeed((int)(m3m4_ratio_out * 255));
       cMotor_s->run(BACKWARD);
    }
    else if(throttle < 0 && loadVal > -500)// detected backward throttle, and cable lax, retrieve
    {
       // main motor
       cMotor->setSpeed(255);
       cMotor->run(FORWARD);
       //active control motor
       cMotor_s->setSpeed((int)(m3m4_ratio_in * 255));
       cMotor_s->run(FORWARD);
    }
    else if(throttle > 0 && loadVal < -200) // in tension, release...
    {
      //main motor
       cMotor->setSpeed(255);
       cMotor->run(BACKWARD);
       //active control motor
       cMotor_s->setSpeed((int)(m3m4_ratio_out * 255));
       cMotor_s->run(BACKWARD);
    }
    else
    {
      //main motor
       cMotor->run(RELEASE);
       //active control motor
       cMotor_s->run(RELEASE);
    }
  }
}

void processCommand(char* input) {
  token = strtok(input, "|");
  int cnt = 0;
  while( token != NULL ) 
   {
      //Serial.println(token);
      if (cnt == 0)
      {
        if (token[0] != 'O')
        {
          Serial1.println("exiting...");
          break;
        }
      }
      else
      {
        if(cnt == 1)
          throttle = processSubCmd(token);
        else if(cnt == 2)
          continue; //by-pass the direction reading. 
          //direction = processSubCmd(token);
        else if(cnt == 3)
          cableCmd = processSubCmd(token);
      }
      
      token = strtok(NULL, "|");
      cnt = cnt + 1;
   }
}

float processSubCmd(char* input) {
  char sign = input[0];
  float cmdNum = atof(++input)/100.0;
  if(sign == '-')
    return cmdNum*-1;
  else
    return cmdNum;
}
