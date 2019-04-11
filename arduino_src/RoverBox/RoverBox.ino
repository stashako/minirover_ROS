//**************************************************************************************
//**************************************************************************************
// *** cable length and load reading format
// "C|xxx.x,xxx.x" => cable length in meter, gauge load. 
//
// *** Temperature from DS1820
// "T|xxx.xx,xxx.xx" =>Celsius, Fahrenheit
//
// *** Humidity and temperature of iButton
// "I|xxx.xx,xxx.xx" =>Celsius, Humidity
//
// *** Pulse Count for RD2014 Radiation Sensor
// "R|xxxx" =>Count
//**************************************************************************************

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include "Q2HX711.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"



//*************Define statements to choose the hood that are attached******************//
#define DS1820 // temperature
//#define iButton // iButton
//#define RD2014 //Radiation Sensor




//**************************** DS1820 temperature *************************************//
#ifdef DS1820
#include "OneWire.h"
OneWire ds(13);
byte i;
byte data[12];
byte addr[8] = {0x00};
byte noVal = {0x00};
byte cfg;
int16_t raw;
float celsius, fahrenheit;
unsigned long lastUpdate_DS1820 = 0;
unsigned long interval_DS1820 = 1000;
#endif
//*************************************************************************************//

//**************************** iButton ************************************************//
#ifdef iButton
#include "OneWire.h"
#include "DS1923.h"
OneWire ds(13);
DS1923* ib;
boolean isDevfound = false;
unsigned long lastUpdate_IButton = 0; 
unsigned long interval_IButton = 1000;
#endif
//*************************************************************************************//

//**************************** RD2014 Radiation ***************************************//
#ifdef RD2014
#define pulsePin 13
int pulseCnt = 0;
unsigned long lastUpdate_RD2014 = 0; 
unsigned long interval_RD2014 = 1000;
#endif
//*************************************************************************************//





unsigned long curmillis = 0;


//**************************** MiniRover Motor ****************************************//
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
// Motor for the cable management system active control
Adafruit_DCMotor *cMotor_s = AFMS.getMotor(4);

// "O|Snum1|Snum2|Snum3" 
// SnumX => S-> pos/neg, numX-> two digits (033 equals to 0.33, with max=1.00)
// Snum1 = forward/backward of joy (forward -> pos)
// Snum2 = left/right of joy (right -> pos)
// Snum3 = in/out of cable (out -> pos)
#define INPUT_SIZE 16
char input[INPUT_SIZE];
char *token;

float throttle = 0;
float direction = 0;
float maxMotorScale = 0;
float leftMotor = 0;
float rightMotor = 0;
float leftMotorScaled = 0;
float rightMotorScaled = 0;
float deadZone = 0.05; 
float cableCmd = 0;
float m3m4_ratio_in = 0.5; //m4 = m3m4_ratio * m3
float m3m4_ratio_out = 0.8; //m4 = m3m4_ratio * m3


//**************************** Cable Management ****************************************//

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


//************** setup *******************//
void setup() {
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("loading ...");

  //read the first value of the opto encoder
  previousVal  = digitalRead(IR_DETECTOR_PIN);

  AFMS.begin();
  
#ifdef iButton
  ib = new DS1923();
  isDevfound = ib->searchDev(ds);
#endif

#ifdef RD2014
  pinMode(pulsePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pulsePin), pulseCount, CHANGE);
#endif

}


//************** loop *******************//
void loop() {
  //update the current time in milli seconds, Do not remove it !!
  curmillis = millis();

  //perform motor control
  motorControl();

  //read and send cable and load update
  getCableLength();
  getLoadCell();
  sendLoadLength();

//DS1820 block
#ifdef DS1820
  getDS1820Temp();
#endif  

//iButton block
#ifdef iButton
  getIbuttonReading();
#endif

//RD2014 Radiation Sensor
#ifdef RD2014
  getRD2014PulseCnt();
#endif
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
  direction = 0;
  
  if (Serial1.available())
  {
    memset(input, 0, sizeof(input));
    Serial1.readBytes(input, INPUT_SIZE);
    //Serial.println(input);
    processCommand(input);

    //mix throttle and direction
    leftMotor = throttle-direction;
    rightMotor = throttle+direction;
  
    //choose the max scale value if it is above 1
    maxMotorScale = max(leftMotor,rightMotor);
    maxMotorScale = max(1,maxMotorScale);

    //and apply it to the mixed values
    leftMotorScaled = constrain(leftMotor/maxMotorScale,-1,1);
    rightMotorScaled = constrain(rightMotor/maxMotorScale,-1,1);

    //Serial.print("throttle= ");
    //Serial.println(throttle);
    //Serial.print("dir= ");
    //Serial.println(direction);
    //Serial.print("left = ");
    //Serial.println(leftMotorScaled*100);
    //Serial.print("right = ");
    //Serial.println(rightMotorScaled*100);

    if(abs(leftMotorScaled) > deadZone)
    {
      if(leftMotorScaled < 0)
      {
        lMotor->setSpeed(leftMotorScaled*-255);
        lMotor->run(FORWARD);
      }
      else
      {
        lMotor->setSpeed(leftMotorScaled*255);
        lMotor->run(BACKWARD);
      }
    }
    else
    {
      lMotor->run(RELEASE);
    }
    
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
          direction = processSubCmd(token);
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

//**************************** DS1820 temperature method*************************************//
#ifdef DS1820
void getDS1820Temp(){
  
  if(curmillis - lastUpdate_DS1820 > interval_DS1820) {
    // save the last time
    lastUpdate_DS1820 = curmillis;  
  } else return;
  
  if ( addr[0] == noVal) {
    ds.search(addr);
    return;
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
       Serial.println("CRC is not valid!");
       ds.reset_search();
       return;
    }
    
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
    ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad

    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    raw = (data[1] << 8) | data[0];
    cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;

    char str[10];
    sprintf(str,"T|%5.2f,%5.2f",celsius,fahrenheit);
    Serial.println(str);
}
#endif


//**************************** DS1820 temperature method*************************************//
#ifdef iButton
void getIbuttonReading()
{
  if(curmillis - lastUpdate_IButton > interval_IButton) {
    // save the last time
    lastUpdate_IButton = curmillis;  
  } else return;
  
  if(!isDevfound){
    isDevfound = ib->searchDev(ds);
    Serial.println("device no available...");
  } else {
    if(ib->readTempHum(ds, curmillis)) {
      char str[10];
      sprintf(str,"I|%5.2f,%5.2f",ib->getTemperature(),ib->getHumidity());
      Serial.println(str);
    }else {
      if(ib->err_code != 10 && ib->err_code != 11) {
        Serial.print("no temp, err=");
        Serial.println(ib->err_code);
      } 
    }
  }
}
#endif



//**************************** RD2014 get Pulse Count *************************************//
#ifdef RD2014
void getRD2014PulseCnt()
{
  if(curmillis - lastUpdate_RD2014 > interval_RD2014) {
    // save the last time
    lastUpdate_RD2014 = curmillis;  
  } else return;
  
  char str[10];
  sprintf(str,"R|%d",pulseCnt);
  Serial.println(str);
  pulseCnt = 0;
}

void pulseCount() {
  pulseCnt += 1;          
}
#endif
