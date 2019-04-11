#ifndef DS1923_h
#define DS1923_h

#include "OneWire.h"

#define ib_addr 0x41 // Since we already know the address type, just check against this. 0x41 is the address type for DS1923

class DS1923 
{
  
  private:
    byte present;
    boolean isWaiting; //waiting for convertion
    boolean isCmdSent; //conversation command sent
    unsigned long previousMillis;
    uint8_t addr[8];
    uint8_t data[9];
    uint8_t crc;
    int cnt;
    float temperature; // in Celsius
    float humidity;
    
  public:
    DS1923();
    boolean searchDev(OneWire ds);
    boolean readTempHum(OneWire ds, unsigned long curMillis);
    float getTemperature();
    float getHumidity(); 
    int err_code;
};

#endif
