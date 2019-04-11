#include "OneWire.h"
#include "DS1923.h"

DS1923::DS1923()
{
  addr[0] = 0x00;
  present = 0;
  temperature = 0.0;
  humidity = 0.0;
  cnt = 0;
  err_code = 0;
  isWaiting = isCmdSent = false;
}

boolean DS1923::searchDev(OneWire ds)
{
  cnt = 0;
  ds.reset();
  while(ds.search(addr)){
    cnt++;
    if(cnt > 128){
      ds.reset();
      ds.reset_search();
      return false;
    }
    if(ds.crc8(addr, 7) != addr[7]){
      continue;
    }

    if(addr[0] == ib_addr){
      ds.reset();
      ds.select(addr);
      //ds.write_bytes((uint8_t[]){0x69, 0x26, 0x02}, 3, false); 
      // Read Address 0226 (device type)
      ds.write(0x69);
      ds.write(0x26);
      ds.write(0x02);
      for (int i = 0; i < 8; i++) ds.write(0xFF);
      ds.read_bytes(data, 9);
      if (data[0] == 32) { // DS1923
        return true;
      } else {
        ds.reset();
        continue;
      }
    }
  }
}

boolean DS1923::readTempHum(OneWire ds, unsigned long curMillis)
{
  if(!isCmdSent){
    //forced conversation...
    ds.reset();
    ds.select(addr);
    ds.write(0x55);
    ds.write(0xFF,1);
    isWaiting = true;
    isCmdSent = true;
    previousMillis = curMillis;
    err_code = 10;
    return false;
  }

  //check time elapsed
  if(curMillis - previousMillis > 600)
    isWaiting = false;

  if(isCmdSent && !isWaiting) {
    //read temperature
    present = ds.reset();
    ds.select(addr);
    if(present){
      //ds.write_bytes((uint8_t[]){0x69, 0x0C, 0x02}, 3); 
      // Command to get memory with temperature data
      ds.write(0x69);
      ds.write(0x0C);
      ds.write(0x02);
      for (int i = 0; i < 8; i++) ds.write(0xFF);
      ds.read_bytes(data, 9); // Read temperature data
      crc = data[8]; // TODO:: check crc
      temperature = (((float)data[1] / 2.0f) - 41.0f) + ((float)data[0] / 512.0f);
    } else {
      ds.reset();
      err_code = 2;
      return false;
    }
  
    //read humidity
    present = ds.reset();
    ds.select(addr);
    if(present){
      //ds.write_bytes((uint8_t[]){0x69, 0x0E, 0x02}, 3); 
      // Command to get memory with temperature data
      ds.write(0x69);
      ds.write(0x0E);
      ds.write(0x02);
      for (int i = 0; i < 8; i++) ds.write(0xFF);
      ds.read_bytes(data, 9); // Read humidity data
      crc = data[8]; // TODO:: check crc
      humidity = (((((float)data[1] * 256.0f + (float)data[0])/16.0f) * 5.02/4096) - 0.958)/0.0307;
    } else {
      ds.reset();
      err_code = 3;
      return false;
    }
    
    return true;
  } else {
    err_code = 11;
    return false;
  }
}


float DS1923::getTemperature()
{
  isCmdSent = false;
  return temperature;
}


float DS1923::getHumidity()
{
  isCmdSent = false;
  return humidity;
}

