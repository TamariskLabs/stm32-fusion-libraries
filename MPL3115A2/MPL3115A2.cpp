#include "MPL3115A2.h"


//void constructure for instantiating new objeects of type MPL3115A2
MPL3115A2::MPL3115A2() {}


//retrives the contents of a specific location in Memory
int MPL3115A2::getRegister(byte registerLocation) {
  //read in current ctrl1 register
  Wire.beginTransmission(DEVICE_ID); // start transmission to device 
  Wire.write(registerLocation); 
  Wire.endTransmission(false); // end transmission

  uint16_t regData;
  //request the information from the control1 register
  Wire.requestFrom((uint8_t)DEVICE_ID, (uint8_t)1);// send data n-bytes read
  regData = Wire.read(); // receive DATA

  return regData;
}

void MPL3115A2::setRegister(byte registerLocation, int data) {
  Wire.beginTransmission(DEVICE_ID);
  Wire.write(registerLocation);
  Wire.write(data);
  Wire.endTransmission(false); // end transmission
}

//this function fetches a new pressure sample from the MPL3115A2
float MPL3115A2::getPressure() {
  uint32_t pressure; //variable that will hold the fetched pressure sample from the MPL3115A2

  //wait until new data is available
  uint8_t sta = 0;
  while (sta & STATUS_PDR_BIT == false) {
    sta = getRegister(STATUS_REGISTER);
    delay(1);
  }

  //set the base register to read from 
  Wire.beginTransmission(DEVICE_ID); // start transmission to device 
  Wire.write(MSB_PRESSURE_REGISTER); 
  Wire.endTransmission(false); // end transmission

  //request 3 bytes of pressure information starting from the base register location 
  Wire.requestFrom((uint8_t)DEVICE_ID, (uint8_t)3);
  pressure = Wire.read(); // receive DATA
  pressure <<= 8;
  pressure |= Wire.read(); // receive DATA
  pressure <<= 8;
  pressure |= Wire.read(); // receive DATA
  pressure >>= 4;

  //convert the data into units of pascals
  float baro = pressure;
  baro /= 4.0;
  return baro;
}


float MPL3115A2::getTempurature() {
  int16_t t;    //variable that will hold the fetched tempurature sample from the MPL3115A2
  
  //wait until new data is available
  uint8_t sta = 0;
  while (sta & STATUS_TDR_BIT == false) {
    sta = getRegister(STATUS_REGISTER);
    delay(1);
  }

  //set the base register to read from
  Wire.beginTransmission(DEVICE_ID); // start transmission to device 
  Wire.write(MSB_TEMPURATURE_REGISTER); 
  Wire.endTransmission(false); // end transmission

  //request 2 bytes of tempurature information starting from the base register location 
  Wire.requestFrom((uint8_t)DEVICE_ID, (uint8_t)2);// send data n-bytes read
  t = Wire.read(); // receive DATA
  t <<= 8;
  t |= Wire.read(); // receive DATA
  t >>= 4;

  //convert the data into units of celcius
  float temp = t;
  temp /= 16.0;
  return temp; 
}


float MPL3115A2::getPressureDelta() {
  uint32_t pressure; //variable that will hold the fetched pressure sample from the MPL3115A2

  //wait until new data is available
  uint8_t sta = 0;
  while (sta & STATUS_PDR_BIT == false) {
    sta = getRegister(STATUS_REGISTER);
    delay(1);
  }

  //set the base register to read from 
  Wire.beginTransmission(DEVICE_ID); // start transmission to device 
  Wire.write(MSB_DELTA_PRESSURE_REGISTER); 
  Wire.endTransmission(false); // end transmission

  //request 3 bytes of pressure information starting from the base register location 
  Wire.requestFrom((uint8_t)DEVICE_ID, (uint8_t)3);
  pressure = Wire.read(); // receive DATA
  pressure <<= 8;
  pressure |= Wire.read(); // receive DATA
  pressure <<= 8;
  pressure |= Wire.read(); // receive DATA
  pressure >>= 4;

  //convert the data into units of pascals
  float baro = pressure;
  baro /= 4.0;
  return baro;
}


float MPL3115A2::getTempuratureDelta() {
  int16_t t;    //variable that will hold the fetched tempurature sample from the MPL3115A2
  
  //wait until new data is available
  uint8_t sta = 0;
  while (sta & STATUS_TDR_BIT == false) {
    sta = getRegister(STATUS_REGISTER);
    delay(1);
  }

  //set the base register to read from
  Wire.beginTransmission(DEVICE_ID); // start transmission to device 
  Wire.write(MSB_DELTA_TEMPURATURE_REGISTER); 
  Wire.endTransmission(false); // end transmission

  //request 2 bytes of tempurature information starting from the base register location 
  Wire.requestFrom((uint8_t)DEVICE_ID, (uint8_t)2);// send data n-bytes read
  t = Wire.read(); // receive DATA
  t <<= 8;
  t |= Wire.read(); // receive DATA
  t >>= 4;

  //convert the data into units of celcius
  float temp = t;
  temp /= 16.0;
  return temp; 
}

//sets the device to be either in active mode or inactive mode
void MPL3115A2::setMode(mode setting) {
  //get the current status of the control register
  uint16_t reg;
  reg = getRegister(CTRL1_REGISTER);

  if(setting == ACTIVE) {
    reg = (reg | CTRL1_SBYB_BIT);
    setRegister(CTRL1_REGISTER, reg); 
  }
  if (setting == INACTIVE) {
    //set the mode bit to inactive
    reg = (reg & 0xFE);
    //set the mode bit to zero
    setRegister(CTRL1_REGISTER, reg); 
  }
}


//sets the over sample ratio of the barometer
void MPL3115A2::setOversampleRatio(overSampleRatio ratio) {
  //get the contents of the current control1 register
  int regData = getRegister(CTRL1_REGISTER);
  
  if(ratio == OS1) {
    //set all the OS bits to zero
    regData = (regData & CTRL1_OS_RANGE_MASK);
    //set OS bits to OS1
    regData = (regData | CTRL1_OS1_BIT);
    //set the control1 register
    setRegister(CTRL1_REGISTER, regData);
  }
  else if(ratio == OS2) {
    //set all the OS bits to zero
    regData = (regData & CTRL1_OS_RANGE_MASK);
    //set OS bits to OS2
    regData = (regData | CTRL1_OS2_BIT);
    //set the control1 register
    setRegister(CTRL1_REGISTER, regData);
  }
  else if(ratio == OS4) {
    //set all the OS bits to zero
    regData = (regData & CTRL1_OS_RANGE_MASK);
    //set OS bits to OS4
    regData = (regData | CTRL1_OS4_BIT);
    //set the control1 register
    setRegister(CTRL1_REGISTER, regData);
  }
  else if(ratio == OS8) {
    //set all the OS bits to zero
    regData = (regData & CTRL1_OS_RANGE_MASK);
    //set OS bits to OS8
    regData = (regData | CTRL1_OS8_BIT);
    //set the control1 register
    setRegister(CTRL1_REGISTER, regData);
  }
  else if(ratio == OS16) {
    //set all the OS bits to zero
    regData = (regData & CTRL1_OS_RANGE_MASK);
    //set OS bits to OS16
    regData = (regData | CTRL1_OS16_BIT);
    //set the control1 register
    setRegister(CTRL1_REGISTER, regData);
  }
  else if(ratio == OS32) {
    //set all the OS bits to zero
    regData = (regData & CTRL1_OS_RANGE_MASK);
    //set OS bits to OS32
    regData = (regData | CTRL1_OS32_BIT);
    //set the control1 register
    setRegister(CTRL1_REGISTER, regData);
  }
  else if(ratio == OS64) {
    //set all the OS bits to zero
    regData = (regData & CTRL1_OS_RANGE_MASK);
    //set OS bits to OS64
    regData = (regData | CTRL1_OS64_BIT);
    //set the control1 register
    setRegister(CTRL1_REGISTER, regData);
  }
  else if(ratio == OS128) {
    //set all the OS bits to zero
    regData = (regData & CTRL1_OS_RANGE_MASK);
    //set OS bits to OS128
    regData = (regData | CTRL1_OS128_BIT);
    //set the control1 register
    setRegister(CTRL1_REGISTER, regData);
  }
}




