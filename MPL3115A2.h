//Library for interacting with the MPL3115A2 Barometric Sensor
//This Library was written by Tamarisk Labs, LLC


//inclusion guard
#ifndef MPL3115A2_h
#define MPL3115A2_h

//import version appropriate arduino library
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

//import board appropriate version of i2c library
#ifdef __AVR_ATtiny85__
 #include <TinyWireM.h>
#else
 #include <Wire.h>
#endif


//the 7 bit i2c address of the device
#define DEVICE_ID 0x60   
           
//register map
//register addresses end in _REGISTER
//specific register bitmasks end in _BIT
#define STATUS_REGISTER  0x00          //register indicating when new data is availible 
#define STATUS_TDR_BIT  0x02           //indicates that new temperature sample is available
#define STATUS_PDR_BIT 0x04            //indicates that a new pressure sample is available 
#define STATUS_PTDR_BIT 0x08           //indicates that either a new pressure or temperature sample is available
#define MSB_PRESSURE_REGISTER 0x01     //register containing bits 12-19 of the last pressure sample
#define CSB_PRESSURE_REGISTER 0x02          //register containing bits 4-11 of the last pressure sample 
#define LSB_PRESSURE_REGISTER 0x03          //register containing bits 0-3 of the last pressure sample
#define MSB_TEMPURATURE_REGISTER 0x04       //register containing bits 4-11 of the last tempurature sample
#define LSB_TEMPURATURE_REGISTER 0x05       //register containing bits 0-3 of the last tempurature sample
#define DATA_READY_SENSOR_REGISTER 0x06     //register indicating when new data is ready 
#define MSB_DELTA_PRESSURE_REGISTER 0x07    //register containing bits 12-19 of delta pressure change data
#define CBS_DELTA_PRESSURE_REGISTER 0x08    //register containing bits 4-11 of delta pressure change data
#define LSB_DELTA_PRESSURE_REGISTER 0x09    //register containing bits 0-3 of delta pressure change data
#define MSB_DELTA_TEMPURATURE_REGISTER 0x0A     //register containing bits 4-11 of delta tempurature change data
#define LSB_DELTA_TEMPURATURE_REGISTER 0x0B     //register containing bits 0-3 of delta tempurature change data
#define WHO_AM_I_REGISTER 0x0C                  //register containing the fixed device number, this is 0xC4
#define CFG_REGISTER 0x13                       //configures the pressure and tempurature data event flag generator
#define CFG_TDEFE_BIT 0x01                      //enable/disable event flag on new tempurature data
#define CFG_PDEFE_BIT 0x02                      //enable/disable event flag on new pressure data
#define CFG_DREM_BIT 0x04                       //enable/disable event flag on either new tempurature or pressure data
#define CTRL1_REGISTER 0x26                     //configures device modes, and oversampling
#define CTRL1_SBYB_BIT 0x01                     //sets the device into active mode
#define CTRL1_OST_BIT 0x02                      //commands the device to take an immediate reading
#define CTRL1_RST_BIT 0x04                      //activates a software reset
#define CTRL1_OS_RANGE_MASK 0xC7                //sets all oversample bits to 0
#define CTRL1_OS1_BIT 0x00                      //sets oversample ratio to 1, minimum time between samples is 6ms
#define CTRL1_OS2_BIT 0x08                      //sets oversample ratio to 2, minimum time between samples is 10ms
#define CTRL1_OS4_BIT 0x10                      //sets oversample ratio to 4, minimum time between samples is 18ms
#define CTRL1_OS8_BIT 0x18                      //sets oversample ratio to 8, minimum time between samples is 34ms
#define CTRL1_OS16_BIT 0x20                     //sets oversample ratio to 16, minimum time between samples is 66ms
#define CTRL1_OS32_BIT 0x28                     //sets oversample ratio to 32, minimum time between samples is 130ms
#define CTRL1_OS64_BIT 0x30                     //sets oversample ratio to 64, minimum time between samples is 258ms
#define CTRL1_OS128_BIT 0x38                    //sets oversample ratio to 128, minimum time between samples is 512ms
#define CTRL1_RAW_BIT 0x40                      //sets the output mode to be raw ADC data
#define CTRL1_ALT_BIT 0x80                      //sets the output mode to be in altitude from sealevel
#define CTRL1_BAR_BIT 0x00                      //sets the output mode to be in pressure
#define CTRL2_REGISTER 0x27                     //configures device aquisition time
#define CTRL3_REGISTER 0x28                     //configures inturrupt pin configuration
#define CTRL4_REGISTER 0x29                     //enable/disable event inturrupts
#define CTRL5_REGISTER 0x2A                     //configures output pin assignment for inturrupts

//Enumerations
enum overSampleRatio {OS1, OS2, OS4, OS8, OS16, OS32, OS64, OS128};
enum mode {ACTIVE, INACTIVE};
enum inturrupt {DATA_READY, FIFO, PRESSURE_THRESHOLD, 
                TEMPURATURE_THREASHOLD, PRESSURE_WINDOW, 
                TEMPURATURE_WINDOW, PRESSURE_CHANGE, TEMPURATURE_CHANGE};
enum outputType {ACTIVE_LOW, ACTIVE_HIGH, INTERNAL_PULLUP, OPEN_DRAIN};


class MPL3115A2{
  public:
    MPL3115A2();
    float getPressure();
    float getTempurature();                     //get a tempurature reading from the device
    float getPressureDelta();                   //get the pressure difference between the lattest two pressure samples
    float getTempuratureDelta();                //get the tempurature difference between the lattest two tempurature samples
    void setMode(mode);                         //set the device into active and in-active mode
    void setOversampleRatio(overSampleRatio);   //set the sample rate of the device
    //void setInturruptPinAssignment();           //configures which pins inturrupts are aserted on
    //void enableInturrupt(inturrupt);            //enables the specified inturrupt source
    // void dissableInturrupt(inturrupt);          //disables the specified inturrupt source
    //void inturruptPinConfiguration();           //sets the output type for each inturrupt source 
  private:     
   enum type {BAROMETER, ALTIMITER, NOTSET};    //parameter type used to keep track of the current state of the reading type
   type pressureMode = NOTSET;                  //parameter used to keep track of the current state of the reading type
   int getRegister(byte);                       //returns the contents of the specified register
   void setRegister(byte, int);                //sets the contents of a specific register
    
  
};

//end of inclusion guard
#endif
