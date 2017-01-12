//  INS_AuxFun.cpp
//
//
//  Created by Andrea Vivani on 13/3/16.
//  Copyright (c) 2016 Andrea Vivani. All rights reserved.
//
#include "INS_Auxfun.h"

//================================================ARDUINO================================================//
#ifdef INS_ARDUINO

//---------------Read one register from the SPI-----------------//
uint8_t INS_SPI_readRegister(const uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t readMask) {
  uint8_t inByte = 0;             // incoming byte
  thisRegister |= readMask;       // register in read mode
  digitalWrite(chipSelectPin, LOW);  // ChipSelect low to select the chip
  SPI.transfer(thisRegister);     // send the command to read thisRegister
  inByte = SPI.transfer(0x00);        // send 0x00 in order to read the incoming byte
  digitalWrite(chipSelectPin, HIGH); // ChipSelect high to select the chip
  return(inByte);         // return the read byte
}

//------------Read multiple registers from the SPI--------------//
void INS_SPI_readMultipleRegisters(const uint8_t chipSelectPin, uint8_t* buffer, const uint8_t number_of_registers, uint8_t startRegister, const uint8_t readMultMask) {
  startRegister |= readMultMask;// register in multiple read mode
  digitalWrite(chipSelectPin, LOW);  // ChipSelect low to select the chip
  SPI.transfer(startRegister);        // send the command to read thisRegister
  while (number_of_registers--){
    *buffer++ = SPI.transfer(0x00);
  }
  digitalWrite(chipSelectPin, HIGH); // ChipSelect high to deselect the chip
  return;
}

//---------------Write one register on the SPI-----------------//
void INS_SPI_writeRegister(const uint8_t chipSelectPin, const uint8_t thisRegister, const uint8_t thisValue, const uint8_t writeMask) {
  digitalWrite(_chipSelectPin, LOW);  // ChipSelect low to select the chip
  SPI.transfer(thisRegister | writeMask);         // send register location
  SPI.transfer(thisValue);        // send value to record into register
  digitalWrite(_chipSelectPin, HIGH); // ChipSelect high to select the chip
  return;
}

//---------------Read one register from the I2C-----------------//
uint8_t INS_I2C_readRegister(uint8_t address, uint8_t thisRegister) {
  uint8_t inByte = 0;     // incoming byte
  Wire.beginTransmission(address); //start transmission to the device
  Wire.write(thisRegister); //register address to read from
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(address, 1);   //request 1 byte
  inByte = Wire.read();       //read the incoming byte
  Wire.endTransmission();
  return(inByte);         // return the read byte
}

//------------Read multiple registers from the I2C--------------//
void INS_I2C_readMultipleRegisters(uint8_t address, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister) {
  Wire.beginTransmission(address); //start transmission to the device
  Wire.write(startRegister); //register address to read from
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(address, number_of_registers); //request n byte
  while (number_of_registers--){
    *buffer++ = Wire.read();        //read the incoming bytes
  }
  Wire.endTransmission(); //end transmission
  return;
}

//---------------Write one register on the I2C-----------------//
uint8_t INS_I2C_writeRegister(uint8_t address, uint8_t thisRegister, const uint8_t thisValue) {
  Wire.beginTransmission(address); //start transmission to the device
  Wire.write(thisRegister); //register address where to write
  Wire.write(thisValue); //write register value
  return (! Wire.endTransmission());  //end transmission
}

//================================================CHIBIOS================================================//
#elif defined(INS_CHIBIOS)

//---------------Read one register from the SPI-----------------//
uint8_t INS_SPI_readRegister(SPIDriver* SPI_int, const SPIConfig* spicfg, uint8_t thisRegister, const uint8_t readMask) {
  uint8_t inByte = 0;             // incoming byte
  thisRegister |= readMask;       // register in read mode
  spiAcquireBus(SPI_int);
  spiStart(SPI_int, spicfg);
  spiSelect(SPI_int);
  spiSend(SPI_int, 1, &thisRegister);
  spiReceive(SPI_int, 1, &inByte);
  spiUnselect(SPI_int);
  spiReleaseBus(SPI_int);
  return (inByte);
}

//------------Read multiple registers from the SPI--------------//
void INS_SPI_readMultipleRegisters(SPIDriver* SPI_int, const SPIConfig* spicfg, uint8_t* buffer, const uint8_t number_of_registers, uint8_t startRegister, const uint8_t readMultMask) {
  startRegister |= readMultMask;// register in multiple read mode
  spiAcquireBus(SPI_int);
  spiStart(SPI_int, spicfg);
  spiSelect(SPI_int);
  spiSend(SPI_int, 1, &startRegister);
  spiReceive(SPI_int, number_of_registers, buffer);
  spiUnselect(SPI_int);
  spiReleaseBus(SPI_int);
  return;
}

//---------------Write one register on the SPI-----------------//
void INS_SPI_writeRegister(SPIDriver* SPI_int, const SPIConfig* spicfg, const uint8_t thisRegister, const uint8_t thisValue, const uint8_t writeMask) {
  uint8_t buffer [2];
  buffer [0] = (thisRegister | writeMask);
  buffer [1] = thisValue;
  spiAcquireBus(SPI_int);
  spiStart(SPI_int, spicfg);
  spiSelect(SPI_int);
  spiSend(SPI_int, 2, buffer);
  spiUnselect(SPI_int);
  spiReleaseBus(SPI_int);
  return;
}

//---------------Read one register from the I2C-----------------//
uint8_t INS_I2C_readRegister(I2CDriver* I2C_int, const I2CConfig* i2ccfg, uint8_t address, uint8_t thisRegister) {
  uint8_t inByte = 0;     // incoming byte
  i2cAcquireBus(I2C_int);
  i2cStart(I2C_int, i2ccfg);
  i2cMasterTransmitTimeout (I2C_int, address, &thisRegister, 1, &inByte, 1, TIME_INFINITE);
  return(inByte);         // return the read byte
}

//------------Read multiple registers from the I2C--------------//
void INS_I2C_readMultipleRegisters(I2CDriver* I2C_int, const I2CConfig* i2ccfg, uint8_t address, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister) {
  i2cAcquireBus(I2C_int);
  i2cStart(I2C_int, i2ccfg);
  i2cMasterTransmitTimeout (I2C_int, address, &startRegister, 1, buffer, number_of_registers, TIME_INFINITE);
  i2cReleaseBus(I2C_int);
  return;
}

//---------------Write one register on the I2C-----------------//
uint8_t INS_I2C_writeRegister(I2CDriver* I2C_int, const I2CConfig* i2ccfg, uint8_t address, uint8_t thisRegister, const uint8_t thisValue) {
  uint8_t buffer [2];
  buffer [0] = thisRegister;
  buffer [1] = thisValue;
  i2cAcquireBus(I2C_int);
  i2cStart(I2C_int, i2ccfg);
  msg_t status = i2cMasterTransmitTimeout (I2C_int, address, buffer, 2, NULL, 0, TIME_INFINITE);
  i2cReleaseBus(I2C_int);
  return ((status == MSG_OK) ? 1 : 0);  //end transmission
}

#endif

//================================================COMMON================================================//
//-----------------Check values for self-test-------------------//
uint8_t INS_ch_st(const float val1, const float val2, const float lim1, const float lim2){
  if (fabs(lim1) > fabs(lim2)){
    return ((fabs(val2 - val1) >= fabs(lim2)) && (fabs(val2 - val1) <= fabs(lim1)));
  }
  return ((fabs(val2 - val1) >= fabs(lim1)) && (fabs(val2 - val1) <= fabs(lim2)));
}
