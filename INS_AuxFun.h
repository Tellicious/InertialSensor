//  INS_AuxFun.h
//
//
//  Created by Andrea Vivani on 13/3/16.
//  Copyright (c) 2016 Andrea Vivani. All rights reserved.
//
#ifndef _INS_AUXFUN_H_
#define _INS_AUXFUN_H_
#include "InertialSensor.h"
#include <stdint.h>
#include <math.h>

#ifdef INS_ARDUINO

//---------------Read one register from the SPI-----------------//
uint8_t INS_SPI_readRegister(const uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t readMask);

//------------Read multiple registers from the SPI--------------//
void INS_SPI_readMultipleRegisters(const uint8_t chipSelectPin, uint8_t* buffer, const uint8_t number_of_registers, uint8_t startRegister, const uint8_t readMultMask);

//---------------Write one register on the SPI-----------------//
void INS_SPI_writeRegister(const uint8_t chipSelectPin, const uint8_t thisRegister, const uint8_t thisValue, const uint8_t writeMask);

//---------------Read one register from the I2C-----------------//
uint8_t INS_I2C_readRegister(uint8_t address, uint8_t thisRegister);

//------------Read multiple registers from the I2C--------------//
void INS_I2C_readMultipleRegisters(uint8_t address, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister);

//---------------Write one register on the I2C-----------------//
uint8_t INS_I2C_writeRegister(uint8_t address, uint8_t thisRegister, const uint8_t thisValue);

//------------------Read data when ready--------------------//
#define INS_read_DRDY(timeout, rawfun, DRDY_pin) \
    uint32_t now = micros(); \
    while((micros() - now) < timeout){ \
      if (digitalRead(DRDY_pin)){ \
        rawfun(); \
        return 1; \
      } \
      if ((int32_t) (micros() - now) < 0){ \
        now = 0L; \
      } \
    } \
    return 0;

//---------Read data when ready (STATUS register)-----------//
#define INS_read_STATUS(timeout, rawfun, statusfun, statusMask) \
    uint32_t now = micros(); \
    while ((micros() - now) < timeout){ \
      if (statusfun() & statusMask){ \
        rawfun(); \
        return 1; \
      } \
    } \
    return 0;

//---------Read data when ready (STATUS register)-----------//
#define INS_read_STATUS_NOT(timeout, rawfun, statusfun, statusMask) \
    uint32_t now = micros(); \
    while ((micros() - now) < timeout){ \
      if (!(statusfun() & statusMask)){ \
        rawfun(); \
        return 1; \
      } \
    } \
    return 0;

//----------------Discard measures looking at overrided values-------------------//
#define INS_discard_measures_over(number_of_measures, timeout, rawfun, statusfun, statusMask) \
    uint32_t now = micros(); \
    while (count < (number_of_measures * 0.5f)){ \
      if (statusfun() & statusMask){ \
        rawfun(); \
        now = micros(); \
        count++; \
      } \
      if ((micros() - now) > timeout){ \
        return 0; \
      } \
      else if ((int32_t) (micros() - now) < 0){ \
        now = 0L; \
      } \
    } \
    return 1;

#elif defined(INS_CHIBIOS)

//---------------Read one register from the SPI-----------------//
uint8_t INS_SPI_readRegister(SPIDriver* SPI_int, const SPIConfig* spicfg,
                         uint8_t thisRegister, const uint8_t readMask);

//------------Read multiple registers from the SPI--------------//
void INS_SPI_readMultipleRegisters(SPIDriver* SPI_int, const SPIConfig* spicfg,
                               uint8_t* buffer,
                               const uint8_t number_of_registers,
                               uint8_t startRegister,
                               const uint8_t readMultMask);

//---------------Write one register on the SPI-----------------//
void INS_SPI_writeRegister(SPIDriver* SPI_int, const SPIConfig* spicfg,
                       const uint8_t thisRegister, const uint8_t thisValue,
                       const uint8_t writeMask);

//---------------Read one register from the I2C-----------------//
uint8_t INS_I2C_readRegister(I2CDriver* I2C_int, const I2CConfig* i2ccfg, uint8_t address, uint8_t thisRegister);

//------------Read multiple registers from the I2C--------------//
void INS_I2C_readMultipleRegisters(I2CDriver* I2C_int, const I2CConfig* i2ccfg, uint8_t address, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister);

//---------------Write one register on the I2C-----------------//
uint8_t INS_I2C_writeRegister(I2CDriver* I2C_int, const I2CConfig* i2ccfg, uint8_t address, uint8_t thisRegister, const uint8_t thisValue);

//------------------Read data when ready--------------------//
#define INS_read_DRDY(timeout, rawfun, gpio_DRDY, DRDY_pin) \
    systime_t end = chVTGetSystemTime() + US2ST(timeout); \
    while (chVTGetSystemTime() < end){ \
      if (palReadPad(gpio_DRDY, DRDY_pin)){ \
        rawfun(); \
        return 1; \
      } \
    } \
    return 0;

//---------Read data when ready (STATUS register)-----------//
#define INS_read_STATUS(timeout, rawfun, statusfun, statusMask) \
    systime_t end = chVTGetSystemTime() + US2ST(timeout); \
    while (chVTGetSystemTime() < end){ \
      if (statusfun() & statusMask){ \
        rawfun(); \
        return 1; \
      } \
    } \
    return 0;

//---------Read data when ready (STATUS register)-----------//
#define INS_read_STATUS_NOT(timeout, rawfun, statusfun, statusMask) \
    systime_t end = chVTGetSystemTime() + US2ST(timeout); \
    while (chVTGetSystemTime() < end){ \
      if (!(statusfun() & statusMask)){ \
        rawfun(); \
        return 1; \
      } \
    } \
    return 0;

//----------------Discard measures looking at overwritten values-------------------//
#define INS_discard_measures_over(number_of_measures, timeout, rawfun, statusfun, statusMask) \
    uint8_t count = 0; \
    systime_t end = chVTGetSystemTime() + US2ST(timeout); \
    while (count < (number_of_measures * 0.5f)){ \
      if (statusfun() & statusMask){ \
        rawfun(); \
        end = chVTGetSystemTime() + US2ST(timeout); \
        count++; \
      } \
      if (chVTGetSystemTime() > end){ \
        return 0; \
      } \
    } \
    return 1; \

#endif

//-----------------Check values for self-test-------------------//
uint8_t INS_ch_st(const float val1, const float val2, const float lim1,
                  const float lim2);

//-------------------Discard measures using readStatus----------------------//
#define  INS_discard_measures(number_of_measures, timeout, readStatusfun) \
    uint8_t count = 0; \
    while (count < number_of_measures){ \
      if(readStatusfun(timeout)){ \
        count++; \
      } \
      else{ \
        return 0; \
      } \
    } \
    return 1;

#endif //_INS_AUXFUN_H_
