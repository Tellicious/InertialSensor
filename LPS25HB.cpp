//
//  LPS25HB.cpp
//
//
//  Created by Andrea Vivani on 20/6/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "LPS25HB.h"
#include "INS_AuxFun.h"
#ifdef INS_ARDUINO
#include "Arduino.h"
#include <SPI.h>
#endif
//====================================Registers Addresses=========================================// 
#define LPS25HB_REF_P_XL	0x08
#define LPS25HB_REF_P_L 	0x09
#define LPS25HB_REF_P_H 	0x0A
#define LPS25HB_WHO_AM_I	0x0F
#define LPS25HB_RES_CONF	0x10
#define LPS25HB_CTRL1		0x20
#define LPS25HB_CTRL2		0x21
#define LPS25HB_CTRL3		0x22
#define LPS25HB_CTRL4		0x23
#define LPS25HB_INT_CFG		0x24
#define LPS25HB_INT_SOURCE	0x25
#define LPS25HB_STATUS		0x27
#define LPS25HB_OUT_XL		0x28
#define LPS25HB_OUT_L		0x29
#define LPS25HB_OUT_H		0x2A
#define LPS25HB_TEMP_OUT_L	0x2B
#define LPS25HB_TEMP_OUT_H	0x2C
#define LPS25HB_FIFO_CTRL	0x2E
#define LPS25HB_FIFO_STATUS	0x2F
#define LPS25HB_THS_PL		0x30
#define LPS25HB_THS_PH		0x31
#define LPS25HB_RPDS_L		0x39
#define LPS25HB_RPDS_H		0x3A
//=======================================Constants=============================================// 
#define LPS25HB_ID			0xBD
#define LPS25HB_READ		0x80
#define LPS25HB_MULT		0x40
//==================================Auxiliary Functions========================================//
#ifdef INS_ARDUINO
  #define LPS25HB_READ_REGISTER(reg) INS_SPI_readRegister(_chipSelectPin, reg, LPS25HB_READ)
  #define LPS25HB_READ_MULTIPLE_REGISTERS(buf, num, start) INS_SPI_readMultipleRegisters(_chipSelectPin, buf, num, startRegister, (LPS25HB_READ | LPS25HB_MULT))
  #define LPS25HB_WRITE_REGISTER(reg, val) INS_SPI_writeRegister(_chipSelectPin, reg, val, 0x00)
#elif defined(INS_CHIBIOS)
  #define LPS25HB_READ_REGISTER(reg) INS_SPI_readRegister(_SPI_int, _spicfg, reg, LPS25HB_READ)
  #define LPS25HB_READ_MULTIPLE_REGISTERS(buf, num, start) INS_SPI_readMultipleRegisters(_SPI_int, _spicfg, buf, num, start, (LPS25HB_READ | LPS25HB_MULT))
  #define LPS25HB_WRITE_REGISTER(reg, val) INS_SPI_writeRegister(_SPI_int, _spicfg, reg, val, 0x00)
#endif
//=====================================Constructors==========================================//
#ifdef INS_ARDUINO
LPS25HB::LPS25HB (uint8_t CS_pin):InertialSensor(), BarometerSensor(), ThermometerSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = 0;
}

LPS25HB::LPS25HB (uint8_t CS_pin, uint8_t DRDY_pin):InertialSensor(), BarometerSensor(), ThermometerSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = DRDY_pin;
}

//-----------------------Initialization-----------------------//
void LPS25HB::init(){
	pinMode(_chipSelectPin,OUTPUT);
	digitalWrite(_chipSelectPin,HIGH);
	if (_DRDY_pin != 0){
		pinMode(_DRDY_pin,INPUT);
	}
	press = 0;
	temperature = 0;
}
#elif defined(INS_CHIBIOS)
LPS25HB::LPS25HB (SPIDriver* SPI, SPIConfig* spicfg):InertialSensor(), BarometerSensor(), ThermometerSensor(){
	_SPI_int = SPI;
	_spicfg = spicfg;
	_DRDY_pin = 0;
    init();
}

LPS25HB::LPS25HB (SPIDriver* SPI, SPIConfig* spicfg, ioportid_t gpio_DRDY, uint8_t DRDY_pin):InertialSensor(), BarometerSensor(), ThermometerSensor(){
	_SPI_int = SPI;
	_spicfg = spicfg;
	_gpio_DRDY = gpio_DRDY;
	_DRDY_pin = DRDY_pin;
	init();
}

//-----------------------Initialization-----------------------//
void LPS25HB::init(){
	palSetPad(_spicfg->ssport, _spicfg->sspad);
	palSetPadMode(_spicfg->ssport, _spicfg->sspad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	if (_DRDY_pin != 0){
		palSetPadMode(_gpio_DRDY, _DRDY_pin, PAL_MODE_INPUT);
	}
	press = 0;
	temperature = 0;
}
#endif
//===================================Public Members=========================================//
//-----------------------Configuration-----------------------//
uint8_t LPS25HB::config_baro(uint8_t odr_conf, uint8_t AVGT, uint8_t AVGP, uint8_t MA_FIFO){
	init();
	// Trash the first reading
	LPS25HB_READ_REGISTER(LPS25HB_WHO_AM_I);
	// Check if the device ID is correct
	if (LPS25HB_READ_REGISTER(LPS25HB_WHO_AM_I)!= LPS25HB_ID){
		return 0;
	}
	//
	//selected averages
	uint8_t RES_CONF_val = (AVGT | AVGP) & 0x0F; 
	LPS25HB_WRITE_REGISTER(LPS25HB_RES_CONF,RES_CONF_val);
	//
	//FIFO enabled, I2C disabled, autozero off, one-shot off
	uint8_t CTRL2_val = (1 << 6);
	LPS25HB_WRITE_REGISTER(LPS25HB_CTRL2,CTRL2_val);
	//
	//Interrupt active high, push pull, data signal on interrupt
	uint8_t CTRL3_val = (1 << 3);
	LPS25HB_WRITE_REGISTER(LPS25HB_CTRL3,CTRL3_val);
	//
	//DRDY on interrupt
	uint8_t CTRL4_val = 1; 
	LPS25HB_WRITE_REGISTER(LPS25HB_CTRL4,CTRL4_val);
	//
	uint8_t FIFO_CTRL_val = 0; 
	if (MA_FIFO != 0){
		//FIFO on moving average with selected watermark
		FIFO_CTRL_val = 0xC0 | MA_FIFO;
	}
	else{
		//FIFO on bypass mode
		FIFO_CTRL_val = 0x00;
	}
	LPS25HB_WRITE_REGISTER(LPS25HB_FIFO_CTRL,FIFO_CTRL_val);
	//
	//power on, selected ODR, continuous update, SPI 4 wire
	_CTRL1_val = (1 << 7) | odr_conf | (1 << 3);
	turn_on_baro();
	// Discard the first n measures
	if(! discard_measures_baro(LPS25HB_DISCARDED_MEASURES,LPS25HB_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-------------------------Turn on---------------------------//
void LPS25HB::turn_on_baro(){
	LPS25HB_WRITE_REGISTER(LPS25HB_CTRL1,_CTRL1_val);
#ifdef INS_ARDUINO
	delay(37);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(37);
#endif
}

//------------------------Turn off---------------------------//
void LPS25HB::turn_off_baro(){
	LPS25HB_WRITE_REGISTER(LPS25HB_CTRL1,(_CTRL1_val & 0x7F));
}

//------------------------Read data-------------------------//
uint8_t LPS25HB::read_raw_baro(){
	uint8_t buffer[3];
  	LPS25HB_READ_MULTIPLE_REGISTERS(buffer, 3, LPS25HB_OUT_XL);
  	press = (float) (((int32_t) (int8_t) buffer[2] << 16 | (uint16_t) buffer[1] << 8 | buffer[0]) * _sc_fact);
  	return 1;
}

//------------------Read data when ready--------------------//
uint8_t LPS25HB::read_baro_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
    INS_read_DRDY(timeout, read_raw_baro, _DRDY_pin);
#elif defined(INS_CHIBIOS)
    INS_read_DRDY(timeout, read_raw_baro, _gpio_DRDY, _DRDY_pin);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LPS25HB::read_baro_STATUS(uint32_t timeout){
  INS_read_STATUS(timeout, read_raw_baro, status_baro, (1 << 1));
}

//----------------------Baro Status------------------------//
uint8_t LPS25HB::status_baro(){
	return LPS25HB_READ_REGISTER(LPS25HB_STATUS);
}

//-------------------Discard measures----------------------//
uint8_t LPS25HB::discard_measures_baro(uint8_t number_of_measures, uint32_t timeout){
  INS_discard_measures_over(number_of_measures, timeout, read_raw_baro, status_baro, (1 << 5));
}

//=============================Public Members Temperature====================================//
//------------------------Read data----------------------//
uint8_t LPS25HB::read_raw_thermo(){
	uint8_t buffer[2];
	LPS25HB_READ_MULTIPLE_REGISTERS(buffer, 2, LPS25HB_TEMP_OUT_L);
	int16_t temperature_tmp = (((int16_t) buffer[1] << 8) | buffer[0]);
	temperature = 42.5f + (float) temperature_tmp / 480.0f;
	return 1;
}

//------------------Read data when ready-----------------//
uint8_t LPS25HB::read_thermo_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
    INS_read_DRDY(timeout, read_raw_thermo, _DRDY_pin);
#elif defined(INS_CHIBIOS)
    INS_read_DRDY(timeout, read_raw_thermo, _gpio_DRDY, _DRDY_pin);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LPS25HB::read_thermo_STATUS(uint32_t timeout){
    INS_read_STATUS(timeout, read_raw_thermo, status_baro, 0x01);
}

//-------------------Discard measures----------------------//
uint8_t LPS25HB::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
  INS_discard_measures_over(number_of_measures, timeout, read_raw_thermo, status_baro, (1 << 4));
}
