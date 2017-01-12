//
//  LIS3MDL.cpp
//
//
//  Created by Andrea Vivani on 7/7/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "LIS3MDL.h"
#include "INS_AuxFun.h"
#ifdef INS_ARDUINO
#include "Arduino.h"
#include <SPI.h>
#endif
//====================================Registers Addresses=========================================// 
#define LIS3MDL_OFFSET_X_REG_L	0x05
#define LIS3MDL_OFFSET_X_REG_H	0x06
#define LIS3MDL_OFFSET_Y_REG_L	0x07
#define LIS3MDL_OFFSET_Y_REG_H	0x08
#define LIS3MDL_OFFSET_Z_REG_L	0x09
#define LIS3MDL_OFFSET_Z_REG_H	0x0A
#define LIS3MDL_WHO_AM_I	0x0F
#define LIS3MDL_CTRL1		0x20
#define LIS3MDL_CTRL2		0x21
#define LIS3MDL_CTRL3		0x22
#define LIS3MDL_CTRL4		0x23
#define LIS3MDL_CTRL5		0x24
#define LIS3MDL_STATUS		0x27
#define LIS3MDL_OUT_XL		0x28
#define LIS3MDL_OUT_XH		0x29
#define LIS3MDL_OUT_YL		0x2A
#define LIS3MDL_OUT_YH		0x2B
#define LIS3MDL_OUT_ZL		0x2C
#define LIS3MDL_OUT_ZH		0x2D
#define LIS3MDL_TEMP_OUT_L	0x2E
#define LIS3MDL_TEMP_OUT_H	0x2F
#define LIS3MDL_INT_CFG		0x30
#define LIS3MDL_INT_SRC		0x31
#define LIS3MDL_INT_THS_L	0x32
#define LIS3MDL_INT_THS_H	0x33
//=======================================Constants=============================================// 
#define LIS3MDL_ID			0x3D
#define LIS3MDL_READ		0x80
#define LIS3MDL_MULT		0x40

//==================================Auxiliary Functions========================================//
#ifdef INS_ARDUINO
  #define LIS3MDL_READ_REGISTER(reg) INS_SPI_readRegister(_chipSelectPin, reg, LIS3MDL_READ)
  #define LIS3MDL_READ_MULTIPLE_REGISTERS(buf, num, start) INS_SPI_readMultipleRegisters(_chipSelectPin, buf, num, startRegister, (LIS3MDL_READ | LIS3MDL_MULT))
  #define LIS3MDL_WRITE_REGISTER(reg, val) INS_SPI_writeRegister(_chipSelectPin, reg, val, 0x00)
#elif defined(INS_CHIBIOS)
  #define LIS3MDL_READ_REGISTER(reg) INS_SPI_readRegister(_SPI_int, _spicfg, reg, LIS3MDL_READ)
  #define LIS3MDL_READ_MULTIPLE_REGISTERS(buf, num, start) INS_SPI_readMultipleRegisters(_SPI_int, _spicfg, buf, num, start, (LIS3MDL_READ | LIS3MDL_MULT))
  #define LIS3MDL_WRITE_REGISTER(reg, val) INS_SPI_writeRegister(_SPI_int, _spicfg, reg, val, 0x00)
#endif
//=====================================Constructors==========================================//
#ifdef INS_ARDUINO
LIS3MDL::LIS3MDL (uint8_t CS_pin):InertialSensor(), MagnetometerSensor(), BarometerSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = 0;
}

LIS3MDL::LIS3MDL (uint8_t CS_pin, uint8_t DRDY_pin):InertialSensor(), MagnetometerSensor(), BarometerSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = DRDY_pin;
}

//-----------------------Initialization-----------------------//
void LIS3MDL::init(){
	pinMode(_chipSelectPin,OUTPUT);
	digitalWrite(_chipSelectPin,HIGH);
	if (_DRDY_pin != 0){
		pinMode(_DRDY_pin,INPUT);
	}
	x = 0;
	y = 0;
	z = 0;
	temperature = 0;
}
#elif defined(INS_CHIBIOS)
LIS3MDL::LIS3MDL (SPIDriver* SPI, SPIConfig* spicfg):InertialSensor(), MagnetometerSensor(), BarometerSensor(){
	_SPI_int = SPI;
	_spicfg = spicfg;
	_DRDY_pin = 0;
    init();
}

LIS3MDL::LIS3MDL (SPIDriver* SPI, SPIConfig* spicfg, ioportid_t gpio_DRDY, uint8_t DRDY_pin):InertialSensor(), MagnetometerSensor(), BarometerSensor(){
	_SPI_int = SPI;
	_spicfg = spicfg;
	_gpio_DRDY = gpio_DRDY;
	_DRDY_pin = DRDY_pin;
	init();
}

//-----------------------Initialization-----------------------//
void LIS3MDL::init(){
	palSetPad(_spicfg->ssport, _spicfg->sspad);
	palSetPadMode(_spicfg->ssport, _spicfg->sspad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	if (_DRDY_pin != 0){
		palSetPadMode(_gpio_DRDY, _DRDY_pin, PAL_MODE_INPUT);
	}
	x = 0;
	y = 0;
	z = 0;
	temperature = 0;
}
#endif

//===================================Public Members=========================================//
//-----------------------Configuration-----------------------//
uint8_t LIS3MDL::config_mag(uint8_t range_conf, uint8_t odr_conf){
	init();
	// Trash the first reading
	LIS3MDL_READ_REGISTER(LIS3MDL_WHO_AM_I);
	// Check if the device ID is correct
	if (LIS3MDL_READ_REGISTER(LIS3MDL_WHO_AM_I) != LIS3MDL_ID){
		return 0;
	}
	//
	//selected range
	uint8_t CTRL2_val = range_conf & 0x60;
	LIS3MDL_WRITE_REGISTER(LIS3MDL_CTRL2, CTRL2_val);
	switch (range_conf){
		case (LIS3MDL_RANGE_4):
			_sc_fact = 1.0f / 6842.0f;
			break;
		case (LIS3MDL_RANGE_8):
			_sc_fact = 1.0f / 3421.0f;
			break;
		case (LIS3MDL_RANGE_12):
			_sc_fact = 1.0f / 2281.0f;
			break;
		case (LIS3MDL_RANGE_16):
			_sc_fact = 1.0f / 1711.0f;
			break;
		default:
		return 2;
	}
	//
	//continuous update
	uint8_t CTRL5_val = 0x00;
	LIS3MDL_WRITE_REGISTER(LIS3MDL_CTRL5, CTRL5_val);
	//
	//Z-axis on selected performance mode, little endian
	uint8_t CTRL4_val = ((odr_conf & 0x60) >> 3);
	LIS3MDL_WRITE_REGISTER(LIS3MDL_CTRL4, CTRL4_val);
	//
	//temperature enable, selected performance mode, selected ODR, no self test
	uint8_t CTRL1_val = (1 << 7) | odr_conf;
	LIS3MDL_WRITE_REGISTER(LIS3MDL_CTRL1, CTRL1_val);
	//
	// clearing offset registers
	LIS3MDL_WRITE_REGISTER(LIS3MDL_OFFSET_X_REG_L, 0x00);
	LIS3MDL_WRITE_REGISTER(LIS3MDL_OFFSET_X_REG_H, 0x00);
	LIS3MDL_WRITE_REGISTER(LIS3MDL_OFFSET_Y_REG_L, 0x00);
	LIS3MDL_WRITE_REGISTER(LIS3MDL_OFFSET_Y_REG_H, 0x00);
	LIS3MDL_WRITE_REGISTER(LIS3MDL_OFFSET_Z_REG_L, 0x00);
	LIS3MDL_WRITE_REGISTER(LIS3MDL_OFFSET_Z_REG_H, 0x00);
	//
	//power on, SPI 4 wire, Continuous conversion mode
	_CTRL3_val = 0x00;
	turn_on_mag();
	// Discard the first n measures
	if(! discard_measures_mag(LIS3MDL_DISCARDED_MEASURES, LIS3MDL_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-------------------------Turn on---------------------------//
void LIS3MDL::turn_on_mag(){
	LIS3MDL_WRITE_REGISTER(LIS3MDL_CTRL3, _CTRL3_val);
#ifdef INS_ARDUINO
	delay(60);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(60);
#endif
}

//------------------------Turn off---------------------------//
void LIS3MDL::turn_off_mag(){
	LIS3MDL_WRITE_REGISTER(LIS3MDL_CTRL3, (_CTRL3_val | 0x03));
}

//------------------------Read data-------------------------//
uint8_t LIS3MDL::read_raw_mag(){
	uint8_t buffer[6];
  	LIS3MDL_READ_MULTIPLE_REGISTERS(buffer, 6, LIS3MDL_OUT_XL);
  	x = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact);
  	y = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact);
  	z = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact);
  	return 1;
}

//------------------Read data when ready--------------------//
uint8_t LIS3MDL::read_mag_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
    INS_read_DRDY(timeout, read_raw_mag, _DRDY_pin);
#elif defined(INS_CHIBIOS)
    INS_read_DRDY(timeout, read_raw_mag, _gpio_DRDY, _DRDY_pin);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LIS3MDL::read_mag_STATUS(uint32_t timeout){
  INS_read_STATUS(timeout, read_raw_mag, status_mag, (1 << 3));
}

//-----------------------Self-Test-------------------------//
uint8_t LIS3MDL::self_test_mag(){
	uint8_t status = 0;
	// Use FS = 12 Gauss
	uint8_t CTRL2_old = LIS3MDL_READ_REGISTER(LIS3MDL_CTRL2);
	uint8_t CTRL2_val = LIS3MDL_RANGE_12 | (CTRL2_old & 0x0F);
	LIS3MDL_WRITE_REGISTER(LIS3MDL_CTRL2, CTRL2_val);
	// Discard the first n measures
	if(!discard_measures_mag(LIS3MDL_DISCARDED_MEASURES_ST, LIS3MDL_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (uint8_t ii = 0; ii < LIS3MDL_MAG_SELF_TEST_MEASURES; ii++){
		read_mag_STATUS(LIS3MDL_DISCARD_TIMEOUT);
		x_pre += x;
		y_pre += y;
		z_pre += z;
	}
	x_pre /= (LIS3MDL_MAG_SELF_TEST_MEASURES * _sc_fact * 2281.0f); // average and revert to FS = 12 Gauss
	y_pre /= (LIS3MDL_MAG_SELF_TEST_MEASURES * _sc_fact * 2281.0f); // average and revert to FS = 12 Gauss
	z_pre /= (LIS3MDL_MAG_SELF_TEST_MEASURES * _sc_fact * 2281.0f); // average and revert to FS = 12 Gauss
	// Turn on self-test
	turn_off_mag();
	// Enable the self-test
	uint8_t CTRL1_val = LIS3MDL_READ_REGISTER(LIS3MDL_CTRL1);
	LIS3MDL_WRITE_REGISTER(LIS3MDL_CTRL1, (CTRL1_val | 0x01));
	turn_on_mag();
	// Discard the first n measures 
	if(! discard_measures_mag(LIS3MDL_DISCARDED_MEASURES_ST, LIS3MDL_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (uint8_t ii = 0; ii < LIS3MDL_MAG_SELF_TEST_MEASURES; ii++){
		read_mag_STATUS(LIS3MDL_DISCARD_TIMEOUT);
		x_post += x;
		y_post += y;
		z_post += z;
	}
	x_post /= (LIS3MDL_MAG_SELF_TEST_MEASURES * _sc_fact * 2281.0f); // average and revert to FS = 12 Gauss
	y_post /= (LIS3MDL_MAG_SELF_TEST_MEASURES * _sc_fact * 2281.0f); // average and revert to FS = 12 Gauss
	z_post /= (LIS3MDL_MAG_SELF_TEST_MEASURES * _sc_fact * 2281.0f); // average and revert to FS = 12 Gauss
	float thrs_xy_min = 1.0f;
	float thrs_xy_max = 3.0f;
	float thrs_z_min = 0.1f;
	float thrs_z_max = 1.0f;
	if (INS_ch_st(x_pre, x_post, thrs_xy_min, thrs_xy_max) && INS_ch_st(y_pre, y_post, thrs_xy_min, thrs_xy_max) && INS_ch_st(z_pre, z_post, thrs_z_min, thrs_z_max)) {
		status = 1;
	}
	turn_off_mag();
	// Remove self test
	LIS3MDL_WRITE_REGISTER(LIS3MDL_CTRL1, CTRL1_val);
	// Reset correct FS value
	LIS3MDL_WRITE_REGISTER(LIS3MDL_CTRL2, CTRL2_old);
	turn_on_mag();
	// Discard the first n measures
	if(! discard_measures_mag(LIS3MDL_DISCARDED_MEASURES_ST, LIS3MDL_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Mag Status------------------------//
uint8_t LIS3MDL::status_mag(){
	return LIS3MDL_READ_REGISTER(LIS3MDL_STATUS);
}

//-------------------Discard measures----------------------//
uint8_t LIS3MDL::discard_measures_mag(uint8_t number_of_measures, uint32_t timeout){
  INS_discard_measures_over(number_of_measures, timeout, read_raw_mag, status_mag, (1<<7));
}

//=============================Public Members Temperature====================================//
//------------------------Read data----------------------//
uint8_t LIS3MDL::read_raw_thermo(){
	uint8_t buffer[2];
	LIS3MDL_READ_MULTIPLE_REGISTERS(buffer, 2, LIS3MDL_TEMP_OUT_L);
	int16_t temperature_tmp = (((int16_t) buffer[1] << 8) | buffer[0]);
	temperature = (float) 25.0f + temperature_tmp * 0.125f;
	return 1;
}

//------------------Read data when ready-----------------//
uint8_t LIS3MDL::read_thermo_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
    INS_read_DRDY(timeout, read_raw_thermo, _DRDY_pin);
#elif defined(INS_CHIBIOS)
    INS_read_DRDY(timeout, read_raw_thermo, _gpio_DRDY, _DRDY_pin);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LIS3MDL::read_thermo_STATUS(uint32_t timeout){
  INS_read_STATUS(timeout, read_raw_thermo, status_mag, (1 << 3));
}

//-------------------Discard measures----------------------//
uint8_t LIS3MDL::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
  INS_discard_measures_over(number_of_measures, timeout, read_raw_thermo, status_mag, (1<<7));
}
