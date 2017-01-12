//
// ADXL345.cpp
//
//
// Created by Andrea Vivani on 22/8/15.
// Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "ADXL345.h"
#include "INS_AuxFun.h"
#ifdef INS_ARDUINO
#include "Arduino.h"
#include <SPI.h>
#endif
//====================================Registers Addresses=========================================// 
#define ADXL345_DEVID 			0x00 
#define ADXL345_THRESH_TAP 		0x1D 
#define ADXL345_OFSX 			0x1E 
#define ADXL345_OFSY 			0x1F 
#define ADXL345_OFSZ 			0x20 
#define ADXL345_DUR 			0x21 
#define ADXL345_LATENT 			0x22 
#define ADXL345_WINDOW 			0x23 
#define ADXL345_THRESH_ACT 		0x24 
#define ADXL345_THRESH_INACT 	0x25 
#define ADXL345_TIME_INACT 		0x26 
#define ADXL345_ACT_INACT_CTL 	0x27 
#define ADXL345_THRESH_FF 		0x28 
#define ADXL345_TIME_FF 		0x29 
#define ADXL345_TAP_AXES 		0x2A 
#define ADXL345_ACT_TAP_STATUS 	0x2B 
#define ADXL345_BW_RATE 		0x2C 
#define ADXL345_POWER_CTL 		0x2D 
#define ADXL345_INT_ENABLE 		0x2E 
#define ADXL345_INT_MAP 		0x2F 
#define ADXL345_INT_SOURCE 		0x30 
#define ADXL345_DATA_FORMAT 	0x31 
#define ADXL345_DATAX0 			0x32 
#define ADXL345_DATAX1 			0x33 
#define ADXL345_DATAY0 			0x34 
#define ADXL345_DATAY1 			0x35 
#define ADXL345_DATAZ0 			0x36 
#define ADXL345_DATAZ1 			0x37 
#define ADXL345_FIFO_CTL 		0x38 
#define ADXL345_FIFO_STATUS 	0x39 
//=======================================Constants=============================================// 
#define ADXL345_ID				0xE5
#define ADXL345_READ			0x80
#define ADXL345_MULT			0x40
//==================================Auxiliary Functions========================================//
#ifdef INS_ARDUINO
  #define ADXL345_READ_REGISTER(reg) INS_SPI_readRegister(_chipSelectPin, reg, ADXL345_READ)
  #define ADXL345_READ_MULTIPLE_REGISTERS(buf, num, start) INS_SPI_readMultipleRegisters(_chipSelectPin, buf, num, startRegister, (ADXL345_READ | ADXL345_MULT))
  #define ADXL345_WRITE_REGISTER(reg, val) INS_SPI_writeRegister(_chipSelectPin, reg, val, 0x00)
#elif defined(INS_CHIBIOS)
  #define ADXL345_READ_REGISTER(reg) INS_SPI_readRegister(_SPI_int, _spicfg, reg, ADXL345_READ)
  #define ADXL345_READ_MULTIPLE_REGISTERS(buf, num, start) INS_SPI_readMultipleRegisters(_SPI_int, _spicfg, buf, num, start, (ADXL345_READ | ADXL345_MULT))
  #define ADXL345_WRITE_REGISTER(reg, val) INS_SPI_writeRegister(_SPI_int, _spicfg, reg, val, 0x00)
#endif
//=====================================Constructors==========================================//
#ifdef INS_ARDUINO
ADXL345::ADXL345 (uint8_t CS_pin):InertialSensor(), AccelerometerSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = 0;
}

ADXL345::ADXL345 (uint8_t CS_pin, uint8_t DRDY_pin):InertialSensor(), AccelerometerSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = DRDY_pin;
}

//-----------------------Initialization-----------------------//
void ADXL345::init(){
	pinMode(_chipSelectPin, OUTPUT);
	digitalWrite(_chipSelectPin, HIGH);
	if (_DRDY_pin != 0){
		pinMode(_DRDY_pin, INPUT);
	}
	// initialize variables
	x = 0;
	y = 0;
	z = 0;
}
#elif defined(INS_CHIBIOS)
ADXL345::ADXL345 (SPIDriver* SPI, SPIConfig* spicfg):InertialSensor(), AccelerometerSensor(){
	_SPI_int = SPI;
	_spicfg = spicfg;
	_DRDY_pin = 0;
    init();
}

ADXL345::ADXL345 (SPIDriver* SPI, SPIConfig* spicfg, ioportid_t gpio_DRDY, uint8_t DRDY_pin):InertialSensor(), AccelerometerSensor(){
	_SPI_int = SPI;
	_spicfg = spicfg;
	_gpio_DRDY = gpio_DRDY;
	_DRDY_pin = DRDY_pin;
	init();
}

//-----------------------Initialization-----------------------//
void ADXL345::init(){
	palSetPad(_spicfg->ssport, _spicfg->sspad);
	palSetPadMode(_spicfg->ssport, _spicfg->sspad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	if (_DRDY_pin != 0){
		palSetPadMode(_gpio_DRDY, _DRDY_pin, PAL_MODE_INPUT);
	}
	// initialize variables
	x = 0;
	y = 0;
	z = 0;
}
#endif
//=============================Public Members Accelerometer====================================//
//-----------------------Configuration-----------------------//
uint8_t ADXL345::config_accel(uint8_t accel_range, uint8_t accel_odr){
	init();
	//
	// Set scale factor
	_sc_fact = INS_G_VAL / 256.0f;
	//
	// Trash the first reading
	ADXL345_READ_REGISTER(ADXL345_DEVID);
	//
	// Check if the device ID is correct
	if (ADXL345_READ_REGISTER(ADXL345_DEVID) != ADXL345_ID){
		return 0;
	}
	//
	// Selected bandwidth
	uint8_t BW_RATE_val = accel_odr;
	ADXL345_WRITE_REGISTER(ADXL345_BW_RATE, BW_RATE_val);
	//
	// Enable DRDY and Overrun interrupts
	uint8_t INT_ENABLE_val = (1 << 7) | 0x01;
	ADXL345_WRITE_REGISTER(ADXL345_INT_ENABLE, INT_ENABLE_val);
	//
	// DRDY on INT1 pin, Overrun on INT2 pin
	uint8_t INT_MAP_val = 0x7F;
	ADXL345_WRITE_REGISTER(ADXL345_INT_MAP, INT_MAP_val);
	//
	// Reset offsets
	ADXL345_WRITE_REGISTER(ADXL345_OFSX, 0x00);
	ADXL345_WRITE_REGISTER(ADXL345_OFSY, 0x00);
	ADXL345_WRITE_REGISTER(ADXL345_OFSZ, 0x00);
	//
	// No self-test, 4-wires SPI, selected range 
	uint8_t DATA_FORMAT_val = (1 << 3) | accel_range;
	ADXL345_WRITE_REGISTER(ADXL345_DATA_FORMAT, DATA_FORMAT_val);
	//
	// Turn on the accel 
	_POWER_CTL_val = (1 << 3);
	turn_on_accel();
	//
	// Discard the first n accel measures
	if(! discard_measures_accel(ADXL345_DISCARDED_MEASURES, ADXL345_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//----------------Turn on accelerometer----------------//
void ADXL345::turn_on_accel(){
	ADXL345_WRITE_REGISTER(ADXL345_POWER_CTL, _POWER_CTL_val);
#ifdef INS_ARDUINO
	delay(200);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(200);
#endif
}

//----------------Turn off accelerometer---------------//
void ADXL345::turn_off_accel(){
	uint8_t POWER_CTL_val = ADXL345_READ_REGISTER(ADXL345_POWER_CTL);
	ADXL345_WRITE_REGISTER(ADXL345_POWER_CTL, (POWER_CTL_val & 0xF7));
}

//-----------------Read accelerometer------------------//
uint8_t ADXL345::read_raw_accel(){
	uint8_t buffer[6];
	ADXL345_READ_MULTIPLE_REGISTERS(buffer, 6, ADXL345_DATAX0);
	x = (float) ((int16_t) ((buffer[1] << 8) | buffer[0]) * _sc_fact);
	y = (float) ((int16_t) ((buffer[3] << 8) | buffer[2]) * _sc_fact);
	z = (float) ((int16_t) ((buffer[5] << 8) | buffer[4]) * _sc_fact);
	return 1;
}

//------------Read accelerometer when ready--------------//
uint8_t ADXL345::read_accel_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
    INS_read_DRDY(timeout, read_raw_accel, _DRDY_pin);
#elif defined(INS_CHIBIOS)
    INS_read_DRDY(timeout, read_raw_accel, _gpio_DRDY, _DRDY_pin);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t ADXL345::read_accel_STATUS(uint32_t timeout){
    INS_read_STATUS(timeout, read_raw_accel, status_accel, 0x80);
}

//--------------------Check biases------------------------//
uint8_t ADXL345::check_accel_biases(float bx, float by, float bz){
	float thrs = 150e-3f * INS_G_VAL; //typical 40mg zero-G level
	if ((fabs(bx) > thrs) || (fabs(by) > thrs) || (fabs(bz) > thrs)){
		return 0;
	}
	return 1;
}

//------------Self-test the accelerometer---------------//
uint8_t ADXL345::self_test_accel(){
	uint8_t status = 0;
	// Discard the first n measures
	if(!discard_measures_accel(ADXL345_DISCARDED_MEASURES_ST, ADXL345_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (uint8_t ii = 0; ii < ADXL345_ACCEL_SELF_TEST_MEASURES; ii++){
		read_accel_STATUS(ADXL345_DISCARD_TIMEOUT);
		x_pre += x;
		y_pre += y;
		z_pre += z;
	}
	x_pre /= ADXL345_ACCEL_SELF_TEST_MEASURES;
	y_pre /= ADXL345_ACCEL_SELF_TEST_MEASURES;
	z_pre /= ADXL345_ACCEL_SELF_TEST_MEASURES;
	// Turn off the sensor and setup self-test
	turn_off_accel();
	uint8_t DATA_FORMAT_val = ADXL345_READ_REGISTER(ADXL345_DATA_FORMAT);
	ADXL345_WRITE_REGISTER(ADXL345_DATA_FORMAT, (DATA_FORMAT_val | (1 << 7)));
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(ADXL345_DISCARDED_MEASURES_ST, ADXL345_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (uint8_t ii = 0; ii < ADXL345_ACCEL_SELF_TEST_MEASURES; ii++){
		read_accel_STATUS(ADXL345_DISCARD_TIMEOUT);
		x_post += x;
		y_post += y;
		z_post += z;
	}
	x_post /= ADXL345_ACCEL_SELF_TEST_MEASURES;
	y_post /= ADXL345_ACCEL_SELF_TEST_MEASURES;
	z_post /= ADXL345_ACCEL_SELF_TEST_MEASURES;
	// Check if values are bigger than the threshold
	if (INS_ch_st(x_pre, x_post, 10.4f, 17.4f) && INS_ch_st(y_pre, y_post, 10.4f, 17.4f) && INS_ch_st(z_pre, z_post, 13.5f, 19.7f)) {
		status = 1;
	}
#ifdef INS_ARDUINO
	else {
		Serial.println(x_pre);
		Serial.println(x_post);
		Serial.println(y_pre);
		Serial.println(y_post);
		Serial.println(z_pre);
		Serial.println(z_post);
	}
#endif
	turn_off_accel();
	ADXL345_WRITE_REGISTER(ADXL345_DATA_FORMAT, DATA_FORMAT_val);
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(ADXL345_DISCARDED_MEASURES_ST, ADXL345_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Accel Status------------------------//
uint8_t ADXL345::status_accel(){
	return ADXL345_READ_REGISTER(ADXL345_INT_SOURCE);
}

//-------------------Discard measures----------------------//
uint8_t ADXL345::discard_measures_accel(uint8_t number_of_measures, uint32_t timeout){
  INS_discard_measures(number_of_measures, timeout, read_accel_STATUS);
}
