//
//  LIS3MDL.cpp
//
//
//  Created by Andrea Vivani on 7/7/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "LIS3MDL.h"
#include "Arduino.h"
#include <SPI.h>
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
//---------------Read one register from the SPI-----------------//
uint8_t LIS3MDL::readRegister(uint8_t chipSelectPin, uint8_t thisRegister) {
  uint8_t inByte = 0;           	// incoming byte
  thisRegister |= LIS3MDL_READ;		// register in read mode
  digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  SPI.transfer(thisRegister);		// send the command to read thisRegister
  inByte = SPI.transfer(0x00);		// send 0x00 in order to read the incoming byte
  digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to select the chip
  return(inByte);			// return the read byte
}

//------------Read multiple registers from the SPI--------------//
void LIS3MDL::readMultipleRegisters(uint8_t chipSelectPin, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister) {
  startRegister |= (LIS3MDL_READ | LIS3MDL_MULT);// register in multiple read mode
  digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  SPI.transfer(startRegister);		// send the command to read thisRegister
  while (number_of_registers--){
  	*buffer++ = SPI.transfer(0x00);
  }
  digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
  return;
}

//---------------Write one register on the SPI-----------------//
void LIS3MDL::writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue) {
  digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  SPI.transfer(thisRegister); 		// send register location
  SPI.transfer(thisValue);  		// send value to record into register
  digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to select the chip
}

//-----------------Check values for self-test-------------------//
uint8_t LIS3MDL::ch_st (const float val1, const float val2, const float lim1, const float lim2){
    if (fabs(lim1) > fabs(lim2)){
        return ((fabs(val2 - val1) >= fabs(lim2)) && (fabs(val2 - val1) <= fabs(lim1)));
    }
    return ((fabs(val2 - val1) >= fabs(lim1)) && (fabs(val2 - val1) <= fabs(lim2)));
    
}

//=====================================Constructors==========================================//
LIS3MDL::LIS3MDL (uint8_t CS_pin):InertialSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = 0;
}

LIS3MDL::LIS3MDL (uint8_t CS_pin, uint8_t DRDY_pin):InertialSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = DRDY_pin;
}

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

//===================================Public Members=========================================//
//-----------------------Configuration-----------------------//
uint8_t LIS3MDL::config_mag(uint8_t range_conf, uint8_t odr_conf){
	init();
	// Trash the first reading
	readRegister(_chipSelectPin, LIS3MDL_WHO_AM_I);
	// Check if the device ID is correct
	if (readRegister(_chipSelectPin, LIS3MDL_WHO_AM_I) != LIS3MDL_ID){
		return 0;
	}
	//
	//selected range
	uint8_t CTRL2_val = range_conf & 0x60;
	writeRegister(_chipSelectPin, LIS3MDL_CTRL2, CTRL2_val);
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
	writeRegister(_chipSelectPin, LIS3MDL_CTRL5, CTRL5_val);
	//
	//Z-axis on selected performance mode, little endian
	uint8_t CTRL4_val = ((odr_conf & 0x60) >> 3);
	writeRegister(_chipSelectPin, LIS3MDL_CTRL4, CTRL4_val);
	//
	//temperature enable, selected perfomance mode, selected ODR, no self test
	uint8_t CTRL1_val = (1 << 7) | odr_conf;
	writeRegister(_chipSelectPin, LIS3MDL_CTRL1, CTRL1_val);
	//
	// clearing offset registers
	writeRegister(_chipSelectPin, LIS3MDL_OFFSET_X_REG_L, 0x00);
	writeRegister(_chipSelectPin, LIS3MDL_OFFSET_X_REG_H, 0x00);
	writeRegister(_chipSelectPin, LIS3MDL_OFFSET_Y_REG_L, 0x00);
	writeRegister(_chipSelectPin, LIS3MDL_OFFSET_Y_REG_H, 0x00);
	writeRegister(_chipSelectPin, LIS3MDL_OFFSET_Z_REG_L, 0x00);
	writeRegister(_chipSelectPin, LIS3MDL_OFFSET_Z_REG_H, 0x00);
	//
	//power on, SPI 4 wire, Continuous conversion mode
	uint8_t _CTRL3_val = 0x00;
	writeRegister(_chipSelectPin, LIS3MDL_CTRL3, _CTRL3_val);
	delay(20);
	// Discard the first n measures
	if(! discard_measures_mag(LIS3MDL_DISCARDED_MEASURES,LIS3MDL_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-------------------------Turn on---------------------------//
void LIS3MDL::turn_on_mag(){
	writeRegister(_chipSelectPin, LIS3MDL_CTRL3, _CTRL3_val);
	delay(20);
}

//------------------------Turn off---------------------------//
void LIS3MDL::turn_off_mag(){
	writeRegister(_chipSelectPin, LIS3MDL_CTRL3, (_CTRL3_val | 0x03));
}

//------------------------Read data-------------------------//
uint8_t LIS3MDL::read_raw_mag(){
	uint8_t buffer[6];
  	readMultipleRegisters(_chipSelectPin, buffer, 6, LIS3MDL_OUT_XL);
  	x = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact);
  	y = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact);
  	z = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact);
  	return 1;
}

//------------------Read data when ready--------------------//
uint8_t LIS3MDL::read_mag_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin)){
			read_raw_mag();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LIS3MDL::read_mag_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin, LIS3MDL_STATUS);
		if (STATUS_val & (1 << 3)){
			read_raw_mag();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//-----------------------Self-Test-------------------------//
uint8_t LIS3MDL::self_test_mag(){
	uint8_t status = 0;
	// Use FS = 12 Gauss
	uint8_t CTRL2_old = readRegister(_chipSelectPin, LIS3MDL_CTRL2);
	uint8_t CTRL2_val = LIS3MDL_RANGE_12 | (CTRL2_old & 0x0F);
	writeRegister(_chipSelectPin, LIS3MDL_CTRL2, CTRL2_val);
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
	uint8_t CTRL1_val = readRegister(_chipSelectPin, LIS3MDL_CTRL1);
	writeRegister(_chipSelectPin, LIS3MDL_CTRL1, (CTRL1_val | 0x01));
	turn_on_mag();
	delay(60);
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
	if (ch_st(x_pre, x_post, thrs_xy_min, thrs_xy_max) && ch_st(y_pre, y_post, thrs_xy_min, thrs_xy_max) && ch_st(z_pre, z_post, thrs_z_min, thrs_z_max)) {
		status = 1;
	}
	/*
	// Define Threshold based on datasheet (quite high...it could be between 1 and 3 (x and y) and between 0.1 and 1 (z))
	float thrs_xy = 2;
	float thrs_z = 0.5;
	// Check if values are bigger than the threshold
	if ((fabs(x) > thrs_xy) && (fabs(y) > thrs_xy) && (fabs(z) > thrs_z)){
		status = 1;
	}*/
	turn_off_mag();
	// Remove self test
	writeRegister(_chipSelectPin, LIS3MDL_CTRL1, CTRL1_val);
	// Reset correct FS value
	writeRegister(_chipSelectPin, LIS3MDL_CTRL2, CTRL2_old);
	turn_on_mag();
	delay(60);
	// Discard the first n measures
	if(! discard_measures_mag(LIS3MDL_DISCARDED_MEASURES_ST, LIS3MDL_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Mag Status------------------------//
uint8_t LIS3MDL::status_mag(){
	return readRegister(_chipSelectPin, LIS3MDL_STATUS);
}

//-------------------Discard measures----------------------//
uint8_t LIS3MDL::discard_measures_mag(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	while (count < (number_of_measures * 0.5)){
		uint8_t STATUS_value = status_mag();
		if (STATUS_value & (1 << 7)){
			read_raw_mag();
			now = micros();
			count++;
		}
		if ((micros() - now) > timeout){
			return 0;
		}
		else if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 1;
}

//=============================Public Members Temperature====================================//
//------------------------Read data----------------------//
uint8_t LIS3MDL::read_raw_thermo(){
	uint8_t buffer[2];
	readMultipleRegisters(_chipSelectPin, buffer, 2, LIS3MDL_TEMP_OUT_L);
	int16_t temperature_tmp = (((int16_t) buffer[1] << 8) | buffer[0]);
	temperature = (float) 25.0 + temperature_tmp * 0.125;
	return 1;
}

//------------------Read data when ready-----------------//
uint8_t LIS3MDL::read_thermo_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin)){
			read_raw_thermo();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 0;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LIS3MDL::read_thermo_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = status_mag();
		if (STATUS_val & (1 << 3)){
			read_raw_thermo();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//-------------------Discard measures----------------------//
uint8_t LIS3MDL::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	while (count < (number_of_measures * 0.5)){
		uint8_t STATUS_value = status_mag();
		if (STATUS_value & (1 << 7)){
			read_raw_thermo();
			now = micros();
			count++;
		}
		if ((micros() - now) > timeout){
			return 0;
		}
		else if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 1;
}
