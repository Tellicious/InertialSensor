//
//  L3GD20H.cpp
//
//
//  Created by Andrea Vivani on 23/2/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "L3GD20H.h"
#include "Arduino.h"
#include <SPI.h>

//====================================Registers Addresses=========================================// 
#define L3GD20H_WHO_AM_I	0x0F
#define L3GD20H_CTRL1		0x20
#define L3GD20H_CTRL2		0x21
#define L3GD20H_CTRL3		0x22
#define L3GD20H_CTRL4		0x23
#define L3GD20H_CTRL5		0x24
#define L3GD20H_REFERENCE	0x25
#define L3GD20H_OUT_TEMP	0x26
#define L3GD20H_STATUS		0x27
#define L3GD20H_OUT_X_L		0x28
#define L3GD20H_OUT_X_H		0x29
#define L3GD20H_OUT_Y_L		0x2A
#define L3GD20H_OUT_Y_H		0x2B
#define L3GD20H_OUT_Z_L		0x2C
#define L3GD20H_OUT_Z_H		0x2D
#define L3GD20H_FIFO_CTRL	0x2E
#define L3GD20H_FIFO_SRC	0x2F
#define L3GD20H_IG_CFG		0x30
#define L3GD20H_IG_SRC		0x31
#define L3GD20H_IG_THS_XH	0x32
#define L3GD20H_IG_THS_XL	0x33
#define L3GD20H_IG_THS_YH	0x34
#define L3GD20H_IG_THS_YL	0x35
#define L3GD20H_IG_THS_ZH	0x36
#define L3GD20H_IG_THS_ZL	0x37
#define L3GD20H_IG_DURATION	0x38
#define L3GD20H_LOW_ODR		0x39
//=======================================Constants=============================================// 
#define L3GD20H_ID			0xD7
#define L3GD20H_READ		0x80
#define L3GD20H_MULT		0x40

//==================================Auxiliary Functions========================================//
//---------------Read one register from the SPI-----------------//
uint8_t L3GD20H::readRegister(uint8_t chipSelectPin, uint8_t thisRegister) {
  uint8_t inByte = 0;           	// incoming byte
  thisRegister |= L3GD20H_READ;		// register in read mode
  digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  SPI.transfer(thisRegister);		// send the command to read thisRegister
  inByte = SPI.transfer(0x00);		// send 0x00 in order to read the incoming byte
  digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to select the chip
  return(inByte);			// return the read byte
}

//------------Read multiple registers from the SPI--------------//
void L3GD20H::readMultipleRegisters(uint8_t chipSelectPin, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister) {
  startRegister |= (L3GD20H_READ | L3GD20H_MULT);// register in multiple read mode
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  	SPI.transfer(startRegister);		// send the command to read thisRegister
  	while (number_of_registers--){
  		*buffer++ = SPI.transfer(0x00);
  	}
  	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
  	return;
}

//---------------Write one register on the SPI-----------------//
void L3GD20H::writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue) {
  digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  SPI.transfer(thisRegister); 		// send register location
  SPI.transfer(thisValue);  		// send value to record into register
  digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to select the chip
}

//-----------------Check values for self-test-------------------//
uint8_t L3GD20H::ch_st (const double val1, const double val2, const double lim1, const double lim2){
    if (fabs(lim1) > fabs(lim2)){
        return ((fabs(val2 - val1) >= fabs(lim2)) && (fabs(val2 - val1) <= fabs(lim1)));
    }
    return ((fabs(val2 - val1) >= fabs(lim1)) && (fabs(val2 - val1) <= fabs(lim2)));
    
}
//=====================================Constructors==========================================//
L3GD20H::L3GD20H (uint8_t CS_pin):InertialSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = 0;
}

L3GD20H::L3GD20H (uint8_t CS_pin, uint8_t DRDY_pin):InertialSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = DRDY_pin;
}

void L3GD20H::init(){
	pinMode(_chipSelectPin,OUTPUT);
	digitalWrite(_chipSelectPin, HIGH);
	if (_DRDY_pin != 0){
		pinMode(_DRDY_pin, INPUT);
	}
	x = 0;
	y = 0;
	z = 0;
}

//===================================Public Members=========================================//
//-----------------------Configuration-----------------------//
uint8_t L3GD20H::config_gyro(uint8_t range_conf, uint8_t odr_conf, uint8_t LPF2_enable, uint8_t HP_enable, uint8_t HP_freq){
	init();
	// Trash the first reading
	readRegister(_chipSelectPin, L3GD20H_WHO_AM_I);
	// Check if the device ID is correct
	if (readRegister(_chipSelectPin, L3GD20H_WHO_AM_I) != L3GD20H_ID){
		return 0;
	}
	//DRDY/INT2 active high, SPI only, selected ODR
	uint8_t LOW_ODR_val = (0 << 5) | (1 << 3) | (0 << 2) | ((odr_conf & 0x10) >> 4); 
	writeRegister(_chipSelectPin, L3GD20H_LOW_ODR,LOW_ODR_val);
	//
        uint8_t CTRL2_val;
	if (HP_enable){
		//no DEN interrupt, set High-pass filter frequency
		CTRL2_val = HP_freq;
	}
	else{
		//no DEN interrupt, no High-pass filter
		CTRL2_val = 0x00;
	}
	writeRegister(_chipSelectPin, L3GD20H_CTRL2,CTRL2_val);
	//
	//Data ready on INT2
	uint8_t CTRL3_val = (1 << 3);
	writeRegister(_chipSelectPin, L3GD20H_CTRL3,CTRL3_val);
	//
	//continuous update, Little endian, selected range, level sensitive latched disabled, normal self-test, SPI 4-wire
	uint8_t CTRL4_val = (0 << 7) | range_conf; 
	writeRegister(_chipSelectPin, L3GD20H_CTRL4,CTRL4_val);
	//
	//FIFO control on Bypass Mode
	uint8_t FIFO_CTRL_val = 0x00;
	writeRegister(_chipSelectPin, L3GD20H_FIFO_CTRL,FIFO_CTRL_val);
	//
	//Enable FIFO, set HP filter, set LPF2
	uint8_t CTRL5_val = (1 << 6);
	if (HP_enable){
		CTRL5_val |= (1 << 4);
		CTRL5_val |= (0x5);
	}
	if (LPF2_enable){ 
		CTRL5_val |= (0xF);
	}
	writeRegister(_chipSelectPin, L3GD20H_CTRL5,CTRL5_val);
	//
	switch (range_conf){
		case (L3GD20H_RANGE_245):
			_sc_fact = 8.75e-3 * INS_TORAD;
			break;
		case (L3GD20H_RANGE_500):
			_sc_fact = 17.5e-3 * INS_TORAD;
			break;
		case (L3GD20H_RANGE_2000):
			_sc_fact = 70e-3 * INS_TORAD;
			break;
		default:
			return 2;
	}
	//
	//selected ODR, power on, 3-axis enabled
	_CTRL1_val = ((odr_conf & 0xF) << 4) | (1 << 3) | 0x7;
	writeRegister(_chipSelectPin, L3GD20H_CTRL1,_CTRL1_val);
	delay(100);
	// Discard the first n measures
	if(! discard_measures_gyro(L3GD20H_DISCARDED_MEASURES,L3GD20H_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-------------------------Turn on---------------------------//
void L3GD20H::turn_on_gyro(){
	writeRegister(_chipSelectPin, L3GD20H_CTRL1,_CTRL1_val);
	delay(100);
}

//------------------------Turn off---------------------------//
void L3GD20H::turn_off_gyro(){
	writeRegister(_chipSelectPin, L3GD20H_CTRL1,(_CTRL1_val & 0xF7));
}

//-------------------------Sleep-----------------------------//
void L3GD20H::sleep_gyro(){
	writeRegister(_chipSelectPin, L3GD20H_CTRL1,(_CTRL1_val & 0xF8));
}

//------------------------Read data-------------------------//
uint8_t L3GD20H::read_raw_gyro(){
	uint8_t buffer[6];
  	readMultipleRegisters(_chipSelectPin, buffer, 6, L3GD20H_OUT_X_L);
  	x = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact);
  	y = (float)	(((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact);
  	z = (float)	(((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact);
  	return 1;
}

//------------------Read data when ready--------------------//
uint8_t L3GD20H::read_gyro_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin)){
			read_raw_gyro();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t L3GD20H::read_gyro_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin, L3GD20H_STATUS);
		if (STATUS_val & (1 << 3)){
			read_raw_gyro();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//--------------------Check biases------------------------//
uint8_t L3GD20H::check_gyro_biases(float bx, float by, float bz){
// Define Threshold based on Full-Scale value
	float thrs;
	if ((_sc_fact - 8.75e-3 * INS_TORAD) < 1e-5){
		thrs = 10 * INS_TORAD * 1.2;	//10 dps, according to datasheet
	}
	else if ((_sc_fact - 17.5e-3 * INS_TORAD) < 1e-5){
		thrs = 15 * INS_TORAD * 1.2;	//15 dps, according to datasheet
	}
	else if ((_sc_fact - 70e-3 * INS_TORAD) < 1e-5){
		thrs = 25 * INS_TORAD * 1.2;	//25 dps, according to datasheet
	}
	else {
		return 0;
	}
	if ((fabs(bx) > thrs) || (fabs(by) > thrs) || (fabs(bz) > thrs)){
		return 0;
	}
	return 1;
}

//---------------------High-Pass Reset-----------------------//
void L3GD20H::HP_reset_gyro(){
	readRegister(_chipSelectPin, L3GD20H_REFERENCE);
}

//-----------------------Self-Test-------------------------//
uint8_t L3GD20H::self_test_gyro(uint8_t mode){
	uint8_t status = 0;
	// Discard the first n measures
	if(!discard_measures_gyro(L3GD20H_DISCARDED_MEASURES_ST, L3GD20H_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (int ii = 0; ii < L3GD20H_GYRO_SELF_TEST_MEASURES; ii++){
		read_gyro_STATUS(L3GD20H_DISCARD_TIMEOUT);
		x_pre += x;
		y_pre += y;
		z_pre += z;
	}
	x_pre /= L3GD20H_GYRO_SELF_TEST_MEASURES;
	y_pre /= L3GD20H_GYRO_SELF_TEST_MEASURES;
	z_pre /= L3GD20H_GYRO_SELF_TEST_MEASURES;
	// Turn on self-test
	turn_off_gyro();
	uint8_t CTRL4_val = readRegister(_chipSelectPin, L3GD20H_CTRL4);
	if (!mode){
		CTRL4_val |= (1 << 1); // Self-test mode 0
	}
	else {
		CTRL4_val |= ((1 << 1) | (1 << 2)); // Self-test mode 1
	}
	writeRegister(_chipSelectPin, L3GD20H_CTRL4, CTRL4_val);
	turn_on_gyro();
	// Discard the first n measures 
	if(!discard_measures_gyro(L3GD20H_DISCARDED_MEASURES_ST, L3GD20H_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (int ii = 0; ii < L3GD20H_GYRO_SELF_TEST_MEASURES; ii++){
		read_gyro_STATUS(L3GD20H_DISCARD_TIMEOUT);
		x_post += x;
		y_post += y;
		z_post += z;
	}
	x_post /= L3GD20H_GYRO_SELF_TEST_MEASURES;
	y_post /= L3GD20H_GYRO_SELF_TEST_MEASURES;
	z_post /= L3GD20H_GYRO_SELF_TEST_MEASURES;
	// Define Threshold based on the Full-Scale value
	float thrs;
	if ((_sc_fact - 8.75e-3 * INS_TORAD) < 1e-5){
		thrs = 50 * INS_TORAD;
	}
	else if ((_sc_fact - 17.5e-3 * INS_TORAD) < 1e-5){
		thrs = 130 * INS_TORAD;
	}
	else if ((_sc_fact - 70e-3 * INS_TORAD) < 1e-5){
		thrs = 250 * INS_TORAD;
	}
	else {
		return 0;
	}
	// Check if values are bigger than the threshold
	if (ch_st(x_pre, x_post, (0.6 * thrs), (1.4 * thrs)) && ch_st(y_pre, y_post, (0.6 * thrs), (1.4 * thrs)) && ch_st(z_pre, z_post, (0.6 * thrs), (1.4 * thrs))) {
		status = 1;
	}
	turn_off_gyro();
	CTRL4_val &= 0xF9; // Remove Self-Test
	writeRegister(_chipSelectPin, L3GD20H_CTRL4, CTRL4_val);
	turn_on_gyro();
	// Discard the first n measures
	if(!discard_measures_gyro(L3GD20H_DISCARDED_MEASURES_ST, L3GD20H_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Gyro Status------------------------//
uint8_t L3GD20H::status_gyro(){
	return readRegister(_chipSelectPin, L3GD20H_STATUS);
}

//-------------------Discard measures----------------------//
uint8_t L3GD20H::discard_measures_gyro(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	while (count < (number_of_measures * 0.5)){
		uint8_t STATUS_value = status_gyro();
		if (STATUS_value & (1 << 7)){
			read_raw_gyro();
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
