//
// ADXL345.cpp
//
//
// Created by Andrea Vivani on 22/8/15.
// Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "ADXL345.h"
#include "Arduino.h"
#include <SPI.h>

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
//---------------Read one register from the SPI-----------------//
uint8_t ADXL345::readRegister(uint8_t chipSelectPin, uint8_t thisRegister) {
	uint8_t inByte = 0;   	// incoming byte
	thisRegister |= ADXL345_READ;		// register in read mode
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(thisRegister);		// send the command to read thisRegister
	inByte = SPI.transfer(0x00);		// send 0x00 in order to read the incoming byte
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	return(inByte);			// return the read byte
}

//------------Read multiple registers from the SPI--------------//
void ADXL345::readMultipleRegisters(uint8_t chipSelectPin, uint8_t * buffer, uint8_t number_of_registers, uint8_t startRegister) {
	startRegister |= (ADXL345_READ | ADXL345_MULT);// register in multiple read mode
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(startRegister);		// send the command to read thisRegister
	while(number_of_registers--){
		*buffer++ = SPI.transfer(0x00);
	}
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	return;
}

//---------------Write one register on the SPI-----------------//
void ADXL345::writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue) {
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(thisRegister); 		// send register location
	SPI.transfer(thisValue); 		// send value to record into register
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	return;
}

//-----------------Check values for self-test-------------------//
uint8_t ADXL345::ch_st (const double val1, const double val2, const double lim1, const double lim2){
    if (fabs(lim1) > fabs(lim2)){
        return ((fabs(val2 - val1) >= fabs(lim2)) && (fabs(val2 - val1) <= fabs(lim1)));
    }
    return ((fabs(val2 - val1) >= fabs(lim1)) && (fabs(val2 - val1) <= fabs(lim2)));
    
}

//=====================================Constructors==========================================//
ADXL345::ADXL345 (uint8_t CS_pin):InertialSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = 0;
}

ADXL345::ADXL345 (uint8_t CS_pin, uint8_t DRDY_pin):InertialSensor(){
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

//=============================Public Members Accelerometer====================================//
//-----------------------Configuration-----------------------//
uint8_t ADXL345::uint8_t config_accel(uint8_t accel_range, uint8_t accel_odr){
	init();
	//
	// Trash the first reading
	readRegister(_chipSelectPin, ADXL345_DEVID);
	//
	// Check if the device ID is correct
	if (readRegister(_chipSelectPin, ADXL345_DEVID) != ADXL345_ID){
		delay(200);
		return 0;
	}
	//
	// Selected bandwidth
	uint8_t BW_RATE_val = accel_odr;
	writeRegister(_chipSelectPin, ADXL345_BW_RATE, BW_RATE_val);
	//
	// Enable DRDY and Overrun interrupts
	uint8_t INT_ENABLE_val = (1 << 7) | 0x01;
	writeRegister(_chipSelectPin, ADXL345_INT_ENABLE, INT_ENABLE_val);
	//
	// DRDY on INT1 pin, Overrun on INT2 pin
	uint8_t INT_MAP_val = 0x7F;
	writeRegister(_chipSelectPin, ADXL345_INT_MAP, INT_MAP_val);
	//
	// No self-test, 4-wires SPI, selected range 
	uint8_t DATA_FORMAT_val = (1 << 3) | accel_range;
	writeRegister(_chipSelectPin, ADXL345_DATA_FORMAT, DATA_FORMAT_val);
	//
	// Turn on the accel 
	uint8_t POWER_CTL_val = (1 << 3);
	writeRegister(_chipSelectPin, ADXL345_POWER_CTL, POWER_CTL_val);
	//
	// Discard the first n accel measures
	if(! discard_measures_accel(ADXL345_DISCARDED_MEASURES, ADXL345_DISCARD_TIMEOUT)){
		delay(200);
		return 0;
	}
	return 1;
}

//----------------Turn on accelerometer----------------//
void ADXL345::turn_on_accel(){
	writeRegister(_chipSelectPin, ADXL345_CTRL1_XL, _POWER_CTL_val);
	delay(200);
}

//----------------Turn off accelerometer---------------//
void ADXL345::turn_off_accel(){
	uint8_t POWER_CTL_val = readRegister(_chipSelectPin, ADXL345_POWER_CTL);
	writeRegister(_chipSelectPin, ADXL345_POWER_CTL, (POWER_CTL_val & 0xF7));
}

//-----------------Read accelerometer------------------//
uint8_t ADXL345::read_raw_accel(){
	uint8_t buffer[6];
	readMultipleRegisters(_chipSelectPin, buffer, 6, ADXL345_DATAX0);
	ax = (float) (((int16_t) (buffer[0] << 8) | buffer[1]) * _sc_fact);
	ay = (float) (((int16_t) (buffer[2] << 8) | buffer[3]) * _sc_fact);
	az = (float) (((int16_t) (buffer[4] << 8) | buffer[5]) * _sc_fact);
	return 1;
}

//------------Read accelerometer when ready--------------//
uint8_t ADXL345::read_accel_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin) == 1){
			read_raw_accel();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 0;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t ADXL345::read_accel_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin, ADXL345_INT_SOURCE);
		if ((STATUS_val & 0x80) == 0x80){
			read_raw_accel();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
	delay(200);
return 0;
}

//--------------------Check biases------------------------//
uint8_t ADXL345::check_accel_biases(float bx, float by, float bz){
	float thrs = 150e-3 * INS_G_VAL; //typical 40mg zero-G level
	if ((fabs(bx) > thrs) || (fabs(by) > thrs) || (fabs(bz) > thrs)){
		return 0;
	}
	return 1;
}

//------------Self-test the accelerometer---------------//
uint8_t ADXL345::self_test_accel(){
	uint8_t status = 0;
	// Discard the first n measures
	if(! discard_measures_accel(ADXL345_DISCARDED_MEASURES_ST, ADXL345_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (int ii = 0; ii < ADXL345_ACCEL_SELF_TEST_MEASURES; ii++){
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
	uint8_t DATA_FORMAT_val = readRegister(_chipSelectPin, ADXL345_DATA_FORMAT);
	writeRegister(_chipSelectPin, ADXL345_DATA_FORMAT, (DATA_FORMAT_val | (1 << 7)));
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(ADXL345_DISCARDED_MEASURES_ST,ADXL345_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (int ii = 0; ii < ADXL345_ACCEL_SELF_TEST_MEASURES; ii++){
		read_accel_STATUS(ADXL345_DISCARD_TIMEOUT);
		x_post += x;
		y_post += y;
		z_post += z;
	}
	x_post /= ADXL345_ACCEL_SELF_TEST_MEASURES;
	y_post /= ADXL345_ACCEL_SELF_TEST_MEASURES;
	z_post /= ADXL345_ACCEL_SELF_TEST_MEASURES;
	// Check if values are bigger than the threshold
	if (ch_st(x_pre, x_post, 0.7, 1.0) && ch_st(y_pre, y_post, 1.5, 1.9) && ch_st(z_pre, z_post, 1.4, 2.0)) {
		status = 1;
	}
	turn_off_accel();
	writeRegister(_chipSelectPin, ADXL345_DATA_FORMAT, DATA_FORMAT_val);
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(ADXL345_DISCARDED_MEASURES_ST, ADXL345_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Accel Status------------------------//
uint8_t ADXL345::status_accel(){
	return readRegister(_chipSelectPin, ADXL345_INT_SOURCE);
}

//-------------------Discard measures----------------------//
uint8_t ADXL345::discard_measures_accel(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	while (count < (number_of_measures)){
		if(! read_accel_STATUS(timeout)){
			return 0;
		}
		count++;
	}
	return 1;
}
