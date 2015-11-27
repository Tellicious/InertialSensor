//
//  BMP180.cpp
//
//
//  Created by Andrea Vivani on 08/11/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "BMP180.h"
#include "Arduino.h"
#include <Wire.h>
//========================================Chip Address============================================//
 #define BMP180_ADDR			0x77
//====================================Registers Addresses=========================================// 
#define BMP180_CAL_AC1			0xAA
#define BMP180_CAL_AC2			0xAC
#define BMP180_CAL_AC3			0xAE
#define BMP180_CAL_AC4			0xB0
#define BMP180_CAL_AC5			0xB2
#define BMP180_CAL_AC6			0xB4
#define BMP180_CAL_B1			0xB6
#define BMP180_CAL_B2			0xB8
#define BMP180_CAL_MB			0xBA
#define BMP180_CAL_MC			0xBC
#define BMP180_CAL_MD			0xBE
#define BMP180_CHIP_ID			0xD0
#define BMP180_RST_REG 			0xE0
#define BMP180_CTRL_MEAS		0xF4
#define BMP180_OUT_MSB			0xF6
#define BMP180_OUT_LSB			0xF7
#define BMP180_OUT_XLSB			0xF8
//=======================================Commands=============================================// 
#define BMP180_READ_TEMP		0x2E
#define BMP180_READ_PRESS		0x34
//=======================================Constants=============================================// 
#define BMP180_ID				0x55

//==================================Auxiliary Functions========================================//
//---------------Read one register from the I2C-----------------//
uint8_t BMP180::readRegister(uint8_t thisRegister) {
  	uint8_t inByte = 0;    	// incoming byte
  	Wire.beginTransmission(BMP180_ADDR); //start transmission to the device
	Wire.write(thisRegister); //register address to read from
	Wire.endTransmission();	//end transmission
  	Wire.requestFrom(BMP180_ADDR, 1);	//request 1 byte
  	inByte = Wire.read();		//read the incoming byte
  	Wire.endTransmission();
  	return(inByte);			// return the read byte
}

//------------Read multiple registers from the I2C--------------//
void BMP180::readMultipleRegisters(uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister) {
  	Wire.beginTransmission(BMP180_ADDR); //start transmission to the device
	Wire.write(startRegister); //register address to read from
	Wire.endTransmission();	//end transmission
  	Wire.requestFrom((uint8_t) BMP180_ADDR, number_of_registers);	//request n byte
  	while(number_of_registers--){
  		*buffer++ = Wire.read();		//read the incoming bytes
  	}
	Wire.endTransmission();	//end transmission
  	return;
}

//---------------Write one register on the I2C-----------------//
uint8_t BMP180::writeRegister(uint8_t thisRegister, const uint8_t thisValue) {
  	Wire.beginTransmission(BMP180_ADDR); //start transmission to the device
	Wire.write(thisRegister); //register address where to write
	Wire.write(thisValue); //write register value
	return (! Wire.endTransmission());	//end transmission
}

//---------------Compute calibration coeff. B5-----------------//
int32_t BMP180::computeB5(int32_t UT) {
	int32_t X1 = (UT - (int32_t) _bmp180_calib.AC6v) * ((int32_t) _bmp180_calib.AC5v) >> 15;
	int32_t X2 = ((int32_t) _bmp180_calib.MCv << 11) / (X1 + (int32_t) _bmp180_calib.MDv);
	return X1 + X2;
}

//=====================================Constructors==========================================//
BMP180::BMP180 ():InertialSensor(){
	return;
}

void BMP180::init(){
	press = 0;
	temperature = 0;
}

//===================================Public Members=========================================//
//-----------------------Configuration-----------------------//
uint8_t BMP180::config_baro(uint8_t oversamp){
	init();
	// Trash the first reading
	readRegister(BMP180_CHIP_ID);
	// Check if the device ID is correct
	if (readRegister(BMP180_CHIP_ID)!= BMP180_ID){
		return 0;
	}
	switch(oversamp){
		case BMP180_OS_1:
			_bmp180Mode = 0;
			break;
		case BMP180_OS_2:
			_bmp180Mode = 1;
			break;
		case BMP180_OS_4:
			_bmp180Mode = 2;
			break;
		default:
			_bmp180Mode = 3;
			break;
	}
	_bmp180_read_press_cmd = BMP180_READ_PRESS + (_bmp180Mode << 6);
	read_coefficients();
	// Discard the first n measures
	if(! discard_measures_baro(BMP180_DISCARDED_MEASURES, BMP180_DISCARD_TIMEOUT)){
		return 0;
	}
	if(! read_thermo_STATUS(BMP180_DISCARD_TIMEOUT)){
		return 0;
	}	
	return 1;
}

//--------------------Read Calibration-----------------------//
void BMP180::read_coefficients(){
	uint8_t buf[2];
	readMultipleRegisters(buf, 2, BMP180_CAL_AC1);
    _bmp180_calib.AC1v = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(buf, 2, BMP180_CAL_AC2);
    _bmp180_calib.AC2v = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(buf, 2, BMP180_CAL_AC3);
    _bmp180_calib.AC3v = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(buf, 2, BMP180_CAL_AC4);
    _bmp180_calib.AC4v = (uint16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(buf, 2, BMP180_CAL_AC5);
    _bmp180_calib.AC5v = (uint16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(buf, 2, BMP180_CAL_AC6);
    _bmp180_calib.AC6v = (uint16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(buf, 2, BMP180_CAL_B1);
    _bmp180_calib.B1v = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(buf, 2, BMP180_CAL_B2);
    _bmp180_calib.B2v = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(buf, 2, BMP180_CAL_MB);
    _bmp180_calib.MBv = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(buf, 2, BMP180_CAL_MC);
    _bmp180_calib.MCv = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(buf, 2, BMP180_CAL_MD);
    _bmp180_calib.MDv = (int16_t) ((buf[1] << 8) | buf [0]);
}

//------------------------Read data-------------------------//
uint8_t BMP180::read_raw_baro(){
	uint8_t buf[3];
	readMultipleRegisters(buf, 3, BMP180_OUT_MSB);
	_bmp180_calib.UP = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> (8 - _bmp180Mode);
	return 1;
}

//--------Temperature compensated pressure reading----------//
uint8_t BMP180::compensate_baro(){
	int32_t  X1, X2, B5, B6, X3, B3, p;
	uint32_t B4, B7;	
	/* Temperature compensation */
	B5 = computeB5(_bmp180_calib.UT);
	/* Pressure compensation */
	B6 = B5 - 4000;
	X1 = (_bmp180_calib.B2v * ((B6 * B6) >> 12)) >> 11;
	X2 = (_bmp180_calib.AC2v * B6) >> 11;
	X3 = X1 + X2;
	B3 = (((((int32_t) _bmp180_calib.AC1v) * 4 + X3) << _bmp180Mode) + 2) >> 2;
	X1 = (_bmp180_calib.AC3v * B6) >> 13;
	X2 = (_bmp180_calib.B1v * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (_bmp180_calib.AC4v * (uint32_t) (X3 + 32768)) >> 15;
	B7 = ((uint32_t) (_bmp180_calib.UP - B3) * (50000 >> _bmp180Mode));
	if (B7 < 0x80000000){
		p = (B7 << 1) / B4;
	}
	else{
		p = (B7 / B4) << 1;
	}
	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	press = (p + ((X1 + X2 + 3791) >> 4));
	return 1;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t BMP180::read_baro_STATUS(uint32_t timeout){
	uint32_t now = micros();
	if(! writeRegister(BMP180_CTRL_MEAS, _bmp180_read_press_cmd)){
		return 0;
	}
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(BMP180_CTRL_MEAS);
		if (!(STATUS_val & (1 << 5))){
			read_raw_baro();
			compensate_baro();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//----------------------Baro Status------------------------//
uint8_t BMP180::status_baro(){
	return readRegister(BMP180_CTRL_MEAS);
}

//-------------------Discard measures----------------------//
uint8_t BMP180::discard_measures_baro(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	if(! writeRegister(BMP180_CTRL_MEAS, _bmp180_read_press_cmd)){
		return 0;
	}
	while (count < number_of_measures){
		uint8_t STATUS_value = status_baro();
		if (!(STATUS_value & (1 << 5))){
			if(! writeRegister(BMP180_CTRL_MEAS, _bmp180_read_press_cmd)){
				return 0;
			}
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
uint8_t BMP180::read_raw_thermo(){
	int32_t B5;     // following ds convention
	uint8_t buf[2];
	readMultipleRegisters(buf, 2, BMP180_OUT_MSB);
	_bmp180_calib.UT = (uint16_t) ((buf[0] << 8) | buf[1]);
	B5 = computeB5(_bmp180_calib.UT);
	temperature = (B5 + 8) >> 4;
	temperature *= 0.1;
	return 1;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t BMP180::read_thermo_STATUS(uint32_t timeout){
	uint32_t now = micros();
	if(! writeRegister(BMP180_CTRL_MEAS, BMP180_READ_TEMP)){
		return 0;
	}
	while((micros() - now) < timeout){
		uint8_t STATUS_val = status_baro();
		if (!(STATUS_val & (1 << 5))){
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
uint8_t BMP180::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	if(! writeRegister(BMP180_CTRL_MEAS, BMP180_READ_TEMP)){
		return 0;
	}
	while (count < number_of_measures){
		uint8_t STATUS_value = status_baro();
		if (!(STATUS_value & (1 << 5))){
			if(! writeRegister(BMP180_CTRL_MEAS, BMP180_READ_TEMP)){
				return 0;
			}
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
