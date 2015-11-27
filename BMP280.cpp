//
//  BMP280.cpp
//
//
//  Created by Andrea Vivani on 01/11/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "BMP280.h"
#include "Arduino.h"
#include <SPI.h>
//====================================Registers Addresses=========================================// 
#define BMP280_DIG_T1			0x88
#define BMP280_DIG_T2			0x8A
#define BMP280_DIG_T3			0x8C
#define BMP280_DIG_P1			0x8E
#define BMP280_DIG_P2			0x90
#define BMP280_DIG_P3			0x92
#define BMP280_DIG_P4			0x94
#define BMP280_DIG_P5			0x96
#define BMP280_DIG_P6			0x98
#define BMP280_DIG_P7			0x9A
#define BMP280_DIG_P8			0x9C
#define BMP280_DIG_P9			0x9E
#define BMP280_CHIP_ID			0xD0
#define BMP280_RST_REG 			0xE0
#define BMP280_STAT_REG			0xF3
#define BMP280_CTRL_MEAS		0xF4
#define BMP280_CONFIG			0xF5
#define BMP280_PRESSURE_MSB		0xF7
#define BMP280_PRESSURE_LSB		0xF8
#define BMP280_PRESSURE_XLSB	0xF9
#define BMP280_TEMPERATURE_MSB	0xFA
#define BMP280_TEMPERATURE_LSB	0xFB
#define BMP280_TEMPERATURE_XLSB	0xFC
//=======================================Constants=============================================// 
#define BMP280_ID				0x58
#define BMP280_READ				0x80

//==================================Auxiliary Functions========================================//
//---------------Read one register from the SPI-----------------//
uint8_t BMP280::readRegister(uint8_t chipSelectPin, uint8_t thisRegister) {
  	uint8_t inByte = 0;    	// incoming byte
  	thisRegister |= BMP280_READ;		// register in read mode
  	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  	SPI.transfer(thisRegister);		// send the command to read thisRegister
  	inByte = SPI.transfer(0x00);		// send 0x00 in order to read the incoming byte
  	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to select the chip
  	return(inByte);			// return the read byte
}

//------------Read multiple registers from the SPI--------------//
void BMP280::readMultipleRegisters(uint8_t chipSelectPin, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister) {
  	startRegister |= BMP280_READ;// register in multiple read mode
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  	SPI.transfer(startRegister);		// send the command to read thisRegister
  	while(number_of_registers--){
  		*buffer++ = SPI.transfer(0x00);
  	}
  	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
  	return;
}

//---------------Write one register on the SPI-----------------//
void BMP280::writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue) {
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(thisRegister); 		// send register location
	SPI.transfer(thisValue);  		// send value to record into register
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to select the chip
}

//-------------Calibrate pressure (64 bit formula)-------------//
void BMP280::baro_cal_64_bit(int32_t adc_P){
	int64_t var1, var2;
  	var1 = ((int64_t) _bmp280_calib.t_fine) - 128000;
  	var2 = var1 * var1 * (int64_t) _bmp280_calib.dig_P6;
  	var2 = var2 + ((var1*(int64_t) _bmp280_calib.dig_P5) << 17);
  	var2 = var2 + (((int64_t) _bmp280_calib.dig_P4) << 35);
  	var1 = ((var1 * var1 * (int64_t) _bmp280_calib.dig_P3) >> 8) + ((var1 * (int64_t) _bmp280_calib.dig_P2) << 12);
  	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) _bmp280_calib.dig_P1) >> 33;
  	if (var1 == 0) {
		_bmp280_calib.p = 0.0;
  		press  = 0.0;
    	return;  // avoid exception caused by division by zero
  	}
  	_bmp280_calib.p = 1048576 - adc_P;
  	_bmp280_calib.p = (((_bmp280_calib.p << 31) - var2) * 3125) / var1;
  	var1 = (((int64_t) _bmp280_calib.dig_P9) * (_bmp280_calib.p >> 13) * (_bmp280_calib.p >> 13)) >> 25;
  	var2 = (((int64_t) _bmp280_calib.dig_P8) * _bmp280_calib.p) >> 19;
 	_bmp280_calib.p = ((_bmp280_calib.p + var1 + var2) >> 8) + (((int64_t) _bmp280_calib.dig_P7) << 4);
  	press = (float) _bmp280_calib.p * 0.00390625;
	return;
}

//-------------Calibrate pressure (32 bit formula)-------------//
void BMP280::baro_cal_32_bit(int32_t adc_P){
	int32_t var1, var2;
	var1 = (((int32_t) _bmp280_calib.t_fine) >> 1) - (int32_t) 64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t) _bmp280_calib.dig_P6);
	var2 = var2 + ((var1 * ((int32_t) _bmp280_calib.dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t) _bmp280_calib.dig_P4) << 16);
	var1 = (((_bmp280_calib.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t) _bmp280_calib.dig_P2) * var1) >> 1)) >> 18; 
	var1 = ((((32768 + var1)) * ((int32_t) _bmp280_calib.dig_P1)) >> 15);
	if (var1 == 0) {
		_bmp280_calib.p = 0.0;
		press  = 0.0;
		return; // avoid exception caused by division by zero 
	}
	_bmp280_calib.p = (((uint32_t) (( (int32_t) 1048576) - adc_P)-(var2 >> 12))) * 3125;
	if (_bmp280_calib.p < 0x80000000) {
		_bmp280_calib.p = (_bmp280_calib.p << 1) / ((uint32_t) var1);
	}
	else
	{
	    _bmp280_calib.p = (_bmp280_calib.p / (uint32_t) var1) * 2;
	}
	var1 = (((int32_t) _bmp280_calib.dig_P9) * ((int32_t) (((_bmp280_calib.p >> 3) * (_bmp280_calib.p >> 3)) >> 13))) >> 12;
	var2 = (((int32_t) (_bmp280_calib.p >> 2)) * ((int32_t) _bmp280_calib.dig_P8)) >> 13;
	_bmp280_calib.p = (uint32_t)((int32_t) _bmp280_calib.p + ((var1 + var2 + _bmp280_calib.dig_P7) >> 4));
	press = (float) _bmp280_calib.p;
	return;
}

//-----------------Calibrate temperature------------------//
void BMP280::thermo_cal(int32_t adc_T){
	int32_t var1, var2;
	var1  = ((((adc_T >> 3) - ((int32_t) _bmp280_calib.dig_T1 << 1))) * ((int32_t) _bmp280_calib.dig_T2)) >> 11;
	var2  = (((((adc_T >> 4) - ((int32_t) _bmp280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t) _bmp280_calib.dig_T1))) >> 12) * ((int32_t) _bmp280_calib.dig_T3)) >> 14;
	_bmp280_calib.t_fine = var1 + var2;
	temperature = (_bmp280_calib.t_fine * 5 + 128) >> 8;
	temperature *= 0.01;
	return;
}

//=====================================Constructors==========================================//
BMP280::BMP280 (uint8_t CS_pin):InertialSensor(){
	_chipSelectPin = CS_pin;
}

void BMP280::init(){
	pinMode(_chipSelectPin,OUTPUT);
	digitalWrite(_chipSelectPin,HIGH);
	press = 0;
	temperature = 0;
}

//===================================Public Members=========================================//
//-----------------------Configuration-----------------------//
uint8_t BMP280::config_baro(uint8_t standy_time, uint8_t mode, uint8_t press_oversamp, uint8_t temp_oversamp, uint8_t IIR_coeff){
	init();
	// Trash the first reading
	readRegister(_chipSelectPin, BMP280_CHIP_ID);
	// Check if the device ID is correct
	if (readRegister(_chipSelectPin, BMP280_CHIP_ID)!= BMP280_ID){
		return 0;
	}
	// selected standby time, IIR filter coefficient and 4-wires SPI
	uint8_t CONFIG_val = standy_time | IIR_coeff | 0x00;
	writeRegister(_chipSelectPin, BMP280_CONFIG, CONFIG_val);
	// selected temp oversampling, pressure oversampling and mode
	_CTRL_MEAS_val = temp_oversamp | press_oversamp | mode;
	writeRegister(_chipSelectPin, BMP280_CTRL_MEAS, _CTRL_MEAS_val);
	writeRegister(_chipSelectPin, BMP280_CTRL_MEAS, _CTRL_MEAS_val);
	delay(20);
	read_coefficients();
	// Discard the first n measures
	if(! discard_measures_baro(BMP280_DISCARDED_MEASURES, BMP280_DISCARD_TIMEOUT)){
		return 0;
	}
	if(! read_thermo_STATUS(BMP280_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//--------------------Read Calibration-----------------------//
void BMP280::read_coefficients(){
	uint8_t buf[2];
	readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_T1);
    _bmp280_calib.dig_T1 = (uint16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_T2);
    _bmp280_calib.dig_T2 = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_T3);
    _bmp280_calib.dig_T3 = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_P1);
    _bmp280_calib.dig_P1 = (uint16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_P2);
    _bmp280_calib.dig_P2 = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_P3);
    _bmp280_calib.dig_P3 = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_P4);
    _bmp280_calib.dig_P4 = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_P5);
    _bmp280_calib.dig_P5 = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_P6);
    _bmp280_calib.dig_P6 = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_P7);
    _bmp280_calib.dig_P7 = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_P8);
    _bmp280_calib.dig_P8 = (int16_t) ((buf[1] << 8) | buf [0]);
    readMultipleRegisters(_chipSelectPin, buf, 2, BMP280_DIG_P9);
    _bmp280_calib.dig_P9 = (int16_t) ((buf[1] << 8) | buf [0]);
}

//-------------------------Turn on---------------------------//
void BMP280::turn_on_baro(){
	writeRegister(_chipSelectPin, BMP280_CTRL_MEAS, _CTRL_MEAS_val);
	delay(20);
}

//------------------------Turn off---------------------------//
void BMP280::turn_off_baro(){
	writeRegister(_chipSelectPin, BMP280_CTRL_MEAS, (_CTRL_MEAS_val & 0xFC));
}

//------------------------Read data-------------------------//
uint8_t BMP280::read_raw_baro(){
	uint8_t buf[3];
  	readMultipleRegisters(_chipSelectPin, buf, 3, BMP280_PRESSURE_MSB);
  	int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	baro_cal_64_bit(adc_P);
  	return 1;
}

//---------------------Read data 32 bit----------------------//
uint8_t BMP280::read_raw_baro_32(){
	uint8_t buf[3];
	readMultipleRegisters(_chipSelectPin, buf, 3, BMP280_PRESSURE_MSB);
	int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	baro_cal_32_bit(adc_P);
	return 1;
}

//---------Read temperature compensated pressure-----------//
uint8_t BMP280::read_baro_compensated(){
	uint8_t buf[6];
	readMultipleRegisters(_chipSelectPin, buf, 6, BMP280_PRESSURE_MSB);
	int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	int32_t adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
	// Temperature 
	thermo_cal(adc_T);
	// Pressure
	baro_cal_64_bit(adc_P);
	return 1;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t BMP280::read_baro_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin, BMP280_STAT_REG);
		if ((STATUS_val & (1 << 3)) == 0x00){
			read_baro_compensated();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//--------------------Single reading----------------------//
uint8_t BMP280::read_single(uint32_t timeout){
	// sensor into forced mode
	writeRegister(_chipSelectPin, BMP280_CTRL_MEAS, (_CTRL_MEAS_val & 0xFD));
	return read_baro_STATUS(timeout);
}

//----------------------Baro Status------------------------//
uint8_t BMP280::status_baro(){
	return readRegister(_chipSelectPin, BMP280_STAT_REG);
}

//-------------------Discard measures----------------------//
uint8_t BMP280::discard_measures_baro(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	while (count < number_of_measures){
		uint8_t STATUS_value = status_baro();
		if ((STATUS_value & (1 << 3)) == 0x00){
			read_raw_baro();
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
uint8_t BMP280::read_raw_thermo(){
	uint8_t buf[3];
	readMultipleRegisters(_chipSelectPin, buf, 3, BMP280_TEMPERATURE_MSB);
	int32_t adc_T = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	thermo_cal(adc_T);
	return 1;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t BMP280::read_thermo_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = status_baro();
		if ((STATUS_val & (1 << 3)) == 0x00){
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
uint8_t BMP280::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	while (count < number_of_measures){
		uint8_t STATUS_value = status_baro();
		if ((STATUS_value & (1 << 3)) == 0x00){
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
