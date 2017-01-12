//
//  HMC5983.cpp
//
//
//  Created by Andrea Vivani on 22/5/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "HMC5983.h"
#include "INS_AuxFun.h"
#ifdef INS_ARDUINO
#include "Arduino.h"
#include <SPI.h>
#endif
//====================================Registers Addresses=========================================// 
#define HMC5983_CFG_A		0x00
#define HMC5983_CFG_B		0x01
#define HMC5983_MODE		0x02
#define HMC5983_OUT_X_H		0x03
#define HMC5983_OUT_X_L		0x04
#define HMC5983_OUT_Z_H		0x05
#define HMC5983_OUT_Z_L		0x06
#define HMC5983_OUT_Y_H		0x07
#define HMC5983_OUT_Y_L		0x08
#define HMC5983_STATUS		0x09
#define HMC5983_ID_REG_A	0x0A
#define HMC5983_ID_REG_B	0x0B
#define HMC5983_ID_REG_C	0x0C
#define HMC5983_OUT_TEMP_H	0x31
#define HMC5983_OUT_TEMP_L	0x32
//=========================================Constants==============================================// 
#define HMC5983_ID_A 		0x48
#define HMC5983_ID_B 		0x34
#define HMC5983_ID_C 		0x33
#define HMC5983_READ		0x80
#define HMC5983_MULT		0x40
//==================================Auxiliary Functions========================================//
#ifdef INS_ARDUINO
  #define HMC5983_READ_REGISTER(reg) INS_SPI_readRegister(_chipSelectPin, reg, HMC5983_READ)
  #define HMC5983_READ_MULTIPLE_REGISTERS(buf, num, start) INS_SPI_readMultipleRegisters(_chipSelectPin, buf, num, startRegister, (HMC5983_READ | HMC5983_MULT))
  #define HMC5983_WRITE_REGISTER(reg, val) INS_SPI_writeRegister(_chipSelectPin, reg, val, 0x00)
#elif defined(INS_CHIBIOS)
  #define HMC5983_READ_REGISTER(reg) INS_SPI_readRegister(_SPI_int, _spicfg, reg, HMC5983_READ)
  #define HMC5983_READ_MULTIPLE_REGISTERS(buf, num, start) INS_SPI_readMultipleRegisters(_SPI_int, _spicfg, buf, num, start, (HMC5983_READ | HMC5983_MULT))
  #define HMC5983_WRITE_REGISTER(reg, val) INS_SPI_writeRegister(_SPI_int, _spicfg, reg, val, 0x00)
#endif
//=====================================Constructors==========================================//
#ifdef INS_ARDUINO
HMC5983::HMC5983 (uint8_t CS_pin):InertialSensor(), MagnetometerSensor(), ThermometerSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = 0;
}

HMC5983::HMC5983 (uint8_t CS_pin, uint8_t DRDY_pin):InertialSensor(), MagnetometerSensor(), ThermometerSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = DRDY_pin;
}

//-----------------------Initialization-----------------------//
void HMC5983::init(){
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
HMC5983::HMC5983 (SPIDriver* SPI, SPIConfig* spicfg):InertialSensor(), MagnetometerSensor(), ThermometerSensor(){
	_SPI_int = SPI;
	_spicfg = spicfg;
	_DRDY_pin = 0;
    init();
}

HMC5983::HMC5983 (SPIDriver* SPI, SPIConfig* spicfg, ioportid_t gpio_DRDY, uint8_t DRDY_pin):InertialSensor(), MagnetometerSensor(), ThermometerSensor(){
	_SPI_int = SPI;
	_spicfg = spicfg;
	_gpio_DRDY = gpio_DRDY;
	_DRDY_pin = DRDY_pin;
    init();
}

//-----------------------Initialization-----------------------//
void HMC5983::init(){
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
uint8_t HMC5983::config_mag(uint8_t range_conf, uint8_t odr_conf, uint8_t average_conf, uint8_t meas_mode){
	init();
	// Trash the first reading 
	HMC5983_READ_REGISTER(HMC5983_ID_REG_A);
	// Check if the device ID is correct
	if ((HMC5983_READ_REGISTER(HMC5983_ID_REG_A) != HMC5983_ID_A) || (HMC5983_READ_REGISTER(HMC5983_ID_REG_B) != HMC5983_ID_B) || (HMC5983_READ_REGISTER(HMC5983_ID_REG_C) != HMC5983_ID_C)){
		return 0;
	}
	//
	//Temp. sensor enabled, selected average number, selected ODR, normal measurement mode
	uint8_t CFG_A_val = 0x80 | average_conf | odr_conf;
	HMC5983_WRITE_REGISTER(HMC5983_CFG_A, CFG_A_val);
	//
	//Selected range
	HMC5983_WRITE_REGISTER(HMC5983_CFG_B, range_conf);
	//
	switch (range_conf){
		case (HMC5983_RANGE_0_88):
			_sc_fact = 0.73e-3f;
			break;
		case (HMC5983_RANGE_1_3):
			_sc_fact = 0.92e-3f;
			break;
		case (HMC5983_RANGE_1_9):
			_sc_fact = 1.22e-3f;
			break;
		case (HMC5983_RANGE_2_5):
			_sc_fact = 1.52e-3f;
			break;
		case (HMC5983_RANGE_4):
			_sc_fact = 2.27e-3f;
			break;
		case (HMC5983_RANGE_4_7):
			_sc_fact = 2.56e-3f;
			break;
		case (HMC5983_RANGE_5_6):
			_sc_fact = 3.03e-3f;
			break;
		case (HMC5983_RANGE_8_1):
			_sc_fact = 4.35e-3f;
			break;
		default:
			return 2;
	}
	//
	// 4 wire SPI, selected measurement mode
	_MODE_val = meas_mode;
	turn_on_mag();
	// Discard the first n measures
	if(! discard_measures_mag(HMC5983_DISCARDED_MEASURES, HMC5983_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-------------------------Turn on---------------------------//
void HMC5983::turn_on_mag(){
	HMC5983_WRITE_REGISTER(HMC5983_MODE, _MODE_val);
#ifdef INS_ARDUINO
	delay(50);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(50);
#endif
}

//------------------------Turn off---------------------------//
void HMC5983::turn_off_mag(){
	HMC5983_WRITE_REGISTER(HMC5983_MODE, (_MODE_val | 0x3));
}

//-------------------------Sleep-----------------------------//
void HMC5983::sleep_mag(){
	HMC5983_WRITE_REGISTER(HMC5983_MODE, (_MODE_val | 0x20));
}

//------------------------Read data-------------------------//
uint8_t HMC5983::read_raw_mag(){
	uint8_t buffer[6];
  	HMC5983_READ_MULTIPLE_REGISTERS(buffer, 6, HMC5983_OUT_X_H);
  	x = (float) (((int16_t) (buffer[0] << 8) | buffer[1]) * _sc_fact);
  	y = (float)	(((int16_t) (buffer[4] << 8) | buffer[5]) * _sc_fact);
  	z = (float)	(((int16_t) (buffer[2] << 8) | buffer[3]) * _sc_fact);
  	return 1;
}

//------------------Read data when ready--------------------//
uint8_t HMC5983::read_mag_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
	INS_read_DRDY(timeout, read_raw_mag, _DRDY_pin)
#elif defined(INS_CHIBIOS)
	INS_read_DRDY(timeout, read_raw_mag, _gpio_DRDY, _DRDY_pin)
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t HMC5983::read_mag_STATUS(uint32_t timeout){
#ifdef INS_ARDUINO
	uint32_t now = micros();
	while((micros() - now) < timeout){
#elif defined(INS_CHIBIOS)
    systime_t end = chVTGetSystemTime() + US2ST(timeout);
    while (chVTGetSystemTime() < end){
#endif
		uint8_t STATUS_val = HMC5983_READ_REGISTER(HMC5983_STATUS);
		if (STATUS_val & 0x01){
			read_raw_mag();
			return 1;
		}
		else if (STATUS_val & 0x02){	//it means that the sensor is locked
			HMC5983_WRITE_REGISTER(HMC5983_MODE, _MODE_val); //reset the sensor
		}
#ifdef INS_ARDUINO
		if ((int32_t) (micros() - now) < 0){
			now = 0L;
		}
#endif
	}
return 0;
}

//---------Read data when ready, in single read mode-----------//
uint8_t HMC5983::read_mag_single_DRDY(uint32_t timeout){
	HMC5983_WRITE_REGISTER(HMC5983_MODE, _MODE_val);
	return read_mag_DRDY(timeout);
}

//---------Read data when ready (STATUS register), in single read mode-----------//
uint8_t HMC5983::read_mag_single_STATUS(uint32_t timeout){
	HMC5983_WRITE_REGISTER(HMC5983_MODE, _MODE_val);
	return read_mag_STATUS(timeout);
}

//-----------------------Self-Test-------------------------//
uint8_t HMC5983::self_test_mag(uint8_t mode){
	uint8_t status = 0;
	// turn off mag and turn on self test
	turn_off_mag();
	uint8_t CFG_A_val = HMC5983_READ_REGISTER(HMC5983_CFG_A);
	if (mode==0){ //positive bias
		HMC5983_WRITE_REGISTER(HMC5983_CFG_A, (CFG_A_val | 0x01));
	}
	else {	//negative bias
		HMC5983_WRITE_REGISTER(HMC5983_CFG_A, (CFG_A_val | 0x02));
	}
	// put into continuous measurement mode
	HMC5983_WRITE_REGISTER(HMC5983_MODE, (_MODE_val & 0xFC));
#ifdef INS_ARDUINO
	delay(50);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(50);
#endif
	// Discard the first n measures 
	if(! discard_measures_mag(HMC5983_DISCARDED_MEASURES_ST, HMC5983_DISCARD_TIMEOUT)){
		return 0;
	}
	read_mag_STATUS(HMC5983_DISCARD_TIMEOUT);
	// define threshold as Gauss
	float thrs_min = 0.623076923076923f;
	float thrs_max = 1.474358974358974f;
	// Check if values are bigger than the threshold
	if (INS_ch_st(0, x, thrs_min, thrs_max) && INS_ch_st(0, y, thrs_min, thrs_max) && INS_ch_st(0, z, thrs_min, thrs_max)) {
		status = 1;
	}
	turn_off_mag();
	//remove self test
	HMC5983_WRITE_REGISTER(HMC5983_CFG_A, CFG_A_val);
	turn_on_mag();
	// Discard the first n measures
	if(! discard_measures_mag(HMC5983_DISCARDED_MEASURES_ST, HMC5983_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Mag Status------------------------//
uint8_t HMC5983::status_mag(){
	return HMC5983_READ_REGISTER(HMC5983_STATUS);
}

//-------------------Discard measures----------------------//
uint8_t HMC5983::discard_measures_mag(uint8_t number_of_measures, uint32_t timeout){
	INS_discard_measures_over(number_of_measures, timeout, read_raw_mag, status_mag, (1 << 4))
}

//=============================Public Members Temperature====================================//
//------------------------Read data----------------------//
uint8_t HMC5983::read_raw_thermo(){
	uint8_t buffer[2];
	HMC5983_READ_MULTIPLE_REGISTERS(buffer, 2, HMC5983_OUT_TEMP_H);
	temperature = 25 + (float) ((((int16_t) buffer[0] << 8) | buffer[1]) >> 7);
	return 1;
}

//------------------Read data when ready-----------------//
uint8_t HMC5983::read_thermo_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin)){
#elif defined(INS_CHIBIOS)
    systime_t end = chVTGetSystemTime() + US2ST(timeout);
    while (chVTGetSystemTime() <= end){
		if (palReadPad(_gpio_DRDY, _DRDY_pin)){
#endif
			read_raw_thermo();
			return 1;
		}
#ifdef INS_ARDUINO
		if ((int32_t) (micros() - now) < 0){
			now = 0L;
		}
#endif
	}
	return 0;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t HMC5983::read_thermo_STATUS(uint32_t timeout){
#ifdef INS_ARDUINO
	uint32_t now = micros();
	while((micros() - now) < timeout){
#elif defined(INS_CHIBIOS)
    systime_t end = chVTGetSystemTime() + US2ST(timeout);
    while (chVTGetSystemTime() <= end){
#endif
		uint8_t STATUS_val = HMC5983_READ_REGISTER(HMC5983_STATUS);
		if (STATUS_val & 0x01){
			read_raw_thermo();
			return 1;
		}
#ifdef INS_ARDUINO
		if ((int32_t) (micros() - now) < 0){
			now = 0L;
		}
#endif
	}
return 0;
}

//-------------------Discard measures----------------------//
uint8_t HMC5983::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	while (count < (number_of_measures)){
		if(read_thermo_STATUS(timeout)){
			count++;
		}
		else{
			return 0;
		}
	}
	return 1;
}
