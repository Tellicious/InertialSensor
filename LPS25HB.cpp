//
//  LPS25HB.cpp
//
//
//  Created by Andrea Vivani on 20/6/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "LPS25HB.h"
#include "Arduino.h"
#include <SPI.h>

//====================================Registers Addresses=========================================// 
#define LPS25HB_REF_P_XL	0x08
#define LPS25HB_REF_P_L 	0x09
#define LPS25HB_REF_P_L 	0x0A
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
#define LPS25HB_RPDS_L		0x3A
//=======================================Constants=============================================// 
#define LPS25HB_ID			0xBD
#define LPS25HB_READ		0x80
#define LPS25HB_MULT		0x40

//==================================Auxiliary Functions========================================//
//---------------Read one register from the SPI-----------------//
uint8_t LPS25HB::readRegister(uint8_t chipSelectPin, uint8_t thisRegister) {
  uint8_t inByte = 0;           	// incoming byte
  thisRegister |= LPS25HB_READ;		// register in read mode
  digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  SPI.transfer(thisRegister);		// send the command to read thisRegister
  inByte = SPI.transfer(0x00);		// send 0x00 in order to read the incoming byte
  digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to select the chip
  return(inByte);			// return the read byte
}

//------------Read multiple registers from the SPI--------------//
void LPS25HB::readMultipleRegisters(uint8_t chipSelectPin, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister) {
  startRegister |= (LPS25HB_READ | LPS25HB_MULT);// register in multiple read mode
  digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  SPI.transfer(startRegister);		// send the command to read thisRegister
  while (number_of_registers--){
  	*buffer++ = SPI.transfer(0x00);
  }
  digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
  return;
}

//---------------Write one register on the SPI-----------------//
void LPS25HB::writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue) {
  digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
  SPI.transfer(thisRegister); 		// send register location
  SPI.transfer(thisValue);  		// send value to record into register
  digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to select the chip
}

//=====================================Constructors==========================================//
LPS25HB::LPS25HB (uint8_t CS_pin):InertialSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = 0;
}

LPS25HB::LPS25HB (uint8_t CS_pin, uint8_t DRDY_pin):InertialSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin = DRDY_pin;
}

void LPS25HB::init(){
	pinMode(_chipSelectPin,OUTPUT);
	digitalWrite(_chipSelectPin,HIGH);
	if (_DRDY_pin != 0){
		pinMode(_DRDY_pin,INPUT);
	}
	press = 0;
	temperature = 0;
}

//===================================Public Members=========================================//
//-----------------------Configuration-----------------------//
uint8_t LPS25HB::config_baro(uint8_t odr_conf, uint8_t AVGT, uint8_t AVGP, uint8_t MA_FIFO){
	init();
	// Trash the first reading
	readRegister(_chipSelectPin, LPS25HB_WHO_AM_I);
	// Check if the device ID is correct
	if (readRegister(_chipSelectPin, LPS25HB_WHO_AM_I)!= LPS25HB_ID){
		return 0;
	}
	//
	//selected averages
	uint8_t RES_CONF_val = (AVGT | AVGP) & 0x0F; 
	writeRegister(_chipSelectPin, LPS25HB_RES_CONF,RES_CONF_val);
	//
	//FIFO enabled, I2C disabled, autozero off, one-shot off
	uint8_t CTRL2_val = (1 << 6);
	writeRegister(_chipSelectPin, LPS25HB_CTRL2,CTRL2_val);
	//
	//Interrupt active high, push pull, data signal on interrupt
	uint8_t CTRL3_val = (1 << 3);
	writeRegister(_chipSelectPin, LPS25HB_CTRL3,CTRL3_val);
	//
	//DRDY on interrupt
	uint8_t CTRL4_val = 1; 
	writeRegister(_chipSelectPin, LPS25HB_CTRL4,CTRL4_val);
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
	writeRegister(_chipSelectPin, LPS25HB_FIFO_CTRL,FIFO_CTRL_val);
	//
	//power on, selected ODR, continuous update, SPI 4 wire
	_CTRL1_val = (1 << 7) | odr_conf | (1 << 3);
	writeRegister(_chipSelectPin, LPS25HB_CTRL1,_CTRL1_val);
	delay(37);
	// Discard the first n measures
	if(! discard_measures_baro(LPS25HB_DISCARDED_MEASURES,LPS25HB_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-------------------------Turn on---------------------------//
void LPS25HB::turn_on_baro(){
	writeRegister(_chipSelectPin, LPS25HB_CTRL1,_CTRL1_val);
	delay(37);
}

//------------------------Turn off---------------------------//
void LPS25HB::turn_off_baro(){
	writeRegister(_chipSelectPin, LPS25HB_CTRL1,(_CTRL1_val & 0x7F));
}

//------------------------Read data-------------------------//
uint8_t LPS25HB::read_raw_baro(){
	uint8_t buffer[3];
  	readMultipleRegisters(_chipSelectPin, buffer, 3, LPS25HB_OUT_XL);
  	press = (float) (((int32_t) (int8_t) buffer[2] << 16 | (uint16_t) buffer[1] << 8 | buffer[0]) * _sc_fact);
  	return 1;
}

//------------------Read data when ready--------------------//
uint8_t LPS25HB::read_baro_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin) == 1){
			read_raw_baro();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LPS25HB::read_baro_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin, LPS25HB_STATUS);
		if ((STATUS_val & (1 << 1)) == (1 << 1)){
			read_raw_baro();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//----------------------Baro Status------------------------//
uint8_t LPS25HB::status_baro(){
	return readRegister(_chipSelectPin, LPS25HB_STATUS);
}

//-------------------Discard measures----------------------//
uint8_t LPS25HB::discard_measures_baro(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	while (count < (number_of_measures * 0.5)){
		uint8_t STATUS_value = status_baro();
		if ((STATUS_value & (1 << 5)) == (1 << 5)){
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
uint8_t LPS25HB::read_raw_thermo(){
	uint8_t buffer[2];
	readMultipleRegisters(_chipSelectPin, buffer, 2, LPS25HB_TEMP_OUT_L);
	int16_t temperature_tmp = (((int16_t) buffer[1] << 8) | buffer[0]);
	temperature = 42.5 + (float) temperature_tmp / 480.0;
	return 1;
}

//------------------Read data when ready-----------------//
uint8_t LPS25HB::read_thermo_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin) == 1){
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
uint8_t LPS25HB::read_thermo_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = status_baro();
		if ((STATUS_val & 0x01) == 0x01){
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
uint8_t LPS25HB::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	while (count < (number_of_measures * 0.5)){
		uint8_t STATUS_value = status_baro();
		if ((STATUS_value & (1 << 4)) == (1 << 4)){
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
