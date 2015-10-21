//  LIS3MDL.h
//
//
//  Created by Andrea Vivani on 7/7/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
//======================================Parameters=============================================//
#define LIS3MDL_MAG_SELF_TEST_MEASURES 	10 	//number of samples to be averaged when performing magnetometer self-test
#define LIS3MDL_DISCARDED_MEASURES		5	//number of measures to be discarded when performing automatic tasks (greater than 1, preferably even)
#define LIS3MDL_DISCARDED_MEASURES_ST 	20 //number of measures to be discarded after performing self tests (greater than 1, preferably even)
#define LIS3MDL_DISCARD_TIMEOUT			2e6 //timeout time in us between measures when discarding
//=======================================ODR Values==============================================//
// CTRL_REG1
#define LIS3MDL_ODR_0_625	0x60
#define LIS3MDL_ODR_1_25	0x64
#define LIS3MDL_ODR_2_5		0x68
#define LIS3MDL_ODR_5		0x6C
#define LIS3MDL_ODR_10		0x70
#define LIS3MDL_ODR_20		0x74
#define LIS3MDL_ODR_40		0x78
#define LIS3MDL_ODR_80		0x7C
#define LIS3MDL_ODR_155		0x62
#define LIS3MDL_ODR_300		0x42
#define LIS3MDL_ODR_560		0x22
#define LIS3MDL_ODR_1000	0x02
//=====================================Range Values Mag==============================================//
// CTRL_REG2
#define LIS3MDL_RANGE_4		0x00
#define LIS3MDL_RANGE_8		0x20
#define LIS3MDL_RANGE_12	0x40
#define LIS3MDL_RANGE_16	0x60


#ifndef LIS3MDL_H_
#define LIS3MDL_H_
#include "InertialSensor.h"
#include <SPI.h>


class LIS3MDL: public InertialSensor {
	public:
		LIS3MDL (uint8_t CS_pin);	//constructor
		LIS3MDL (uint8_t CS_pin, uint8_t DRDY_pin);	//constructor with Data ready pin
		virtual void init(); //initializes pins and variables
		float x, y, z;	//output data
		float temperature;
		uint8_t config_mag(uint8_t range_conf, uint8_t odr_conf); //configure the magnetometer
		//===============Magnetometer=================//
		virtual void turn_on_mag(); //turn on the magnetometer
		virtual void turn_off_mag(); //turn off the magnetometer
		virtual inline uint8_t read_mag(uint32_t timeout){return read_mag_STATUS(timeout);}; //default read method for magnetometer, timeout in us
		uint8_t read_raw_mag(); //read data from magnetometer registers
		uint8_t read_mag_DRDY(uint32_t timeout); //read data from magnetometer if DRDY is high, timeout in us
		uint8_t read_mag_STATUS(uint32_t timeout); //read data from magnetometer if available (reads the status register), timeout in us
		uint8_t read_mag_single_DRDY(uint32_t timeout); //read data from magnetometer if DRDY is high, when in single read mode, timeout in us
		uint8_t read_mag_single_STATUS(uint32_t timeout); //read data from magnetometer if available (reads the status register), when in single read mode, timeout in us
		uint8_t self_test_mag();
		uint8_t status_mag(); //returns the value of the status register
		virtual uint8_t discard_measures_mag(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called, timeout in us
		//================Temperature=================//
		virtual void turn_on_thermo(){return turn_on_mag();}; //turn on the thermometer
		virtual void turn_off_thermo(){return;}; //turn off the thermometer
		virtual inline uint8_t read_thermo(uint32_t timeout){return read_raw_thermo();}; //default read method for thermometer 
		uint8_t read_raw_thermo(); //read data from thermometer registers
		uint8_t read_thermo_DRDY(uint32_t timeout); //read temperature when DRDY is high, timeout in us
		uint8_t read_thermo_STATUS(uint32_t timeout); //read data from thermometer if available (reads the status register), timeout in us
		virtual uint8_t discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called, timeout in us
	private:
		float _sc_fact;		//scale factor
		uint8_t _chipSelectPin, _DRDY_pin;	//ChipSelectPin and Data Ready pin
		uint8_t _CTRL3_val; //value of the register, used when powering up and down the sensor
		uint8_t readRegister(uint8_t chipSelectPin, uint8_t thisRegister);
		void readMultipleRegisters(uint8_t chipSelectPin, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister);
		void writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue);
		uint8_t ch_st (const double val1, const double val2, const double lim1, const double lim2);
};
#endif
