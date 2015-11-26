//  ADXL345.h
//
//
//  Created by Andrea Vivani on 18/7/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
//======================================Parameters=============================================//
#define ADXL345_GYRO_SELF_TEST_MEASURES 10 //number of samples to be averaged when performing gyroscope self-test
#define ADXL345_ACCEL_SELF_TEST_MEASURES 10 //number of samples to be averaged when performing accelerometer self-test
#define ADXL345_DISCARDED_MEASURES	5	//number of measures to be discarded when performing automatic tasks (greater than 1, preferably even)
#define ADXL345_DISCARDED_MEASURES_ST 10 //number of measures to be discarded after performing self-tests (greater than 1, preferably even)
#define ADXL345_DISCARD_TIMEOUT		2e6 //timeout time in us between measures when discarding
//============================================ODR Accel================================================//
#define ADXL345_ODR_0_1			0x00
#define ADXL345_ODR_0_2			0x01
#define ADXL345_ODR_0_39		0x02
#define ADXL345_ODR_0_78		0x03
#define ADXL345_ODR_1_56		0x04
#define ADXL345_ODR_3_13		0x05
#define ADXL345_ODR_6_25		0x06
#define ADXL345_ODR_12_5		0x07
#define ADXL345_ODR_25			0x08
#define ADXL345_ODR_50			0x09
#define ADXL345_ODR_100			0x0A
#define ADXL345_ODR_200			0x0B
#define ADXL345_ODR_400			0x0C
#define ADXL345_ODR_800			0x0D
#define ADXL345_ODR_1600		0x0E
#define ADXL345_ODR_3200		0x0F
//====================================Range Values Accel=============================================//
#define ADXL345_RANGE_2			0x00
#define ADXL345_RANGE_4			0x01
#define ADXL345_RANGE_8			0x02
#define ADXL345_RANGE_16		0x03

#ifndef ADXL345_H_
#define ADXL345_H_
#include "InertialSensor.h"
#include <SPI.h>

class ADXL345: public InertialSensor{
	public:
		ADXL345 (uint8_t CS_pin);	//constructor
		ADXL345 (uint8_t CS_pin, uint8_t DRDY_pin);	//constructor with Data ready pin
		virtual void init(); //initializes pins and variables
		float x,y,z;	//accelerometer output data
		//===============Accelerometer================//
		uint8_t config_accel(uint8_t accel_range, uint8_t accel_odr); //configure the gyroscope and the accelerometer
		virtual void turn_on_accel(); //turn on the accelerometer
		virtual void turn_off_accel(); //turn off the accelerometer
		virtual inline uint8_t read_accel(uint32_t timeout){return read_accel_STATUS(timeout);}; //default read method for accelerometer, timeout in us
		uint8_t read_raw_accel(); //read data from accelerometer registers
		uint8_t read_accel_DRDY(uint32_t timeout); //read data from accelerometer if DRDY is high, timeout in us
		uint8_t read_accel_STATUS(uint32_t timeout); //read data from gyroscope if available (reads the status register), timeout in us
		virtual uint8_t check_accel_biases(float bx, float by, float bz); //check if accel biases are within reasonable limits, according to datasheet
		uint8_t self_test_accel(); //self-test
		uint8_t status_accel(); //returns the value of the status register
		virtual uint8_t discard_measures_accel(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called. timeout in us
	private:
		float _sc_fact = INS_G_VAL / 256.0;	//scale factor
		uint8_t _chipSelectPin, _DRDY_pin;	//ChipSelectPin and Data Ready pin
		uint8_t _POWER_CTL_val; //values of the register used when powering up and down the sensor
		uint8_t readRegister(uint8_t chipSelectPin, uint8_t thisRegister);
		void readMultipleRegisters(uint8_t chipSelectPin, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister);
		void writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue);
		uint8_t ch_st (const double val1, const double val2, const double lim1, const double lim2);
};
#endif