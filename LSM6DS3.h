//  LSM6DS3.h
//
//
//  Created by Andrea Vivani on 18/7/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#ifndef LSM6DS3_H_
#define LSM6DS3_H_
#include "InertialSensor.h"
#include <SPI.h>
//======================================Parameters=============================================//
#define LSM6DS3_GYRO_SELF_TEST_MEASURES 10 //number of samples to be averaged when performing gyroscope self-test
#define LSM6DS3_ACCEL_SELF_TEST_MEASURES 10 //number of samples to be averaged when performing accelerometer self-test
#define LSM6DS3_DISCARDED_MEASURES	5	//number of measures to be discarded when performing automatic tasks (greater than 1, preferably even)
#define LSM6DS3_DISCARDED_MEASURES_ST 10 //number of measures to be discarded after performing self-tests (greater than 1, preferably even)
#define LSM6DS3_DISCARD_TIMEOUT		2e6 //timeout time in us between measures when discarding
//============================================ODR Gyro================================================//
#define LSM6DS3_ODR_G_13			0x10
#define LSM6DS3_ODR_G_26			0x20
#define LSM6DS3_ODR_G_52			0x30
#define LSM6DS3_ODR_G_104			0x40
#define LSM6DS3_ODR_G_208			0x50
#define LSM6DS3_ODR_G_416			0x60
#define LSM6DS3_ODR_G_833			0x70
#define LSM6DS3_ODR_G_1660			0x80
//============================================ODR Accel================================================//
#define LSM6DS3_ODR_A_13			0x10
#define LSM6DS3_ODR_A_26			0x20
#define LSM6DS3_ODR_A_52			0x30
#define LSM6DS3_ODR_A_104			0x40
#define LSM6DS3_ODR_A_208			0x50
#define LSM6DS3_ODR_A_416			0x60
#define LSM6DS3_ODR_A_833			0x70
#define LSM6DS3_ODR_A_1660			0x80
#define LSM6DS3_ODR_A_3330			0x90
#define LSM6DS3_ODR_A_6660			0xA0
//====================================Range Values Gyro==============================================//
#define LSM6DS3_RANGE_G_125			0x02
#define LSM6DS3_RANGE_G_245			0x00
#define LSM6DS3_RANGE_G_500			0x04
#define LSM6DS3_RANGE_G_1000		0x08
#define LSM6DS3_RANGE_G_2000		0x0C
//====================================Range Values Accel=============================================//
#define LSM6DS3_RANGE_A_2			0x00
#define LSM6DS3_RANGE_A_4			0x08
#define LSM6DS3_RANGE_A_8			0x0C
#define LSM6DS3_RANGE_A_16			0x04
//=====================================High-pass Gyro==============================================//
#define LSM6DS3_HP_G_0_0081			0x00
#define LSM6DS3_HP_G_0_0324			0x10
#define LSM6DS3_HP_G_2_07			0x20
#define LSM6DS3_HP_G_16_32			0x30
//=====================================Low-pass Accel==============================================//
#define LSM6DS3_LP_A_50				0x03
#define LSM6DS3_LP_A_100			0x02
#define LSM6DS3_LP_A_200			0x01
#define LSM6DS3_LP_A_400			0x00
//====================================High-pass Accel==============================================//
#define LSM6DS3_HP_A_ODR_4			0x00
#define LSM6DS3_HP_A_ODR_100		0x20
#define LSM6DS3_HP_A_ODR_9			0x40
#define LSM6DS3_HP_A_ODR_400		0x60

class LSM6DS3: public InertialSensor{
	public:
		LSM6DS3 (uint8_t CS_pin);	//constructor
		LSM6DS3 (uint8_t CS_pin, uint8_t DRDY_pin_G, uint8_t DRDY_pin_A);	//constructor with Data ready pin
		virtual void init(); //initializes pins and variables
		float gx,gy,gz;	//gyroscope output data
		float ax,ay,az;	//accelerometer output data
		float temperature; //temperature value
		//==================Gyroscope==================//
		uint8_t config_accel_gyro(uint8_t gyro_range, uint8_t gyro_odr, uint8_t HP_enable_gyro, uint8_t HP_freq_gyro, uint8_t accel_range, uint8_t accel_odr, uint8_t LPF_enable_accel, uint8_t LPF_freq_accel, uint8_t HP_enable_accel, uint8_t HP_freq_accel); //configure the gyroscope and the accelerometer
		virtual void turn_on_gyro(); //turn on the gyroscope
		virtual void turn_off_gyro(); //turn off the gyroscope
		virtual void sleep_gyro(); //put the gyroscope to sleep
		virtual inline uint8_t read_gyro(uint32_t timeout){return read_gyro_STATUS(timeout);}; //default read method from gyroscope, timeout in us
		uint8_t read_raw_gyro(); //read data from gyroscope registers
		uint8_t read_gyro_DRDY(uint32_t timeout); //read data from gyroscope if DRDY is high, timeout in us
		uint8_t read_gyro_STATUS(uint32_t timeout); //read data from gyroscope if available (reads the status register). timeout in us
		virtual uint8_t check_gyro_biases(float bx, float by, float bz); //check if gyro biases are within reasonable limits, according to datasheet
		void HP_reset_gyro(); //resets the High-Pass filter
		uint8_t self_test_gyro(uint8_t mode); //self-test
		uint8_t status_gyro(); //returns the value of the status register
		virtual uint8_t discard_measures_gyro(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called, timeout in us
		//===============Accelerometer================//
		virtual void turn_on_accel(); //turn on the accelerometer
		virtual void turn_off_accel(); //turn off the accelerometer
		virtual inline uint8_t read_accel(uint32_t timeout){return read_accel_STATUS(timeout);}; //default read method for accelerometer, timeout in us
		uint8_t read_raw_accel(); //read data from accelerometer registers
		uint8_t read_accel_DRDY(uint32_t timeout); //read data from accelerometer if DRDY is high, timeout in us
		uint8_t read_accel_STATUS(uint32_t timeout); //read data from gyroscope if available (reads the status register), timeout in us
		virtual uint8_t check_accel_biases(float bx, float by, float bz); //check if accel biases are within reasonable limits, according to datasheet
		uint8_t self_test_accel(uint8_t mode); //self-test
		uint8_t status_accel(); //returns the value of the status register
		virtual uint8_t discard_measures_accel(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called. timeout in us
		//================Temperature=================//
		virtual void turn_on_thermo(){return;}; //turn on the thermometer
		virtual void turn_off_thermo(){return;}; //turn off the thermometer
		virtual inline uint8_t read_thermo(uint32_t timeout){return read_thermo_STATUS(timeout);}; //default read method for thermometer 
		uint8_t read_raw_thermo(); //read data from thermometer registers
		uint8_t read_thermo_DRDY(uint32_t timeout); //read temperature when DRDY is high (same INT2 and ODR as magnetometer), timeout in us
		uint8_t read_thermo_STATUS(uint32_t timeout); //read data from thermometer if available (reads the status register), timeout in us
		virtual uint8_t discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called, timeout in us
	private:
		float _sc_fact_g, _sc_fact_a;	//scale factors
		uint8_t _chipSelectPin, _DRDY_pin_G, _DRDY_pin_A;	//ChipSelectPin and Data Ready pin
		uint8_t _CTRL1_val, _CTRL2_val; //values of the register used when powering up and down the sensor
		uint8_t readRegister(uint8_t chipSelectPin, uint8_t thisRegister);
		void readMultipleRegisters(uint8_t chipSelectPin, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister);
		void writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue);
		uint8_t ch_st (const float val1, const float val2, const float lim1, const float lim2);
};
#endif