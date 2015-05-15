//  LSM9DS0.h
//
//
//  Created by Andrea Vivani on 28/3/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
//======================================Parameters=============================================//
#define LSM9DS0_ACCEL_SELF_TEST_MEASURES 50 //number of samples to be averaged when performing accelerometer self-test
#define LSM9DS0_DISCARDED_MEASURES	20	//number of measures to be discarded when performing automatic tasks (greater than 1, preferably even)
#define LSM9DS0_DISCARDED_MEASURES_ST 100 //number of measures to be discarded after performing self-tests (greater than 1, preferably even)
#define LSM9DS0_DISCARD_TIMEOUT		2e6 //timeout time in us between measures when discarding
//======================================ODR Values Gyro=============================================//
// 4 LSB values: CTRL1_G as DR 1:0 and BW 1:0
#define LSM9DS0_ODR_G_95_C0_12_5	0x00
#define LSM9DS0_ODR_G_95_C0_25		0x03
#define LSM9DS0_ODR_G_190_C0_12_5	0x04
#define LSM9DS0_ODR_G_190_C0_25		0x05
#define LSM9DS0_ODR_G_190_C0_50		0x06
#define LSM9DS0_ODR_G_190_C0_70		0x07
#define LSM9DS0_ODR_G_380_C0_20		0x08
#define LSM9DS0_ODR_G_380_C0_25		0x09
#define LSM9DS0_ODR_G_380_C0_50		0x0A
#define LSM9DS0_ODR_G_380_C0_100	0x0B
#define LSM9DS0_ODR_G_760_C0_30		0x0C
#define LSM9DS0_ODR_G_760_C0_35		0x0D
#define LSM9DS0_ODR_G_760_C0_50		0x0E
#define LSM9DS0_ODR_G_760_C0_100	0x0F
//====================================Range Values Gyro==============================================//
#define LSM9DS0_RANGE_G_245			0x00
#define LSM9DS0_RANGE_G_500			0x10
#define LSM9DS0_RANGE_G_2000		0x20
//======================================ODR Values Accel=============================================//
// 4 LSB values: CTRL1_XM as A0DR 3:0
#define LSM9DS0_A_PDOWN				0x00
#define LSM9DS0_ODR_A_3_125			0x01
#define LSM9DS0_ODR_A_6_25			0x02
#define LSM9DS0_ODR_A_12_5			0x03
#define LSM9DS0_ODR_A_25			0x04
#define LSM9DS0_ODR_A_50			0x05
#define LSM9DS0_ODR_A_100			0x06
#define LSM9DS0_ODR_A_200			0x07
#define LSM9DS0_ODR_A_400			0x08
#define LSM9DS0_ODR_A_800			0x09
#define LSM9DS0_ODR_A_1600			0x0A
//=====================================Anti-Alias Accel==============================================//
// 2 LSB values: CTRL2_XM as ABW 1:0
#define LSM9DS0_BW_A_50				0x03
#define LSM9DS0_BW_A_362			0x02
#define LSM9DS0_BW_A_194			0x01
#define LSM9DS0_BW_A_773			0x00
//====================================Range Values Accel=============================================//
// 3 LSB values: CTRL2_XM as AFS 2:0
#define LSM9DS0_RANGE_A_2			0x00
#define LSM9DS0_RANGE_A_4			0x01
#define LSM9DS0_RANGE_A_6			0x02
#define LSM9DS0_RANGE_A_8			0x03
#define LSM9DS0_RANGE_A_16			0x04
//=======================================ODR Values Mag==============================================//
// 3 LSB values: CTRL5_XM as M_ODR 2:0
#define LSM9DS0_ODR_M_3_125			0x00
#define LSM9DS0_ODR_M_6_25			0x01
#define LSM9DS0_ODR_M_12_5			0x02
#define LSM9DS0_ODR_M_25			0x03
#define LSM9DS0_ODR_M_50			0x04
#define LSM9DS0_ODR_M_100			0x05
//=====================================Range Values Mag==============================================//
// 2 LSB values: CTRL6_XM as MFS 1:0
#define LSM9DS0_RANGE_M_2			0x00
#define LSM9DS0_RANGE_M_4			0x01
#define LSM9DS0_RANGE_M_8			0x02
#define LSM9DS0_RANGE_M_12			0x03

#ifndef LSM9DS0_H_
#define LSM9DS0_H_
#include "InertialSensor.h"
#include <SPI.h>

class LSM9DS0: public InertialSensor{
	public:
		LSM9DS0 (uint8_t CS_pin_G, uint8_t CS_pin_XM);	//constructor
		LSM9DS0 (uint8_t CS_pin_G, uint8_t CS_pin_XM, uint8_t DRDY_pin_G, uint8_t DRDY_pin_A, uint8_t DRDY_pin_M);	//constructor with Data ready pin
		virtual void init(); //initializes pins, SPI and variables
		float gx,gy,gz;	//gyroscope output data
		float ax,ay,az;	//accelerometer output data
		float mx,my,mz;	//magnetometer output data
		float temperature; //temperature value
		//==================Gyroscope==================//
		uint8_t config_gyro(uint8_t gyro_range, uint8_t gyro_odr, uint8_t LPF2_enable, uint8_t HP_enable, uint8_t HP_freq); //configure the gyroscope
		virtual void turn_on_gyro(); //turn on the gyroscope
		virtual void turn_off_gyro(); //turn off the gyroscope
		virtual void sleep_gyro(); //put the gyroscope to sleep
		virtual inline uint8_t read_gyro(uint32_t timeout){return read_gyro_STATUS(timeout);}; //default read method from gyroscope, timeout in us
		uint8_t read_raw_gyro(); //read data from gyroscope registers
		uint8_t read_gyro_DRDY(uint32_t timeout); //read data from gyroscope if DRDY is high, timeout in us
		uint8_t read_gyro_STATUS(uint32_t timeout); //read data from gyroscope if available (reads the status register). timeout in us
		virtual uint8_t check_gyro_biases(float bx, float by, float bz); //check if gyro biases are within reasonable limits, according to datasheet
		void HP_reset_gyro(); //resets the High-Pass filter
		uint8_t self_test_gyro(uint8_t mode); //self-test: mode 0 - X positive, Y, Z negative, mode 1 - X negative, Y, Z positive
		uint8_t status_gyro(); //returns the value of the status register
		virtual uint8_t discard_measures_gyro(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called, timeout in us
		//===============Accelerometer================//
		uint8_t config_accel_mag(uint8_t accel_range, uint8_t accel_odr, uint8_t accel_bw, uint8_t mag_range, uint8_t mag_odr,uint8_t HP_accel_enable); //configure the accelerometer and the magnetometer
		virtual void turn_on_accel(); //turn on the accelerometer
		virtual void turn_off_accel(); //turn off the accelerometer
		virtual inline uint8_t read_accel(uint32_t timeout){return read_accel_STATUS(timeout);}; //default read method for accelerometer, timeout in us
		uint8_t read_raw_accel(); //read data from accelerometer registers
		uint8_t read_accel_DRDY(uint32_t timeout); //read data from accelerometer if DRDY is high, timeout in us
		uint8_t read_accel_STATUS(uint32_t timeout); //read data from gyroscope if available (reads the status register), timeout in us
		virtual uint8_t check_accel_biases(float bx, float by, float bz); //check if accel biases are within reasonable limits, according to datasheet
		void HP_reset_accel(); //resets the High-Pass filter
		uint8_t self_test_accel(uint8_t mode); //self-test: mode 0 - X, Y, Z positive, mode 1 - X, Y, Z negative (TO BE PERFORMED WITHOUT HP FILTER!!!)
		uint8_t status_accel(); //returns the value of the status register
		virtual uint8_t discard_measures_accel(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called. timeout in us
		//===============Magnetometer=================//
		virtual void turn_on_mag(); //turn on the magnetometer
		virtual void turn_off_mag(); //turn off the magnetometer
		virtual inline uint8_t read_mag(uint32_t timeout){return read_mag_STATUS(timeout);}; //default read method for magnetometer, timeout in us
		uint8_t read_raw_mag(); //read data from magnetometer registers
		uint8_t read_mag_DRDY(uint32_t timeout); //read data from magnetometer if DRDY is high, timeout in us
		uint8_t read_mag_STATUS(uint32_t timeout); //read data from gyroscope if available (reads the status register), timeout in us
		uint8_t status_mag(); //returns the value of the status register
		virtual uint8_t discard_measures_mag(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called, timeout in us
		//================Temperature=================//
		virtual void turn_on_thermo(){return turn_on_mag();}; //turn on the thermometer
		virtual void turn_off_thermo(){return;}; //turn off the thermometer
		virtual inline uint8_t read_thermo(uint32_t timeout){return read_raw_thermo();}; //default read method for thermometer 
		uint8_t read_raw_thermo(); //read data from thermometer registers
		uint8_t read_thermo_DRDY(uint32_t timeout); //read temperature when DRDY is high (same INT2 and ODR as magnetometer), timeout in us
		uint8_t read_thermo_STATUS(uint32_t timeout); //read data from gyroscope if available (reads the status register). timeout in us
		virtual uint8_t discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called, timeout in us
	private:
		float _sc_fact_g, _sc_fact_a, _sc_fact_m;		//scale factors
		uint8_t _chipSelectPin_G, _chipSelectPin_XM, _DRDY_pin_G, _DRDY_pin_A, _DRDY_pin_M;	//ChipSelectPin and Data Ready pin
		uint8_t _CTRL1_val_G, _CTRL1_val_XM, _CTRL7_val_XM; //values of the register used when powering up and down the sensor
		//uint8_t _mySPCR; //value of the SPI configuration register for this IC
		uint8_t readRegister(uint8_t chipSelectPin, uint8_t thisRegister);
		void readMultipleRegisters(uint8_t chipSelectPin, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister);
		void writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue);
};
#endif