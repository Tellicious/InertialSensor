//  L3GD20H.h
//
//
//  Created by Andrea Vivani on 23/2/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#ifndef L3GD20H_H_
#define L3GD20H_H_
#include "InertialSensor.h"
#ifdef INS_ARDUINO
#include <SPI.h>
#endif
//======================================Parameters=============================================//
#define L3GD20H_GYRO_SELF_TEST_MEASURES 10 	//number of samples to be averaged when performing gyro self-test
#define L3GD20H_DISCARDED_MEASURES		5	//number of measures to be discarded when performing automatic tasks (greater than 1, preferably even)
#define L3GD20H_DISCARDED_MEASURES_ST 	10 	//number of measures to be discarded after performing self tests (greater than 1, preferably even)
#define L3GD20H_DISCARD_TIMEOUT			2e6 //timeout time in us between measures when discarding
//======================================ODR Values=============================================//
// 5 LSB values: the first one has to be set in the LOW_ODR register, the remaining 4 in
// CTRL1 as DR 1:0 and BW 1:0
#define L3GD20H_ODR_12_5_CO_n		0x10
#define L3GD20H_ODR_25_CO_n			0x14
#define L3GD20H_ODR_50_CO_16_6		0x18
#define L3GD20H_ODR_100_CO_12_5		0x00
#define L3GD20H_ODR_100_CO_25		0x03
#define L3GD20H_ODR_200_CO_12_5		0x04
#define L3GD20H_ODR_200_CO_n		0x05
#define L3GD20H_ODR_200_CO_70		0x07
#define L3GD20H_ODR_400_CO_20		0x08
#define L3GD20H_ODR_400_CO_25		0x09
#define L3GD20H_ODR_400_CO_50		0x0A
#define L3GD20H_ODR_400_CO_110		0x0B
#define L3GD20H_ODR_800_CO_30		0x0C
#define L3GD20H_ODR_800_CO_35		0x0D
#define L3GD20H_ODR_800_CO_n		0x0E
#define L3GD20H_ODR_800_CO_100		0x0F
//======================================Range Values=============================================//
#define L3GD20H_RANGE_245			0x00
#define L3GD20H_RANGE_500			0x10
#define L3GD20H_RANGE_2000			0x20

class L3GD20H: public InertialSensor {
	public:
#ifdef INS_ARDUINO
		L3GD20H (uint8_t CS_pin);	//constructor
		L3GD20H (uint8_t CS_pin, uint8_t DRDY_pin);	//constructor with Data ready pin
#elif INS_CHIBIOS
		L3GD20H (SPIDriver* SPI, ioportid_t gpio_CS_CS, uin8_t CS_pin);	//constructor
		L3GD20H (SPIDriver* SPI, ioportid_t gpio_CS, uin8_t CS_pin, ioportid_t gpio_DRDY, uint8_t DRDY_pin);	//constructor stm32_gpio_t*
#endif
		virtual void init(); //initializes pins and variables
		float x, y, z;	//output data
		uint8_t config_gyro(uint8_t range_conf, uint8_t odr_conf, uint8_t LPF2_enable, uint8_t HP_enable, uint8_t HP_freq); //configure the gyroscope
		virtual void turn_on_gyro(); //turn on the sensor
		virtual void turn_off_gyro(); //turn off the sensor
		virtual void sleep_gyro(); //put the sensor to sleep
		virtual inline uint8_t read_gyro(uint32_t timeout){return read_gyro_STATUS(timeout);}; //default read method from gyroscope, timeout in us
		uint8_t read_raw_gyro(); //read data from gyroscope
		uint8_t read_gyro_DRDY(uint32_t timeout); //read data from gyroscope if DRDY is high. timeout in us
		uint8_t read_gyro_STATUS(uint32_t timeout); //read data from gyroscope if available (reads the status register). timeout in us
		virtual uint8_t check_gyro_biases(float bx, float by, float bz); //check if gyro biases are within reasonable limits, according to datasheet
		void HP_reset_gyro(); //resets the High-Pass filter
		uint8_t self_test_gyro(uint8_t mode); //self-test: mode 0 - X positive, Y negative, Z positive, mode 1 - X negative, Y positive, Z negative
		uint8_t status_gyro(); //returns the value of the status register
		virtual uint8_t discard_measures_gyro(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called. timeout in us
	private:
		float _sc_fact;		//scale factor
		uint8_t _chipSelectPin, _DRDY_pin;	//ChipSelectPin and Data Ready pin
#ifdef INS_CHIBIOS
		SPIDriver* _SPI_int;
		ioportid_t _gpio_CS;
		ioportid_t _gpio_DRDY;
		SPIConfig _spicfg;
#endif
		uint8_t _CTRL1_val; //value of the register, used when powering up and down the sensor
		uint8_t readRegister(uint8_t chipSelectPin, uint8_t thisRegister);
		void readMultipleRegisters(uint8_t chipSelectPin, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister);
		void writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue);
		uint8_t ch_st (const float val1, const float val2, const float lim1, const float lim2);
};
#endif
