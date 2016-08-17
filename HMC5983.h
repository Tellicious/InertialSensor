//  HMC5983.h
//
//
//  Created by Andrea Vivani on 21/5/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#ifndef HMC5983_H_
#define HMC5983_H_
#include "InertialSensor.h"
#ifdef INS_ARDUINO
#include <SPI.h>
#endif
//======================================Parameters=============================================//
#define HMC5983_DISCARDED_MEASURES		5	//number of measures to be discarded when performing automatic tasks (greater than 1, preferably even)
#define HMC5983_DISCARDED_MEASURES_ST 	10 	//number of measures to be discarded after performing self tests (greater than 1, preferably even)
#define HMC5983_DISCARD_TIMEOUT			2e6 //timeout time in us between measures when discarding
//=======================================ODR Values==============================================//
// Config register A
#define HMC5983_ODR_0_75	0x00
#define HMC5983_ODR_1_5		0x04
#define HMC5983_ODR_3		0x08
#define HMC5983_ODR_7_5		0x0C
#define HMC5983_ODR_15		0x10
#define HMC5983_ODR_30		0x14
#define HMC5983_ODR_75		0x18
#define HMC5983_ODR_220		0x1C
//=======================================Samples Averaged==============================================//
// Config register A
#define HMC5983_AVERAGE_1	0x00
#define HMC5983_AVERAGE_2	0x20
#define HMC5983_AVERAGE_4	0x40
#define HMC5983_AVERAGE_8	0x60
//=====================================Measurement Mode==============================================//
// Config register A
#define HMC5983_CONTINUOUS	0x00
#define HMC5983_SINGLE		0x01
//=====================================Range Values Mag==============================================//
// Config register B
#define HMC5983_RANGE_0_88	0x00
#define HMC5983_RANGE_1_3	0x20
#define HMC5983_RANGE_1_9	0x40
#define HMC5983_RANGE_2_5	0x60
#define HMC5983_RANGE_4		0x80
#define HMC5983_RANGE_4_7	0xA0
#define HMC5983_RANGE_5_6	0xC0
#define HMC5983_RANGE_8_1	0xE0

class HMC5983: public InertialSensor {
	public:
#ifdef INS_ARDUINO
		HMC5983 (uint8_t CS_pin);	//constructor
		HMC5983 (uint8_t CS_pin, uint8_t DRDY_pin);	//constructor with Data ready pin
#elif INS_CHIBIOS
		HMC5983 (SPIDriver* SPI, ioportid_t gpio_CS, uin8_t CS_pin);	//constructor
		HMC5983 (SPIDriver* SPI, ioportid_t gpio_CS, uin8_t CS_pin, ioportid_t gpio_DRDY, uint8_t DRDY_pin);	//constructor stm32_gpio_t*
#endif
		virtual void init(); //initializes pins and variables
		float x, y, z;	//output data
		float temperature;
		uint8_t config_mag(uint8_t range_conf, uint8_t odr_conf, uint8_t average_conf, uint8_t meas_mode); //configure the magnetometer
		//===============Magnetometer=================//
		virtual void turn_on_mag(); //turn on the magnetometer
		virtual void turn_off_mag(); //turn off the magnetometer
		void sleep_mag(); //put the sensor to sleep
		virtual inline uint8_t read_mag(uint32_t timeout){return read_mag_STATUS(timeout);}; //default read method for magnetometer, timeout in us
		uint8_t read_raw_mag(); //read data from magnetometer registers
		uint8_t read_mag_DRDY(uint32_t timeout); //read data from magnetometer if DRDY is high, timeout in us
		uint8_t read_mag_STATUS(uint32_t timeout); //read data from magnetometer if available (reads the status register), timeout in us
		uint8_t read_mag_single_DRDY(uint32_t timeout); //read data from magnetometer if DRDY is high, when in single read mode, timeout in us
		uint8_t read_mag_single_STATUS(uint32_t timeout); //read data from magnetometer if available (reads the status register), when in single read mode, timeout in us
		uint8_t self_test_mag(uint8_t mode); //self-test: mode 0 - X, Y, Z positive, mode 1 - X, Y, Z negative
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
#ifdef INS_CHIBIOS
		SPIDriver* _SPI_int;
		ioportid_t _gpio_CS;
		ioportid_t _gpio_DRDY;
		SPIConfig _spicfg;
#endif
		uint8_t _MODE_val; //value of the register, used when powering up and down the sensor
		uint8_t readRegister(uint8_t chipSelectPin, uint8_t thisRegister);
		void readMultipleRegisters(uint8_t chipSelectPin, uint8_t* buffer, uint8_t number_of_registers, uint8_t startRegister);
		void writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue);
		uint8_t ch_st (const float val1, const float val2, const float lim1, const float lim2);
};
#endif