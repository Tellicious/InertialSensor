//  LPS25HB.h
//
//
//  Created by Andrea Vivani on 20/6/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#ifndef LPS25HB_H_
#define LPS25HB_H_
#include "InertialSensor.h"
#ifdef INS_ARDUINO
#include <SPI.h>
#endif
//======================================Parameters=============================================//
#define LPS25HB_DISCARDED_MEASURES	10	//number of measures to be discarded when performing automatic tasks (greater than 1, preferably even)
#define LPS25HB_DISCARD_TIMEOUT		2e6 //timeout time in us between measures when discarding
//======================================ODR Values=============================================//
// CTRL1 value
#define LPS25HB_ODR_ONE_SHOT		0x00
#define LPS25HB_ODR_1				0x10
#define LPS25HB_ODR_7				0x20
#define LPS25HB_ODR_12_5			0x30
#define LPS25HB_ODR_25				0x40
//======================================Average number=============================================//
#define LPS25HB_AVP_8				0x00
#define LPS25HB_AVP_32				0x01
#define LPS25HB_AVP_128				0x02
#define LPS25HB_AVP_512				0x03
#define LPS25HB_AVT_8				0x00
#define LPS25HB_AVT_16				0x04
#define LPS25HB_AVT_32				0x08
#define LPS25HB_AVT_64				0x0C
//======================================Moving average number=============================================//
#define LPS25HB_NO_MA				0x00
#define	LPS25HB_MA_2				0x01
#define LPS25HB_MA_4				0x03
#define LPS25HB_MA_8				0x07
#define LPS25HB_MA_16				0x0F
#define LPS25HB_MA_32				0x1F

class LPS25HB: public InertialSensor, public BarometerSensor, public ThermometerSensor {
	public:
#ifdef INS_ARDUINO
		LPS25HB (uint8_t CS_pin);	//constructor
		LPS25HB (uint8_t CS_pin, uint8_t DRDY_pin);	//constructor with Data ready pin
#elif defined(INS_CHIBIOS)
		LPS25HB (SPIDriver* SPI, SPIConfig* spicfg);	//constructor
		LPS25HB (SPIDriver* SPI, SPIConfig* spicfg, ioportid_t gpio_DRDY, uint8_t DRDY_pin);	//constructor
#endif
		virtual void init(); //initializes pins and variables
		float press, temperature; //output data
		uint8_t config_baro(uint8_t odr_conf, uint8_t AVGT, uint8_t AVGP, uint8_t MA_FIFO); //configure the barometer
		//================Barometer===================//
		virtual void turn_on_baro(); //turn on the sensor
		virtual void turn_off_baro(); //turn off the sensor
		virtual inline uint8_t read_baro(uint32_t timeout){return read_baro_STATUS(timeout);}; //default read method from barometer, timeout in us
		uint8_t read_raw_baro(); //read data from barometer
		uint8_t read_baro_DRDY(uint32_t timeout); //read data from barometer if DRDY is high. timeout in us
		uint8_t read_baro_STATUS(uint32_t timeout); //read data from barometer if available (reads the status register). timeout in us
		uint8_t status_baro(); //returns the value of the status register
		virtual uint8_t discard_measures_baro(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called. timeout in us
		//================Temperature=================//
		virtual void turn_on_thermo(){return turn_on_baro();}; //turn on the thermometer
		virtual void turn_off_thermo(){return;}; //turn off the thermometer
		virtual inline uint8_t read_thermo(uint32_t timeout){return read_thermo_STATUS(timeout);}; //default read method for thermometer 
		uint8_t read_raw_thermo(); //read data from thermometer registers
		uint8_t read_thermo_DRDY(uint32_t timeout); //read temperature when DRDY is high (same INT2 and ODR as magnetometer), timeout in us
		uint8_t read_thermo_STATUS(uint32_t timeout); //read data from thermometer if available (reads the status register), timeout in us
		virtual uint8_t discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called, timeout in us
	private:
		float _sc_fact = 1.0f / 4096.0f;		//scale factor
		uint8_t _DRDY_pin;	//Data Ready pin
		uint8_t _CTRL1_val; //value of the register, used when powering up and down the sensor
#ifdef INS_ARDUINO
		uint8_t _chipSelectPin;
#elif defined(INS_CHIBIOS)
		SPIDriver* _SPI_int;
        SPIConfig* _spicfg;
		ioportid_t _gpio_DRDY;
#endif
};
#endif
