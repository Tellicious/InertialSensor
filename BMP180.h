//  BMP180.h
//
//
//  Created by Andrea Vivani on 08/11/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#ifndef BMP180_H_
#define BMP180_H_
#include "InertialSensor.h"
#ifdef INS_ARDUINO
#include <Wire.h>
#endif
//======================================Parameters=============================================//
#define BMP180_DISCARDED_MEASURES	10	//number of measures to be discarded when performing automatic tasks (greater than 1, preferably even)
#define BMP180_DISCARD_TIMEOUT		2e6 //timeout time in us between measures when discarding
//====================Oversampling===================//
#define BMP180_OS_1		0
#define BMP180_OS_2		1	
#define BMP180_OS_4		2	
#define BMP180_OS_8		3	

//================Calibration Data==================//
typedef struct{
			int16_t AC1v;
			int16_t AC2v;
			int16_t AC3v;
			uint16_t AC4v;
			uint16_t  AC5v;
			uint16_t  AC6v;
			int16_t  B1v;
			int16_t  B2v;
			int16_t  MBv;
			int16_t  MCv;
			int16_t  MDv;

			int32_t UT;

			int32_t UP;
			
} bmp180_calibration_data;

class BMP180: public InertialSensor, public BarometerSensor, public ThermometerSensor {
	public:
#ifdef INS_ARDUINO
		BMP180 ();	//constructor
#elif defined(INS_CHIBIOS)
		BMP180(I2CDriver* I2C, I2CConfig* i2ccfg); //constructor
#endif
		virtual void init(); //initializes pins and variables
		float press, temperature; //output data
		uint8_t config_baro(uint8_t oversamp); //configure the barometer
		void read_coefficients(); //read the calibration coefficients from chip memory
		//================Barometer===================//
		virtual void turn_on_baro(){return;}; //turn on the sensor
		virtual void turn_off_baro(){return;}; //turn off the sensor
		virtual inline uint8_t read_baro(uint32_t timeout){return read_baro_STATUS(timeout);}; //default read method from barometer, timeout in us
		uint8_t read_raw_baro(); //read data from barometer
		uint8_t compensate_baro(); //temperature compensated pressure
		uint8_t read_baro_STATUS(uint32_t timeout); //read data from barometer if available (reads the status register), timeout in us
		uint8_t status_baro(); //returns the value of the status register
		virtual uint8_t discard_measures_baro(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called
		//================Temperature=================//
		virtual void turn_on_thermo(){return;}; //turn on the thermometer
		virtual void turn_off_thermo(){return;}; //turn off the thermometer
		virtual inline uint8_t read_thermo(uint32_t timeout){return read_thermo_STATUS(timeout);}; //default read method for thermometer 
		uint8_t read_raw_thermo(); //read data from thermometer registers
		uint8_t read_thermo_STATUS(uint32_t timeout); //read data from thermometer if available (reads the status register), timeout in us
		virtual uint8_t discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called, timeout in us
	private:
		uint8_t _bmp180_read_press_cmd; //pressure reading command
		uint8_t _bmp180OSS; //power mode (oversampling) 
		bmp180_calibration_data _bmp180_calib; //calibration data struct
		int32_t computeB5(int32_t raw_t);
#ifdef INS_CHIBIOS
		I2CDriver* _I2C_int;
		I2CConfig* _i2ccfg;
#endif
};
#endif
/**
 * @brief   Supported modes for the I2C bus.
 */
/*
typedef enum {
  OPMODE_I2C = 1,
  OPMODE_SMBUS_DEVICE = 2,
  OPMODE_SMBUS_HOST = 3,
} i2copmode_t;
*/
/**
 * @brief   Supported duty cycle modes for the I2C bus.
 */
/*
typedef enum {
  STD_DUTY_CYCLE = 1,
  FAST_DUTY_CYCLE_2 = 2,
  FAST_DUTY_CYCLE_16_9 = 3,
} i2cdutycycle_t;
*/
/**
 * @brief   Type of I2C driver configuration structure.
 */
/*
typedef struct {
  // End of the mandatory fields.
  i2copmode_t     op_mode;       // @brief Specifies the I2C mode.
  uint32_t        clock_speed;   // @brief Specifies the clock frequency.
                                 //     @note Must be set to a value lower
                                 //     than 400kHz.
  i2cdutycycle_t  duty_cycle;    // @brief Specifies the I2C fast mode
                                 //   duty cycle.
} I2CConfig; */
