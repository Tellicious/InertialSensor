//  BMP280.h
//
//
//  Created by Andrea Vivani on 01/11/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#ifndef BMP280_H_
#define BMP280_H_
#include "InertialSensor.h"
#ifdef INS_ARDUINO
#include <SPI.h>
#endif
//======================================Parameters=============================================//
#define BMP280_DISCARDED_MEASURES	10	//number of measures to be discarded when performing automatic tasks (greater than 1, preferably even)
#define BMP280_DISCARD_TIMEOUT		2e6 //timeout time in us between measures when discarding
//===============Pressure Oversampling=============//
#define BMP280_P_OS_0 	0x00
#define BMP280_P_OS_1	0x04
#define BMP280_P_OS_2	0x08	
#define BMP280_P_OS_4	0x0C	
#define BMP280_P_OS_8	0x10	
#define BMP280_P_OS_16	0x14
//==============Temperature Oversampling=============//
#define BMP280_T_OS_0 	0x00
#define BMP280_T_OS_1	0x20
#define BMP280_T_OS_2	0x40	
#define BMP280_T_OS_4	0x60	
#define BMP280_T_OS_8	0x80	
#define BMP280_T_OS_16	0xA0
//===============IIR Filter Coefficient==============//
#define BMP280_FILT_OFF 0X00
#define BMP280_FILT_2	0X04
#define BMP280_FILT_4	0X08	
#define BMP280_FILT_8	0X0C	
#define BMP280_FILT_16	0X10
//===================Standby Time====================//
#define BMP280_SB_0_5 	0X00
#define BMP280_SB_62_5	0X20
#define BMP280_SB_125	0X40	
#define BMP280_SB_250	0X60	
#define BMP280_SB_500	0X80
#define BMP280_SB_1000	0XA0
#define BMP280_SB_2000	0XC0
#define BMP280_SB_4000	0XE0
//======================Mode========================//
#define BMP280_NORMAL_MODE 	0X03
#define BMP280_FORCED_MODE 	0X01

//================Calibration Data==================//
typedef struct{
			uint16_t dig_T1;
			int16_t dig_T2;
			int16_t dig_T3;

			uint16_t dig_P1;
			int16_t  dig_P2;
			int16_t  dig_P3;
			int16_t  dig_P4;
			int16_t  dig_P5;
			int16_t  dig_P6;
			int16_t  dig_P7;
			int16_t  dig_P8;
			int16_t  dig_P9;

			int32_t t_fine;

			int64_t p;
			
} bmp280_calibration_data;

class BMP280: public InertialSensor, public BarometerSensor, public ThermometerSensor {
	public:
	#ifdef INS_ARDUINO
		BMP280 (uint8_t CS_pin);	//constructor
	#elif defined(INS_CHIBIOS)
		BMP280 (SPIDriver* SPI, SPIConfig* SPIcfg);	//constructor
	#endif
		virtual void init(); //initializes pins and variables
		float press, temperature; //output data
		uint8_t config_baro(uint8_t standy_time, uint8_t mode, uint8_t press_oversamp, uint8_t temp_oversamp, uint8_t IIR_coeff); //configure the barometer
		void read_coefficients(); //read the calibration coefficients from chip memory
		//================Barometer===================//
		virtual void turn_on_baro(); //turn on the sensor
		virtual void turn_off_baro(); //turn off the sensor
		virtual inline uint8_t read_baro(uint32_t timeout){return read_baro_STATUS(timeout);}; //default read method from barometer, timeout in us
		uint8_t read_raw_baro(); //read data from barometer
		uint8_t read_raw_baro_32(); //read data from barometer (32 bit compensation formula)
		uint8_t read_baro_compensated(); //temperature compensated pressure
		uint8_t read_baro_STATUS(uint32_t timeout); //read data from barometer if available (reads the status register). timeout in us
		uint8_t read_single(uint32_t timeout); //one-shot reading (with temperature compensation)
		uint8_t status_baro(); //returns the value of the status register
		virtual uint8_t discard_measures_baro(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called. timeout in us
		//================Temperature=================//
		virtual void turn_on_thermo(){return turn_on_baro();}; //turn on the thermometer
		virtual void turn_off_thermo(){return;}; //turn off the thermometer
		virtual inline uint8_t read_thermo(uint32_t timeout){return read_thermo_STATUS(timeout);}; //default read method for thermometer 
		uint8_t read_raw_thermo(); //read data from thermometer registers
		uint8_t read_thermo_STATUS(uint32_t timeout); //read data from thermometer if available (reads the status register), timeout in us
		virtual uint8_t discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout); //discards the first n measures after being called, timeout in us
	private:
	#ifdef INS_ARDUINO
		uint8_t _chipSelectPin;	//ChipSelectPin
	#elif defined(INS_CHIBIOS)
		SPIDriver* _SPI_int;
		SPIConfig* _spicfg;
	#endif
		uint8_t _CTRL_MEAS_val; //value of the register, used when powering up and down the sensor
		bmp280_calibration_data _bmp280_calib; //calibration data struct
		void baro_cal_64_bit(int32_t adc_P); //calibrate pressure reading using 64 bit formula
		void baro_cal_32_bit(int32_t adc_P); //calibrate pressure reading using 32 bit formula
		void thermo_cal(int32_t adc_T); //calibrate temperature reading
};
#endif
