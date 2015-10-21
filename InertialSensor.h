//  InertialSensor.h
//
//
//  Created by Andrea Vivani on 15/4/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#ifndef INERTIAL_SENSOR_H_
#define INERTIAL_SENSOR_H_
#include "Arduino.h"
//======================================Parameters=============================================//
#define INS_G_VAL 		9.81	//magnitude of static accelerometer reading (the sensor is calibrated to return that value) in m/s^2
#define INS_MAG_VAL		1.0		//magnitude of magnetometer reading, if 0 magnetometer is calibrated to return the value directly coming from the sensor in Gauss
#define INS_TORAD		0.01745329251994329576923690768488
#define INS_TODEG		57.29577951308232087679815481410517
typedef enum{
	X_FRONT_Z_UP,
	X_LEFT_Z_UP,
	X_BACK_Z_UP,
	X_RIGHT_Z_UP,
	X_FRONT_Z_DOWN,
	X_LEFT_Z_DOWN,
	X_BACK_Z_DOWN,
	X_RIGHT_Z_DOWN,
	X_FRONT_Z_UP_LH,
	X_LEFT_Z_UP_LH,
	X_BACK_Z_UP_LH,
	X_RIGHT_Z_UP_LH,
	X_FRONT_Z_DOWN_LH,
	X_LEFT_Z_DOWN_LH,
	X_BACK_Z_DOWN_LH,
	X_RIGHT_Z_DOWN_LH
} INS_orientation;

class InertialSensor{
	public:
		InertialSensor(){};
		virtual void init(){return;}; //initializes pins, bus and variables
		//==================Gyroscope==================//
		virtual void turn_on_gyro(){return;}; //turn on the gyroscope
		virtual void turn_off_gyro(){return;}; //turn off the gyroscope
		virtual void sleep_gyro(){return;}; //put the gyroscope to sleep
		virtual inline uint8_t read_gyro(uint32_t timeout){return 0;}; //read data from gyroscope, timeout in us, output in rad/s
		virtual uint8_t check_gyro_biases(float bx, float by, float bz){return 0;}; //check if gyro biases are within reasonable limits
		virtual uint8_t discard_measures_gyro(uint8_t number_of_measures, uint32_t timeout){return 0;}; //discards the first n measures after being called, timeout in us
		//================Accelerometer================//
		virtual void turn_on_accel(){return;}; //turn on the accelerometer
		virtual void turn_off_accel(){return;}; //turn off the accelerometer
		virtual inline uint8_t read_accel(uint32_t timeout){return 0;}; //read data from accelerometer, output in m/s^2
		virtual uint8_t check_accel_biases(float bx, float by, float bz){return 0;}; //check if accelerometer biases are within reasonable limits
		virtual uint8_t discard_measures_accel(uint8_t number_of_measures, uint32_t timeout){return 0;}; //discards the first n measures after being called, timeout in us
		//=================Magnetometer================//
		virtual void turn_on_mag(){return;}; //turn on the magnetometer
		virtual void turn_off_mag(){return;}; //turn off the magnetometer
		virtual inline uint8_t read_mag(uint32_t timeout){return 0;}; //read data from magnetometer, timeout in us, output in Gauss
		virtual uint8_t discard_measures_mag(uint8_t number_of_measures, uint32_t timeout){return 0;}; //discards the first n measures after being called, timeout in us
		//==================Barometer=================//
		virtual void turn_on_baro(){return;}; //turn on the barometer
		virtual void turn_off_baro(){return;}; //turn off the barometer
		virtual inline uint8_t read_baro(uint32_t timeout){return 0;}; //read data from barometer, timeout in us, output in hPa
		virtual uint8_t discard_measures_baro(uint8_t number_of_measures, uint32_t timeout){return 0;}; //discards the first n measures after being called, timeout in us
		//=================Thermometer================//
		virtual void turn_on_thermo(){return;}; //turn on the temperature sensor
		virtual void turn_off_thermo(){return;}; //turn off the temperature sensor
		virtual inline uint8_t read_thermo(uint32_t timeout){return 0;}; //read data from temperature sensor, timeout in us, output in Celsius degrees
		virtual uint8_t discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){return 0;}; //discards the first n measures after being called, timeout in us
};
//==================Include Sensors Libraries==================//
#include "LSM6DS3.h"
#include "LSM9DS0.h"
#include "LSM9DS1.h"
#include "L3GD20H.h"
#include "HMC5983.h"
#include "LIS3MDL.h"
#include "LPS25HB.h"
//==================Include Generic Libraries==================//
#include "INS_Gyro.h"
#include "INS_Accel.h"
#include "INS_Mag.h"
#include "INS_Thermo.h"
#include "INS_Baro.h"
#endif