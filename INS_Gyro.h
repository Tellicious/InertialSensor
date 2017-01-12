//  INS_Gyro.h
//
//
//  Created by Andrea Vivani on 15/4/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#ifndef INS_Gyro_H_
#define INS_Gyro_H_
#include "InertialSensor.h"
//======================================Parameters=============================================//
#define INS_GYRO_AVERAGED_MEASURES 200.0f //default number of measures used to calibrate the sensor
#define INS_GYRO_DISCARDED_MEASURES	20	//number of measures to be discarded
#define INS_GYRO_TIMEOUT 2e6	//default timeout when performing calibration

class INS_Gyro{
public:
	static uint8_t instanced; //number of instanced gyroscopes
	uint8_t instance; //instance number of this gyroscope
	float x, y, z; //output values
	float bx, by, bz; //gyroscope biases
	INS_Gyro(GyroscopeSensor &sensor, float &meas_x, float &meas_y, float &meas_z, INS_orientation orientation); //constructor
	uint8_t read(uint32_t timeout=INS_GYRO_TIMEOUT); //read data, timeout in us
	void turn_on(); //turn on the sensor
	void turn_off(); //turn off the sensor
	void sleep(); //put the sensor to sleep
	uint8_t calibrate(uint8_t number_of_measures=INS_GYRO_AVERAGED_MEASURES, uint32_t timeout=INS_GYRO_TIMEOUT); //calibrate the gyroscope by removing biases
private:
	GyroscopeSensor &_sens;
	float* _x_p;
	float* _y_p;
	float* _z_p; //pointers to sensor output values
	int8_t _sx, _sy, _sz; //factors to rotate readings in order to provide always X front, Y right and Z down
};
#endif
