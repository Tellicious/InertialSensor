//  INS_Baro.h
//
//
//  Created by Andrea Vivani on 15/4/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#ifndef INS_Baro_H_
#define INS_Baro_H_
#include "InertialSensor.h"
//======================================Parameters=============================================//
#define INS_BARO_TIMEOUT 2e6	//default timeout when performing calibration
#define INS_BARO_CALIBRATION_MEASURES 20 //default number of measures to be used during calibration
#define INS_BARO_DEFAULT_GROUND 101325.0f //default ground pressure value

class INS_Baro{
public:
	static uint8_t instanced; //number of instanced barometer
	uint8_t instance; //instance number of this barometer
	float press; //output pressure value
	static float invP0; //inversed of ground pressure
	float altitude; //output altitude
	INS_Baro(BarometerSensor &sensor, float &meas); //constructor
	uint8_t read(uint32_t timeout=INS_BARO_TIMEOUT); //read data in hPa, timeout in us
	uint8_t read_altitude(uint32_t timeout); //read altitude in m, timeout in us
	static uint8_t set_ground(float P0 = INS_BARO_DEFAULT_GROUND); //set ground pressure value
	uint8_t auto_ground(uint8_t number_of_measures = INS_BARO_CALIBRATION_MEASURES, uint32_t timeout = INS_BARO_TIMEOUT);	//set current pressure value to be ground reference
	void turn_on(); //turn on the sensor
	void turn_off(); //turn off the sensor
private:
	BarometerSensor &_sens;
	float* _press_p;
};
#endif
