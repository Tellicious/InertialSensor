//  INS_Thermo.h
//
//
//  Created by Andrea Vivani on 15/4/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#ifndef INS_Thermo_H_
#define INS_Thermo_H_
#include "InertialSensor.h"
//======================================Parameters=============================================//
#define INS_THERMO_TIMEOUT 2e6	//default timeout when performing calibration

class INS_Thermo{
public:
	static uint8_t instanced; //number of instanced thermometer
	uint8_t instance; //instance number of this thermometer
	float temp; //output values
	float bt; //thermometer bias (if any)
	INS_Thermo(ThermometerSensor &sensor, float &meas); //constructor
	uint8_t read(uint32_t timeout=INS_THERMO_TIMEOUT); //read data, timeout in us
	void turn_on(); //turn on the sensor
	void turn_off(); //turn off the sensor
private:
	ThermometerSensor &_sens;
	float* _temp_p;
};
#endif
