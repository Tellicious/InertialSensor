//  INS_Thermo.cpp
//
//
//  Created by Andrea Vivani on 15/4/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#include "INS_Thermo.h"
uint8_t INS_Thermo::instanced = 0;

//=====================================Constructor==========================================//
INS_Thermo::INS_Thermo(ThermometerSensor &sensor, float &meas):_sens(sensor){
	_temp_p = &meas;
	temp = *_temp_p;
	instanced++;
	instance = instanced;
	bt = 0;
}

//====================================Public Members=========================================//
//---------------------------Read-----------------------------//
uint8_t INS_Thermo::read(uint32_t timeout){
	uint8_t res = _sens.read_thermo(timeout);
	temp = (*_temp_p) - bt;
	return res;
}

//-------------------------Turn on---------------------------//
void INS_Thermo::turn_on(){
	_sens.turn_on_thermo();
	return;
}

//------------------------Turn off---------------------------//
void INS_Thermo::turn_off(){
	_sens.turn_off_thermo();
	return;
}
