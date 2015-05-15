//  INS_Baro.cpp
//
//
//  Created by Andrea Vivani on 15/4/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#include "INS_Baro.h"
uint8_t INS_Baro::instanced=0;
float INS_Baro::invP0=1.0/INS_BARO_DEFAULT_GROUND;

//=====================================Constructor==========================================//
INS_Baro::INS_Baro(InertialSensor &sensor, float &meas):_sens(sensor){
	_press_p=&meas;
	press=*_press_p;
	invP0=1.0/INS_BARO_DEFAULT_GROUND;
	instanced++;
	instance=instanced;
}

//====================================Public Members=========================================//
//---------------------------Read-----------------------------//
uint8_t INS_Baro::read(uint32_t timeout){
	uint8_t res=_sens.read_baro(timeout);
	press=*_press_p;
	return res;
}

//----------------------Read altitude-------------------------//
uint8_t INS_Baro::read_altitude(uint32_t timeout){
	if(_sens.read_baro(timeout)){
		press=*_press_p;
		altitude=44330.76067152237*(1-pow((press*invP0), 0.190262371810727));
		return 1;
	}
	return 0;
}

//-------------------Set ground pressure----------------------//
uint8_t INS_Baro::set_ground(float P0){
	if(P0){
		invP0=1.0/P0;
		return 1;
	}
	return 0;
}

//----------------Auto set ground pressure-------------------//
uint8_t INS_Baro::auto_ground(uint8_t number_of_measures, uint32_t timeout){
	float p0_tmp=0;
	for (uint8_t ii=1;ii<number_of_measures;ii++){
		if (!read(timeout)){
			return 0;
		}
		p0_tmp+=press;
	}
	invP0=number_of_measures/p0_tmp;
}

//-------------------------Turn on---------------------------//
void INS_Baro::turn_on(){
	_sens.turn_on_baro();
	return;
}

//------------------------Turn off---------------------------//
void INS_Baro::turn_off(){
	_sens.turn_off_baro();
	return;
}