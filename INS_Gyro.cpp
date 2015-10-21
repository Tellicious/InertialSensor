//  INS_Gyro.cpp
//
//
//  Created by Andrea Vivani on 15/4/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#include "INS_Gyro.h"
uint8_t INS_Gyro::instanced=0;

//=====================================Constructor==========================================//
INS_Gyro::INS_Gyro(InertialSensor &sensor, float &meas_x, float &meas_y, float &meas_z, INS_orientation orientation):_sens(sensor){
	switch (orientation){
		case (X_FRONT_Z_UP):
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx = 1;
			_sy = - 1;
			_sz = - 1;
			break;
		case (X_LEFT_Z_UP):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx = - 1;
			_sy = - 1;
			_sz = - 1;
			break;
		case (X_BACK_Z_UP):
            _x_p = &meas_x;
            _y_p = &meas_y;
            _z_p = &meas_z;
			_sx = - 1;
			_sy = 1;
			_sz = - 1;
			break;
		case (X_RIGHT_Z_UP):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx = 1;
			_sy = 1;
			_sz = - 1;
			break;
		case (X_FRONT_Z_DOWN):
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx = 1;
			_sy = 1;
			_sz = 1;
			break;
		case (X_LEFT_Z_DOWN):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx = 1;
			_sy = - 1;
			_sz = 1;
			break;
		case (X_BACK_Z_DOWN):
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx = - 1;
			_sy = - 1;
			_sz = 1;
			break;
		case (X_RIGHT_Z_DOWN):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx = - 1;
			_sy = 1;
			_sz = 1;
			break;
		case (X_FRONT_Z_UP_LH):
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx = 1;
			_sy = 1;
			_sz = - 1;
			break;
		case (X_LEFT_Z_UP_LH):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx = - 1;
			_sy = 1;
			_sz = - 1;
			break;
		case (X_BACK_Z_UP_LH):
            _x_p = &meas_x;
            _y_p = &meas_y;
            _z_p = &meas_z;
			_sx = - 1;
			_sy = - 1;
			_sz = - 1;
			break;
		case (X_RIGHT_Z_UP_LH):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx = 1;
			_sy = - 1;
			_sz = - 1;
			break;
		case (X_FRONT_Z_DOWN_LH):
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx = 1;
			_sy = - 1;
			_sz = 1;
			break;
		case (X_LEFT_Z_DOWN_LH):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx = 1;
			_sy = 1;
			_sz = 1;
			break;
		case (X_BACK_Z_DOWN_LH):
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx = - 1;
			_sy = 1;
			_sz = 1;
			break;
		case (X_RIGHT_Z_DOWN_LH):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx = - 1;
			_sy = - 1;
			_sz = 1;
			break;
		default:
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx = 0;
			_sy = 0;
			_sz = 0;
			break;
	}
	x = (float) _sx * (*_x_p);
	y = (float) _sy * (*_y_p);
	z = (float) _sz * (*_z_p);
	instanced++;
	instance = instanced;
	bx = 0;
	by = 0;
	bz = 0;
}

//====================================Public Members=========================================//
//---------------------------Read-----------------------------//
uint8_t INS_Gyro::read(uint32_t timeout){
	uint8_t res=_sens.read_gyro(timeout);
	x = (float) _sx * (*_x_p) - bx;
	y = (float) _sy * (*_y_p) - by;
	z = (float) _sz * (*_z_p) - bz;
	return res;
}

//-------------------------Turn on---------------------------//
void INS_Gyro::turn_on(){
	_sens.turn_on_gyro();
	return;
}

//------------------------Turn off---------------------------//
void INS_Gyro::turn_off(){
	_sens.turn_off_gyro();
	return;
}

//-------------------------Sleep----------------------------//
void INS_Gyro::sleep(){
	_sens.sleep_gyro();
	return;
}

//-----------------------Calibrate---------------------------//
uint8_t INS_Gyro::calibrate(uint8_t number_of_measures, uint32_t timeout){
	float bx_tmp=0;
	float by_tmp=0;
	float bz_tmp=0;
	bx=0;
	by=0;
	bz=0;
	// Discard the first n measures 
	if(!_sens.discard_measures_gyro(INS_GYRO_DISCARDED_MEASURES,timeout)){
		return 0;
	}
	for (uint8_t ii=0;ii<number_of_measures;ii++){
		if(this->read(timeout)){
			bx_tmp+=x;
			by_tmp+=y;
			bz_tmp+=z;
		}
		else{
			return 0;
		}
	}
	bx=bx_tmp/number_of_measures;
	by=by_tmp/number_of_measures;
	bz=bz_tmp/number_of_measures;
	return _sens.check_gyro_biases(bx,by,bz);
}