//  INS_Accel.cpp
//
//
//  Created by Andrea Vivani on 15/4/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#include "INS_Accel.h"
uint8_t INS_Accel::instanced=0;

//=====================================Constructor==========================================//
INS_Accel::INS_Accel(InertialSensor &sensor, float &meas_x, float &meas_y, float &meas_z, INS_orientation orientation):_sens(sensor), _data(1,1,0){
	switch (orientation){
		case (X_FRONT_Z_UP):
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx=1;
			_sy=-1;
			_sz=-1;
			break;
		case (X_LEFT_Z_UP):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx=-1;
			_sy=-1;
			_sz=-1;
			break;
		case (X_BACK_Z_UP):
			_sx=-1;
			_sy=1;
			_sz=-1;
			break;
		case (X_RIGHT_Z_UP):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx=1;
			_sy=1;
			_sz=-1;
			break;
		case (X_FRONT_Z_DOWN):
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx=1;
			_sy=1;
			_sz=1;
			break;
		case (X_LEFT_Z_DOWN):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx=1;
			_sy=-1;
			_sz=1;
			break;
		case (X_BACK_Z_DOWN):
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx=-1;
			_sy=-1;
			_sz=1;
			break;
		case (X_RIGHT_Z_DOWN):
			_y_p = &meas_x;
			_x_p = &meas_y;
			_z_p = &meas_z;
			_sx=-1;
			_sy=1;
			_sz=1;
			break;
		default:
			_x_p = &meas_x;
			_y_p = &meas_y;
			_z_p = &meas_z;
			_sx=0;
			_sy=0;
			_sz=0;
			break;
	}
	x = (float) _sx * (*_x_p);
	y = (float) _sy * (*_y_p);
	z = (float) _sz * (*_z_p);
	instanced++;
	instance=instanced;
	bx=0;
	by=0;
	bz=0;
	s11=1;
	s12=0;
	s13=0;
	s22=1;
	s23=0;
	s33=1;
}

//====================================Public Members=========================================//
//---------------------------Read-----------------------------//
uint8_t INS_Accel::read(uint32_t timeout){
	uint8_t res=_sens.read_accel(timeout);
	float ax_ub = (float) _sx * (*_x_p) - bx;
	float ay_ub = (float) _sy * (*_y_p) - by;
	float az_ub = (float) _sz * (*_z_p) - bz;
	x = s11 * ax_ub + s12 * ay_ub + s13 * az_ub;
	y = s12 * ax_ub + s22 * ay_ub + s23 * az_ub;
	z = s13 * ax_ub + s23 * ay_ub + s33 * az_ub;
	return res;
}

//-------------------------Turn on---------------------------//
void INS_Accel::turn_on(){
	_sens.turn_on_accel();
	return;
}

//------------------------Turn off---------------------------//
void INS_Accel::turn_off(){
	_sens.turn_off_accel();
	return;
}

//--------------------Init calibration-----------------------//
void INS_Accel::initialize_calibration(uint8_t number_of_measures){
	MatrixXf TMP(number_of_measures, 3);
	_data=TMP;
	_cal_count=0;
	bx=0;
	by=0;
	bz=0;
	s11=1;
	s12=0;
	s13=0;
	s22=1;
	s23=0;
	s33=1;
	return;
}

//-----------------Acquire single reading--------------------//
void INS_Accel::cal_acquire(uint32_t timeout){
	this->read(timeout);
	_data(_cal_count,0)=x;
	_data(_cal_count,1)=y;
	_data(_cal_count,2)=z;
	_cal_count++;
	return;
}

//----------------Acquire averaged readings-------------------//
void INS_Accel::cal_acquire_averaged(uint8_t average_length, uint32_t timeout){
	float x_tmp=0;
	float y_tmp=0;
	float z_tmp=0;
	for(uint8_t ii=0;ii<average_length;ii++){
		this->read(timeout);
		x_tmp+=x;
		y_tmp+=y;
		z_tmp+=z;
	}
	_data(_cal_count,0)=x_tmp/average_length;
	_data(_cal_count,1)=y_tmp/average_length;
	_data(_cal_count,2)=z_tmp/average_length;
	_cal_count++;
	return;
}

//------------------Compute coefficients---------------------//
uint8_t INS_Accel::cal_compute(INS_Accel_cal calibration_mode, uint8_t print_flag){
	MatrixXf P(1,1);
	if (calibration_mode==INS_Accel_cal_9_param){
		int8_t _data0[] = {0, 0, 0, 1, 0, 0, 1, 0, 1};
		MatrixXs X0(9, 1, _data0);
		P = GaussNewton_Sens_Cal_9(_data, (float) INS_G_VAL, X0, (uint16_t) INS_ACCEL_NMAX_ITER, (double) INS_ACCEL_TOL);
	}
	else{
		int8_t _data0[] = {0, 0, 0, 1, 1, 1};
		MatrixXs X0(6, 1, _data0);
		P = GaussNewton_Sens_Cal_6(_data, (float) INS_G_VAL, X0, (uint16_t) INS_ACCEL_NMAX_ITER, (double) INS_ACCEL_TOL);
	}
	// Check if reached the maximum number of iterations or not (returns a zeroed-out matrix in that case)
	if (P.sum()==0){
		reset_data_matrix();
		return 2;
	}
	// Check if values were too less linearly independent (algorithm does not converge)
	else if (isnan(P.sum())){
		reset_data_matrix();
		return 0;
	}
	// If there were no numerical issues
	else{
		if (calibration_mode==INS_Accel_cal_9_param){
			bx=P(0,0);
			by=P(1,0);
			bz=P(2,0);
			s11=P(3,0);
			s12=P(4,0);
			s13=P(5,0);
			s22=P(6,0);
			s23=P(7,0);
			s33=P(8,0);
		}
		else {
			bx=P(0,0);
			by=P(1,0);
			bz=P(2,0);
			s11=P(3,0);
			s22=P(4,0);
			s33=P(5,0);
		}
		if (print_flag){
			print_calibration_values();
		}
		reset_data_matrix();
		// Check if biases are within boundaries
		if(_sens.check_accel_biases(bx,by,bz)){
			return 1;
		}
		else{
			return 3;
		}
	}
}

//------------------Complete calibration---------------------//
uint8_t INS_Accel::calibrate(uint8_t number_of_measures, INS_Accel_cal calibration_mode, uint32_t timeout, uint8_t print_flag){
	Serial.println();
	Serial.println("Starting calibration...");
	initialize_calibration(number_of_measures);
	Serial.println("Press a key to acquire a sample");
	while(_cal_count<number_of_measures){
		if (Serial.available()>0){
            Serial.read();
			cal_acquire(INS_ACCEL_TIMEOUT);
			Serial.print("Sample acquired! Acc x: ");
			Serial.print(x,4);
			Serial.print(", Acc y: ");
			Serial.print(y,4);
			Serial.print(", Acc z: ");
			Serial.println(z,4);
			Serial.println("Press a key to acquire a sample");
		}
	}
	Serial.println("Done acquisition, now computing...");
	return cal_compute(calibration_mode, print_flag);
}

//-------------Complete calibration with average-------------//
uint8_t INS_Accel::calibrate_average(uint8_t number_of_measures, uint8_t average_length, INS_Accel_cal calibration_mode, uint32_t timeout, uint8_t print_flag){
	Serial.println();
	Serial.println("Starting calibration with averaged values...");
	initialize_calibration(number_of_measures);
	Serial.println("Press a key to acquire a sample");
	while(_cal_count<number_of_measures){
		if (Serial.available()>0){
            Serial.read();
			cal_acquire_averaged(INS_ACCEL_TIMEOUT);
			Serial.print("Sample acquired! Acc x: ");
			Serial.print(x,4);
			Serial.print(", Acc y: ");
			Serial.print(y,4);
			Serial.print(", Acc z: ");
			Serial.println(z,4);
			Serial.println("Press a key to acquire a sample");
		}
	}
	Serial.println("Done acquisition, now computing...");
	return cal_compute(calibration_mode, print_flag);
}

//====================================Private Members=========================================//
//--------------------Print cal values-----------------------//
void INS_Accel::print_calibration_values(){
	Serial.println();
	Serial.println("Scale factors matrix:");
	// first row
	Serial.print(s11,4);
	Serial.print("\t");
	Serial.print(s12,4);
	Serial.print("\t");
	Serial.println(s13,4);
	// second row
	Serial.print(s12,4);
	Serial.print("\t");
	Serial.print(s22,4);
	Serial.print("\t");
	Serial.println(s23,4);
	// third row
	Serial.print(s13,4);
	Serial.print("\t");
	Serial.print(s23,4);
	Serial.print("\t");
	Serial.println(s33,4);
	// Biases
	Serial.println();
	Serial.println("Bias values:");
	Serial.print("Bx: ");
	Serial.print(bx,4);
	Serial.print(", By: ");
	Serial.print(by,4);
	Serial.print(", Bz: ");
	Serial.println(bz,4);
    Serial.println();
}

//-------------------Reset data matrix---------------------//
void INS_Accel::reset_data_matrix(){
	MatrixXf TMP(1,1,0);
	_data=TMP;
}

