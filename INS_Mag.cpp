//  INS_Mag.cpp
//
//
//  Created by Andrea Vivani on 15/4/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#include "INS_Mag.h"
uint8_t INS_Mag::instanced = 0;

//=====================================Constructor==========================================//
INS_Mag::INS_Mag(InertialSensor &sensor, float &meas_x, float &meas_y, float &meas_z, INS_orientation orientation):_sens(sensor), _data(1,1,0){
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
	s11 = 1;
	s12 = 0;
	s13 = 0;
	s22 = 1;
	s23 = 0;
	s33 = 1;
}

//====================================Public Members=========================================//
//---------------------------Read-----------------------------//
uint8_t INS_Mag::read(uint32_t timeout){
	uint8_t res=_sens.read_mag(timeout);
	float mx_ub = (float) _sx * (*_x_p) - bx;
	float my_ub = (float) _sy * (*_y_p) - by;
	float mz_ub = (float) _sz * (*_z_p) - bz;
	x = s11 * mx_ub + s12 * my_ub + s13 * mz_ub;
	y = s12 * mx_ub + s22 * my_ub + s23 * mz_ub;
	z = s13 * mx_ub + s23 * my_ub + s33 * mz_ub;
	return res;
}

//-------------------------Turn on---------------------------//
void INS_Mag::turn_on(){
	_sens.turn_on_mag();
	return;
}

//------------------------Turn off---------------------------//
void INS_Mag::turn_off(){
	_sens.turn_off_mag();
	return;
}

//--------------------Init calibration-----------------------//
void INS_Mag::initialize_calibration(uint8_t number_of_measures){
	MatrixXf TMP(number_of_measures, 3);
	_data = TMP;
	_cal_count = 0;
	bx = 0;
	by = 0;
	bz = 0;
	s11 = 1;
	s12 = 0;
	s13 = 0;
	s22 = 1;
	s23 = 0;
	s33 = 1;
	return;
}

//-----------------Acquire single reading--------------------//
void INS_Mag::cal_acquire(uint32_t timeout){
	this->read(timeout);
	_data(_cal_count,0) = x;
	_data(_cal_count,1) = y;
	_data(_cal_count,2) = z;
	_cal_count++;
	return;
}

//----------------Acquire averaged readings-------------------//
void INS_Mag::cal_acquire_averaged(uint8_t average_length, uint32_t timeout){
	float x_tmp = 0;
	float y_tmp = 0;
	float z_tmp = 0;
	for(uint8_t ii = 0; ii < average_length; ii++){
		this->read(timeout);
		x_tmp += x;
		y_tmp += y;
		z_tmp += z;
	}
	_data(_cal_count,0) = x_tmp / average_length;
	_data(_cal_count,1) = y_tmp / average_length;
	_data(_cal_count,2) = z_tmp / average_length;
	_cal_count++;
	return;
}

//------------------Compute coefficients---------------------//
uint8_t INS_Mag::cal_compute(INS_Mag_cal calibration_mode, uint8_t print_flag){
	MatrixXf P(1,1);
	float radius;
	if(!INS_MAG_VAL){
		radius = mean_radius(_data);
	}
	else{
		radius = INS_MAG_VAL;
	}
	if (calibration_mode == INS_Mag_cal_9_param){
		int8_t _data0[] = {0, 0, 0, 1, 0, 0, 1, 0, 1};
		MatrixXs X0(9, 1, _data0);
		P = GaussNewton_Sens_Cal_9(_data, radius, X0, (uint16_t) INS_MAG_NMAX_ITER, (double) INS_MAG_TOL);
	}
	else{
		int8_t _data0[] = {0, 0, 0, 1, 1, 1};
		MatrixXs X0(6, 1, _data0);
		P = GaussNewton_Sens_Cal_6(_data, radius, X0, (uint16_t) INS_MAG_NMAX_ITER, (double) INS_MAG_TOL);
	}
	// Check result
	if (P.sum() == 0){
		reset_data_matrix();
		return 2;
	}
	else if (isnan(P.sum())){
		reset_data_matrix();
		return 0;
	}
	else{
		if (calibration_mode == INS_Mag_cal_9_param){
			bx = P(0,0);
			by = P(1,0);
			bz = P(2,0);
			s11 = P(3,0);
			s12 = P(4,0);
			s13 = P(5,0);
			s22 = P(6,0);
			s23 = P(7,0);
			s33 = P(8,0);
		}
		else {
			bx = P(0,0);
			by = P(1,0);
			bz = P(2,0);
			s11 = P(3,0);
			s22 = P(4,0);
			s33 = P(5,0);
		}
		if (print_flag){
			print_calibration_values();
		}
		reset_data_matrix();
		return 1;
	}
}

//------------------Complete calibration---------------------//
#ifdef INS_ARDUINO
uint8_t INS_Mag::calibrate(uint8_t number_of_measures, INS_Mag_cal calibration_mode, uint32_t timeout, uint8_t print_flag){
	Serial.println();
	Serial.println("Starting calibration...");
	initialize_calibration(number_of_measures);
	Serial.println("Press a key to acquire a sample");
	while(_cal_count < number_of_measures){
		if (Serial.available()>0){
            Serial.read();
			cal_acquire(timeout);
			Serial.print("Sample acquired! Mag x: ");
			Serial.print(x,4);
			Serial.print(", Mag y: ");
			Serial.print(y,4);
			Serial.print(", Mag z: ");
			Serial.println(z,4);
			Serial.println("Press a key to acquire a sample");
		}
	}
	Serial.println("Done acquisition, now computing...");
	return cal_compute(calibration_mode, print_flag);
}
#elif INS_CHIBIOS
uint8_t INS_Mag::calibrate(uint8_t number_of_measures, INS_Mag_cal calibration_mode, uint32_t timeout, uint8_t print_flag){
	chprintf(SERIAL_INT, "\r\nStarting calibration...\r\n");
	initialize_calibration(number_of_measures);
	chprintf(SERIAL_INT, "Press a key to acquire a sample\r\n");
	while(_cal_count < number_of_measures){
		chEvtWaitOne(EVENT_MASK(1));
		chSysLock();
		flags_INS = chEvtGetAndClearFlags(&el_INS);
		chSysUnlock();
		if (flags_INS & CHN_INPUT_AVAILABLE){
			msg_t charbuf;
			do{
				charbuf = chnGetTimeout(SERIAL_INT, TIME_IMMEDIATE);
			}
			while (charbuf != Q_TIMEOUT);
			cal_acquire(timeout);
			chprintf(SERIAL_INT,"Sample acquired! Mag x: %-9.4f, Mag y: %-9.4f, Mag z: %-9.4\r\nPress a key to acquire a sample\r\n", x, y ,z);
		}
	}
	chprintf(SERIAL_INT, "Done acquisition, now computing...\r\n");
	return cal_compute(calibration_mode, print_flag);
}
#endif

//-------------Complete calibration with average-------------//
#ifdef INS_ARDUINO
uint8_t INS_Mag::calibrate_average(uint8_t number_of_measures, uint8_t average_length, INS_Mag_cal calibration_mode, uint32_t timeout, uint8_t print_flag){
	Serial.println();
	Serial.println("Starting calibration with averaged values...");
	initialize_calibration(number_of_measures);
	Serial.println("Press a key to acquire a sample");
	while(_cal_count < number_of_measures){
		if (Serial.available()>0){
            Serial.read();
			cal_acquire_averaged(average_length, timeout);
			Serial.print("Sample acquired! Mag x: ");
			Serial.print(x,4);
			Serial.print(", Mag y: ");
			Serial.print(y,4);
			Serial.print(", Mag z: ");
			Serial.println(z,4);
			Serial.println("Press a key to acquire a sample");
		}
	}
	Serial.println("Done acquisition, now computing...");
	return cal_compute(calibration_mode, print_flag);
}
#elif INS_CHIBIOS
uint8_t INS_Mag::calibrate_average(uint8_t number_of_measures, uint8_t average_length, INS_Mag_cal calibration_mode, uint32_t timeout, uint8_t print_flag){
		chprintf(SERIAL_INT, "\r\nStarting calibration with averaged values...\r\n");
	initialize_calibration(number_of_measures);
	chprintf(SERIAL_INT, "Press a key to acquire a sample\r\n");
	while(_cal_count < number_of_measures){
		chEvtWaitOne(EVENT_MASK(1));
		chSysLock();
		flags_INS = chEvtGetAndClearFlags(&el_INS);
		chSysUnlock();
		if (flags_INS & CHN_INPUT_AVAILABLE){
			msg_t charbuf;
			do{
				charbuf = chnGetTimeout(SERIAL_INT, TIME_IMMEDIATE);
			}
			while (charbuf != Q_TIMEOUT);
			cal_acquire_averaged(timeout);
			chprintf(SERIAL_INT,"Sample acquired! Mag x: %-9.4f, Mag y: %-9.4f, Mag z: %-9.4\r\nPress a key to acquire a sample\r\n", x, y ,z);
		}
	}
	chprintf(SERIAL_INT, "Done acquisition, now computing...\r\n");
	return cal_compute(calibration_mode, print_flag);
}
#endif

//====================================Private Members=========================================//
//--------------------Print cal values-----------------------//
void INS_Mag::print_calibration_values(){
#ifdef INS_ARDUINO
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
#elif INS_CHIBIOS
	chprintf(SERIAL_INT, "\r\nScale factors matrix:\r\n");
	chprintf(SERIAL_INT, "%-9.4f\t%-9.4f\t%-9.4f\r\n%-9.4f\t%-9.4f\t%-9.4f\r\n%-9.4f\t%-9.4f\t%-9.4f\r\n", s11, s12, s13, s12, s22, s23, s13, s23, s33);
	chprintf(SERIAL_INT, "\r\nBias values:\r\n");
	chprintf(SERIAL_INT, "Bx: %-9.4f, By: %-9.4f, Bz: %-9.4f\r\n\r\n", bx, by, bz);
#endif
}

//-------------------Reset data matrix---------------------//
void INS_Mag::reset_data_matrix(){
	MatrixXf TMP(1,1,0);
	_data=TMP;
}

//------------------Compute mean radius-------------------//
float INS_Mag::mean_radius(MatrixXf &data){
	MatrixXf radii(data.rows(),1);
	for (uint16_t ii = 0; ii < data.rows(); ii++){
		radii(ii,0) = sqrtf(data(ii,0) * data(ii,0) + data(ii,1) * data(ii,1) + data(ii,2) * data(ii,2));
	}
	return (radii.sum() / data.rows()); 
}
