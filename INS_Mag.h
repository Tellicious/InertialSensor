//  INS_Mag.h
//
//
//  Created by Andrea Vivani on 15/4/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#ifndef INS_Mag_H_
#define INS_Mag_H_
#include "InertialSensor.h"
#include "MatLib.h"
#include "math.h"
//======================================Parameters=============================================//
#define INS_MAG_DISCARDED_MEASURES	20	//number of measures to be discarded
#define INS_MAG_CALIBRATION_MEASURES 20 //default numer of measures to be used during calibration
#define INS_MAG_AVERAGED_MEASURES 10 //default number of measures used to average the readings when calibrating
#define INS_MAG_TIMEOUT 2e6	//default timeout when performing calibration
#define INS_MAG_TOL 1e-5	//tolerance used to stop calibration
#define INS_MAG_NMAX_ITER 200 //number of maximum iterations during calibration

typedef enum {
	INS_Mag_cal_9_param,
	INS_Mag_cal_6_param,
} INS_Mag_cal;

class INS_Mag{
public:
	static uint8_t instanced; //number of instanced magnetometers
	uint8_t instance; //instance number of this magnetometer
	float x, y, z; //output values
	float s11, s12, s13, s22, s23, s33; //magerometer scale factors
	float bx, by, bz; //magerometer biases
	INS_Mag(InertialSensor &sensor, float &meas_x, float &meas_y, float &meas_z, INS_orientation orientation); //constructor
	uint8_t read(uint32_t timeout=INS_MAG_TIMEOUT); //read data, timeout in us
	void turn_on(); //turn on the sensor
	void turn_off(); //turn off the sensor
	void initialize_calibration(uint8_t number_of_measures);
	void cal_acquire(uint32_t timeout=INS_MAG_TIMEOUT); //acquire measure during calibration, timeout in us
	void cal_acquire_averaged(uint8_t average_length=INS_MAG_AVERAGED_MEASURES, uint32_t timeout=INS_MAG_TIMEOUT); //acquire averaged measure during calibration, timeout for each measure in us
	// CALIBRATION RETURNS 0 IF NOT POSSIBLE, 1 IF SUCCESSFUL, 2 IF NOT CONVERGED
	uint8_t cal_compute(INS_Mag_cal calibration_mode=INS_Mag_cal_9_param, uint8_t print_flag=1); //compute values, mode 0: 9 parameters, mode 1: 6 parameters, print_flag 0: no print, print_flag=1 prints the result
	uint8_t calibrate(uint8_t number_of_measures=INS_MAG_CALIBRATION_MEASURES, INS_Mag_cal calibration_mode=INS_Mag_cal_9_param, uint32_t timeout=INS_MAG_TIMEOUT, uint8_t print_flag=1); //complete calibration procedure using single readings, mode 0: 9 parameters, mode 1: 6 parameters, print_flag 0: no print, print_flag=1 prints the result
	uint8_t calibrate_average(uint8_t number_of_measures=INS_MAG_CALIBRATION_MEASURES, uint8_t average_length=INS_MAG_AVERAGED_MEASURES, INS_Mag_cal calibration_mode=INS_Mag_cal_9_param, uint32_t timeout=INS_MAG_TIMEOUT, uint8_t print_flag=1);	//complete calibration using averaged readings, mode 0: 9 parameters, mode 1: 6 parameters, print_flag 0: no print, print_flag=1 prints the result
private:
	InertialSensor &_sens;
	float* _x_p;
	float* _y_p;
	float* _z_p; //pointers to sensor output values
	int8_t _sx, _sy, _sz; //factors to rotate readings in order to provide always X front, Y right and Z down
	MatrixXf _data; //matrix object used during calibration
	uint8_t _cal_count; //counter used to calibrate
	void print_calibration_values(); //prints the calibration values
	void reset_data_matrix(); //resets the _data matrix to a single value, in order to free some space
	float mean_radius(MatrixXf &data); //returns the mean radius of a given set of measures
};
#endif