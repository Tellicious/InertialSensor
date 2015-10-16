//
// LSM9DS0.cpp
//
//
// Created by Andrea Vivani on 28/3/15.
// Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "LSM9DS0.h"
#include "Arduino.h"
#include <SPI.h>

//====================================Registers Addresses=========================================// 
#define LSM9DS0_WHO_AM_I_G		0x0F
#define LSM9DS0_CTRL_REG1_G 	0x20
#define LSM9DS0_CTRL_REG2_G		0x21
#define LSM9DS0_CTRL_REG3_G		0x22
#define LSM9DS0_CTRL_REG4_G		0x23
#define LSM9DS0_CTRL_REG5_G		0x24
#define LSM9DS0_REFERENCE_G		0x25
#define LSM9DS0_STATUS_REG_G	0x27
#define LSM9DS0_OUT_X_L_G		0x28
#define LSM9DS0_OUT_X_H_G		0x29
#define LSM9DS0_OUT_Y_L_G		0x2A
#define LSM9DS0_OUT_Y_H_G		0x2B
#define LSM9DS0_OUT_Z_L_G		0x2C
#define LSM9DS0_OUT_Z_H_G		0x2D
#define LSM9DS0_FIFO_CTRL_REG_G	0x2E
#define LSM9DS0_FIFO_SRC_REG_G	0x2F
#define LSM9DS0_INT1_CFG_G		0x30
#define LSM9DS0_INT1_SRC_G		0x31
#define LSM9DS0_INT1_TSH_XH_G	0x32
#define LSM9DS0_INT1_TSH_XL_G	0x33
#define LSM9DS0_INT1_TSH_YH_G	0x34
#define LSM9DS0_INT1_TSH_YL_G	0x35
#define LSM9DS0_INT1_TSH_ZH_G	0x36
#define LSM9DS0_INT1_TSH_ZL_G	0x37
#define LSM9DS0_INT1_DURATION_G	0x38
#define LSM9DS0_OUT_TEMP_L_XM	0x05
#define LSM9DS0_OUT_TEMP_H_XM	0x06
#define LSM9DS0_STATUS_REG_M	0x07
#define LSM9DS0_OUT_X_L_M		0x08
#define LSM9DS0_OUT_X_H_M		0x09
#define LSM9DS0_OUT_Y_L_M		0x0A
#define LSM9DS0_OUT_Y_H_M		0x0B
#define LSM9DS0_OUT_Z_L_M		0x0C
#define LSM9DS0_OUT_Z_H_M		0x0D
#define LSM9DS0_WHO_AM_I_XM		0x0F
#define LSM9DS0_INT_CTRL_REG_M	0x12
#define LSM9DS0_INT_SRC_REG_M	0x13
#define LSM9DS0_INT_THS_L_M		0x14
#define LSM9DS0_INT_THS_H_M		0x15
#define LSM9DS0_OFFSET_X_L_M	0x16
#define LSM9DS0_OFFSET_X_H_M	0x17
#define LSM9DS0_OFFSET_Y_L_M	0x18
#define LSM9DS0_OFFSET_Y_H_M	0x19
#define LSM9DS0_OFFSET_Z_L_M	0x1A
#define LSM9DS0_OFFSET_Z_H_M	0x1B
#define LSM9DS0_REFERENCE_X		0x1C
#define LSM9DS0_REFERENCE_Y		0x1D
#define LSM9DS0_REFERENCE_Z		0x1E
#define LSM9DS0_CTRL_REG0_XM	0x1F
#define LSM9DS0_CTRL_REG1_XM	0x20
#define LSM9DS0_CTRL_REG2_XM	0x21
#define LSM9DS0_CTRL_REG3_XM	0x22
#define LSM9DS0_CTRL_REG4_XM	0x23
#define LSM9DS0_CTRL_REG5_XM	0x24
#define LSM9DS0_CTRL_REG6_XM	0x25
#define LSM9DS0_CTRL_REG7_XM	0x26
#define LSM9DS0_STATUS_REG_A	0x27
#define LSM9DS0_OUT_X_L_A		0x28
#define LSM9DS0_OUT_X_H_A		0x29
#define LSM9DS0_OUT_Y_L_A		0x2A
#define LSM9DS0_OUT_Y_H_A		0x2B
#define LSM9DS0_OUT_Z_L_A		0x2C
#define LSM9DS0_OUT_Z_H_A		0x2D
#define LSM9DS0_FIFO_CTRL_REG 	0x2E
#define LSM9DS0_FIFO_SRC_REG 	0x2F
#define LSM9DS0_INT_GEN_1_REG	0x30
#define LSM9DS0_INT_GEN_1_SRC	0x31
#define LSM9DS0_INT_GEN_1_THS	0x32
#define LSM9DS0_INT_GEN_1_DURATION	0x33
#define LSM9DS0_INT_GEN_2_REG	0x34
#define LSM9DS0_INT_GEN_2_SRC	0x35
#define LSM9DS0_INT_GEN_2_THS	0x36
#define LSM9DS0_INT_GEN_2_DURATION	0x37
#define LSM9DS0_CLICK_CFG	0x38
#define LSM9DS0_CLICK_SRC	0x39
#define LSM9DS0_CLICK_THS	0x3A
#define LSM9DS0_TIME_LIMIT	0x3B
#define LSM9DS0_TIME_LATENCY	0x3C
#define LSM9DS0_TIME_WINDOW	0x3D
#define LSM9DS0_ACT_THS	0x3E
#define LSM9DS0_ACT_DUR	0x3F

//=======================================Constants=============================================// 
#define LSM9DS0_ID_G		0xD4
#define LSM9DS0_ID_XM		0x49
#define LSM9DS0_READ		0x80
#define LSM9DS0_MULT		0x40

//==================================Auxiliary Functions========================================//
//---------------Read one register from the SPI-----------------//
uint8_t LSM9DS0::readRegister(uint8_t chipSelectPin, uint8_t thisRegister) {
	uint8_t inByte = 0;   	// incoming byte
	thisRegister |= LSM9DS0_READ;		// register in read mode
	//uint8_t oldSPCR = SPCR;				// actual SPI configuration register
	//SPCR = _mySPCR;						// set the desired SPCR
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(thisRegister);		// send the command to read thisRegister
	inByte = SPI.transfer(0x00);		// send 0x00 in order to read the incoming byte
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	//SPCR = oldSPCR;						// restores the old SPCR
	return(inByte);			// return the read byte
}

//------------Read multiple registers from the SPI--------------//
void LSM9DS0::readMultipleRegisters(uint8_t chipSelectPin, uint8_t * buffer, uint8_t number_of_registers, uint8_t startRegister) {
	startRegister |= (LSM9DS0_READ | LSM9DS0_MULT);// register in multiple read mode
	//uint8_t oldSPCR = SPCR;				// actual SPI configuration register
	//SPCR = _mySPCR;						// set the desired SPCR
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(startRegister);		// send the command to read thisRegister
	for (uint8_t ii = 0; ii < number_of_registers; ii++){
		buffer[ii] = SPI.transfer(0x00);
	}
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	//SPCR = oldSPCR;						// restores the old SPCR
	return;
}

//---------------Write one register on the SPI-----------------//
void LSM9DS0::writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue) {
	//uint8_t oldSPCR = SPCR;				// actual SPI configuration register
	//SPCR = _mySPCR;						// set the desired SPCR
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(thisRegister); 		// send register location
	SPI.transfer(thisValue); 		// send value to record into register
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	//SPCR = oldSPCR;						// restores the old SPCR
	return;
}

//-----------------Check values for self-test-------------------//
uint8_t LSM9DS0::ch_st (const double val1, const double val2, const double lim1, const double lim2){
    if (fabs(lim1) > fabs(lim2)){
        return ((fabs(val2 - val1) >= fabs(lim2)) && (fabs(val2 - val1) <= fabs(lim1)));
    }
    return ((fabs(val2 - val1) >= fabs(lim1)) && (fabs(val2 - val1) <= fabs(lim2)));
    
}

//=====================================Constructors==========================================//
LSM9DS0::LSM9DS0 (uint8_t CS_pin_G, uint8_t CS_pin_XM):InertialSensor(){
	_chipSelectPin_G = CS_pin_G;
	_chipSelectPin_XM = CS_pin_XM;
	_DRDY_pin_G = 0;
	_DRDY_pin_A = 0;
	_DRDY_pin_M = 0;
}

LSM9DS0::LSM9DS0 (uint8_t CS_pin_G, uint8_t CS_pin_XM, uint8_t DRDY_pin_G, uint8_t DRDY_pin_A, uint8_t DRDY_pin_M):InertialSensor(){
	_chipSelectPin_G = CS_pin_G;
	_chipSelectPin_XM = CS_pin_XM;
	_DRDY_pin_G = DRDY_pin_G;
	_DRDY_pin_A = DRDY_pin_A;
	_DRDY_pin_M = DRDY_pin_M;
}

//-----------------------Initialization-----------------------//
void LSM9DS0::init(){
	pinMode(_chipSelectPin_G,OUTPUT);
	pinMode(_chipSelectPin_XM,OUTPUT);
	digitalWrite(_chipSelectPin_G,HIGH);
	digitalWrite(_chipSelectPin_XM,HIGH);
	if ((_DRDY_pin_G != 0) || (_DRDY_pin_A != 0) || (_DRDY_pin_M != 0)){
		pinMode(_DRDY_pin_G,INPUT);
		pinMode(_DRDY_pin_A,INPUT);
		pinMode(_DRDY_pin_M,INPUT);
	}
	// initialize variables
	gx = 0;
	gy = 0;
	gz = 0;
	ax = 0;
	ay = 0;
	az = 0;
	mx = 0;
	my = 0;
	mz = 0;
	temperature = 0;
}

//=================================Public Members Gyro=======================================//
//-----------------------Configuration-----------------------//
uint8_t LSM9DS0::config_gyro(uint8_t gyro_range, uint8_t gyro_odr, uint8_t LPF2_enable, uint8_t HP_enable, uint8_t HP_freq){
	init();
	// Trash the first reading
	readRegister(_chipSelectPin_G, LSM9DS0_WHO_AM_I_G);
	// Check if the device ID is correct
	if (readRegister(_chipSelectPin_G, LSM9DS0_WHO_AM_I_G) != LSM9DS0_ID_G){
		return 0;
	}
	//
	uint8_t CTRL2_val;
	if (HP_enable){
		//no DEN interrupt, set High-pass filter frequency
		CTRL2_val = HP_freq;
	}
	else{
		//no DEN interrupt, no High-pass filter
		CTRL2_val = 0x00;
	}
	writeRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG2_G, CTRL2_val);
	//
	//Data ready on DRDY_G
	uint8_t CTRL3_val = (1 << 3);
	writeRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG3_G, CTRL3_val);
	//
	//Continuous update, Little endian, selected range, normal self-test, SPI 4-wire
	uint8_t CTRL4_val = (0 << 7) | gyro_range; 
	writeRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG4_G, CTRL4_val);
	//
	//FIFO control on Bypass Mode
	uint8_t FIFO_CTRL_val = 0x00;
	writeRegister(_chipSelectPin_G, LSM9DS0_FIFO_CTRL_REG_G, FIFO_CTRL_val);
	//
	//Enable FIFO, set HP filter, set LPF2
	uint8_t CTRL5_val = (1 << 6);
	if (HP_enable){
		CTRL5_val |= (1 << 4);
		CTRL5_val |= (0x5);
	}
	if (LPF2_enable){ 
		CTRL5_val |= (0xF);
	}
	writeRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG5_G, CTRL5_val);
	//
	switch (gyro_range){
		case (LSM9DS0_RANGE_G_245):
			_sc_fact_g = 8.75e-3 * INS_TORAD;
			break;
		case (LSM9DS0_RANGE_G_500):
			_sc_fact_g = 17.5e-3 * INS_TORAD;
			break;
		case (LSM9DS0_RANGE_G_2000):
			_sc_fact_g = 70e-3 * INS_TORAD;
			break;
		default:
			return 2;
	}
	//
	//Clear the Reference register
	writeRegister(_chipSelectPin_G, LSM9DS0_REFERENCE_G, 0x00);
	//Selected ODR, power on, 3-axis enabled
	_CTRL1_val_G = (gyro_odr << 4) | (1 << 3) | 0x7;
	writeRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG1_G, _CTRL1_val_G);
	delay(200);
	// Discard the first n measures
	if(! discard_measures_gyro(LSM9DS0_DISCARDED_MEASURES, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-------------------------Turn on---------------------------//
void LSM9DS0::turn_on_gyro(){
	writeRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG1_G,_CTRL1_val_G);
	delay(200);
}

//------------------------Turn off---------------------------//
void LSM9DS0::turn_off_gyro(){
	writeRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG1_G,(_CTRL1_val_G & 0xF7));
}

//-------------------------Sleep-----------------------------//
void LSM9DS0::sleep_gyro(){
	writeRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG1_G,(_CTRL1_val_G & 0xF8));
}

//------------------------Read data-------------------------//
uint8_t LSM9DS0::read_raw_gyro(){
	uint8_t buffer[6];
	readMultipleRegisters(_chipSelectPin_G, buffer, 6, LSM9DS0_OUT_X_L_G);
	gx = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_g);
	gy = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_g);
	gz = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_g);
	return 1;
}

//------------------Read data when ready--------------------//
uint8_t LSM9DS0::read_gyro_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin_G) == 1){
			read_raw_gyro();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 0;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS0::read_gyro_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin_G, LSM9DS0_STATUS_REG_G);
		if ((STATUS_val & (1 << 3)) == (1 << 3)){
			read_raw_gyro();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//--------------------Check biases------------------------//
uint8_t LSM9DS0::check_gyro_biases(float bx, float by, float bz){
// Define Threshold based on Full-Scale value
	float thrs;
	if ((_sc_fact_g - 8.75e-3 * INS_TORAD) < 1e-5){
		thrs = 10 * INS_TORAD * 1.2; //typical 10dps offset
	}
	else if ((_sc_fact_g - 17.5e-3 * INS_TORAD) < 1e-5){
		thrs = 15 * INS_TORAD * 1.2;	//typical 15dps offset
	}
	else if ((_sc_fact_g - 70e-3 * INS_TORAD) < 1e-5){
		thrs = 25 * INS_TORAD * 1.2;	//typical 25dps offset
	}
	else {
		return 0;
	}
	if ((fabs(bx) > thrs) || (fabs(by) > thrs) || (fabs(bz) > thrs)){
		return 0;
	}
	return 1;
}

//---------------------High-Pass Reset----------------------//
void LSM9DS0::HP_reset_gyro(){
	readRegister(_chipSelectPin_G, LSM9DS0_REFERENCE_G);
}

//-----------------------Self-Test-------------------------//
uint8_t LSM9DS0::self_test_gyro(uint8_t mode){
	uint8_t status = 0;
	// Discard the first n measures
	if(!discard_measures_gyro(LSM9DS0_DISCARDED_MEASURES_ST, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (int ii = 0; ii < LSM9DS0_GYRO_SELF_TEST_MEASURES; ii++){
		read_gyro_STATUS(LSM9DS0_DISCARD_TIMEOUT);
		x_pre += gx;
		y_pre += gy;
		z_pre += gz;
	}
	x_pre /= LSM9DS0_GYRO_SELF_TEST_MEASURES;
	y_pre /= LSM9DS0_GYRO_SELF_TEST_MEASURES;
	z_pre /= LSM9DS0_GYRO_SELF_TEST_MEASURES;
	// Turn on self-test
	turn_off_gyro();
	uint8_t CTRL4_val = readRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG4_G);
	if (mode == 0){
		CTRL4_val |= (1 << 1); // Self-test mode 0
	}
	else {
		CTRL4_val |= ((1 << 1) | (1 << 2)); // Self-test mode 1
	}
	writeRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG4_G, CTRL4_val);
	turn_on_gyro();
	// Discard the first n measures 
	if( ! discard_measures_gyro(LSM9DS0_DISCARDED_MEASURES_ST, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (int ii = 0; ii < LSM9DS0_GYRO_SELF_TEST_MEASURES; ii++){
		read_gyro_STATUS(LSM9DS0_DISCARD_TIMEOUT);
		x_post += gx;
		y_post += gy;
		z_post += gz;
	}
	x_post /= LSM9DS0_GYRO_SELF_TEST_MEASURES;
	y_post /= LSM9DS0_GYRO_SELF_TEST_MEASURES;
	z_post /= LSM9DS0_GYRO_SELF_TEST_MEASURES;
	// Define threshold based on the Full-Scale value
	float thrs_min, thrs_max;
	if ((_sc_fact_g - 8.75e-3 * INS_TORAD) < 1e-5){
		thrs_min = 20.0 * INS_TORAD;
		thrs_max = 250.0 * INS_TORAD;
	}
	else if ((_sc_fact_g - 17.5e-3 * INS_TORAD) < 1e-5){
		thrs_min = 70.0 * INS_TORAD;
		thrs_max = 400.0 * INS_TORAD;
	}
	else if ((_sc_fact_g - 70e-3 * INS_TORAD) < 1e-5){
		thrs_min = 150.0 * INS_TORAD;
		thrs_max = 1000.0 * INS_TORAD;
	}
	else {
		return 0;
	}
	if (ch_st(x_pre, x_post, thrs_min, thrs_max) && ch_st(y_pre, y_post, thrs_min, thrs_max) && ch_st(z_pre, z_post, thrs_min, thrs_max)) {
		status = 1;
	}
	// Check if values are bigger than the threshold
	/*if (mode == 0){
		if ((gx > thrs) && (gy < - thrs) && (gz < - thrs)){
			status = 1;
		}
	}
	else {
		if ((gx < - thrs) && (gy > thrs) && (gz > thrs)){
			status = 1;
		}
	}*/
	turn_off_gyro();
	CTRL4_val &= 0xF9; // Removes Self-Test
	writeRegister(_chipSelectPin_G, LSM9DS0_CTRL_REG4_G, CTRL4_val);
	turn_on_gyro();
	// Discard the first n measures
	if(! discard_measures_gyro(LSM9DS0_DISCARDED_MEASURES_ST, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Gyro Status------------------------//
uint8_t LSM9DS0::status_gyro(){
	return readRegister(_chipSelectPin_G, LSM9DS0_STATUS_REG_G);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS0::discard_measures_gyro(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	while (count < (number_of_measures * 0.5)){
		uint8_t STATUS_value = status_gyro();
		if ((STATUS_value & (1 << 7)) == (1 << 7)){
			read_raw_gyro();
			now = micros();
			count++;
		}
		if ((micros() - now) > timeout){
			return 0;
		}
		else if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 1;
}

//=============================Public Members Accelerometer==================================== //
//-----------------------Configuration-----------------------//
uint8_t LSM9DS0::config_accel_mag(uint8_t accel_range, uint8_t accel_odr, uint8_t accel_bw, uint8_t mag_range, uint8_t mag_odr,uint8_t HP_accel_enable){
	init();
	// Trash the first reading
	readRegister(_chipSelectPin_XM, LSM9DS0_WHO_AM_I_XM);
	// Check if the device ID is correct
	if (readRegister(_chipSelectPin_XM, LSM9DS0_WHO_AM_I_XM) != LSM9DS0_ID_XM){
		return 0;
	}
	//
	// Enable FIFO Mode
	uint8_t CTRL0_val = 0x0|(1 << 6);
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG0_XM, CTRL0_val);
	//
	switch (accel_range){
		case (LSM9DS0_RANGE_A_2):
			_sc_fact_a = INS_G_VAL * 0.00006103515625;
			break;
		case (LSM9DS0_RANGE_A_4):
			_sc_fact_a = INS_G_VAL * 0.0001220703125;
			break;
		case (LSM9DS0_RANGE_A_6):
			_sc_fact_a = INS_G_VAL * 0.00018310546875;
			break;
		case (LSM9DS0_RANGE_A_8):
			_sc_fact_a = INS_G_VAL * 0.000244140625;
			break;
		case (LSM9DS0_RANGE_A_16):
			_sc_fact_a = INS_G_VAL * 0.000732421875;
			break;
		default:
			return 0;
	}
	//
	// Set selcted anti-alias and full-scale (accelerometer), disable self-test, 4-wire SPI
	uint8_t CTRL2_val = (accel_bw << 6) | (accel_range << 3);
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG2_XM, CTRL2_val);
	//
	// Accelerometer data ready on interrupt 1
	uint8_t CTRL3_val = (1 << 2);
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG3_XM, CTRL3_val);
	//
	// Magnetometer data ready on interrupt 2
	uint8_t CTRL4_val = (1 << 2);
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG4_XM, CTRL4_val);
	//
	// Enable temperature sensor, set high-resolution for magnetometer, set selected ODR (magnetometer)
	uint8_t CTRL5_val = (1 << 7) | (1 << 6) | (1 << 5) | (mag_odr << 2);
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG5_XM, CTRL5_val);
	//
	switch (mag_range){
		case (LSM9DS0_RANGE_M_2):
			_sc_fact_m = 0.08e-3;
			break;
		case (LSM9DS0_RANGE_M_4):
			_sc_fact_m = 0.16e-3;
			break;
		case (LSM9DS0_RANGE_M_8):
			_sc_fact_m = 0.32e-3;
			break;
		case (LSM9DS0_RANGE_M_12):
			_sc_fact_m = 0.48e-3;
			break;
		default:
			return 0;
	}
	//
	// Set selected full-scale (magnetometer)
	uint8_t CTRL6_val = (mag_range << 5);
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG6_XM, CTRL6_val);
	//
	// Bypass mode on FIFO
	uint8_t FIFO_CTRL_val = 0x0;
	writeRegister(_chipSelectPin_XM, LSM9DS0_FIFO_CTRL_REG, FIFO_CTRL_val);
	//
	// Clear the Reference/Offset registers
	writeRegister(_chipSelectPin_XM, LSM9DS0_OFFSET_X_L_M, 0x00);
	writeRegister(_chipSelectPin_XM, LSM9DS0_OFFSET_X_H_M, 0x00);
	writeRegister(_chipSelectPin_XM, LSM9DS0_OFFSET_Y_L_M, 0x00);
	writeRegister(_chipSelectPin_XM, LSM9DS0_OFFSET_Y_H_M, 0x00);
	writeRegister(_chipSelectPin_XM, LSM9DS0_OFFSET_Z_L_M, 0x00);
	writeRegister(_chipSelectPin_XM, LSM9DS0_OFFSET_Z_H_M, 0x00);
	writeRegister(_chipSelectPin_XM, LSM9DS0_REFERENCE_X, 0x00);
	writeRegister(_chipSelectPin_XM, LSM9DS0_REFERENCE_Y, 0x00);
	writeRegister(_chipSelectPin_XM, LSM9DS0_REFERENCE_Z, 0x00);
	// Set HP filter on accelerometer, magnetometer on continuous conversion
	_CTRL7_val_XM = 0x0;
	if (HP_accel_enable){
		_CTRL7_val_XM = (1 << 5);
	}
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG7_XM, _CTRL7_val_XM);
	//
	// Set selected ODR (accelerometer), continuous update, turn on the accelerometer
	_CTRL1_val_XM = (accel_odr << 4) | 0x7;
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG1_XM, _CTRL1_val_XM);
	delay(200);
	// Discard the first n measures
	if(! discard_measures_accel(LSM9DS0_DISCARDED_MEASURES, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	// Discard the first n measures
	if(! discard_measures_mag(LSM9DS0_DISCARDED_MEASURES, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;

}

//----------------Turn on accelerometer----------------//
void LSM9DS0::turn_on_accel(){
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG1_XM, _CTRL1_val_XM);
	delay(200);
}

//----------------Turn off accelerometer---------------//
void LSM9DS0::turn_off_accel(){
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG1_XM, (_CTRL1_val_XM & 0xF));
}

//-----------------Read accelerometer------------------//
uint8_t LSM9DS0::read_raw_accel(){
	uint8_t buffer[6];
	readMultipleRegisters(_chipSelectPin_XM, buffer, 6, LSM9DS0_OUT_X_L_A);
	ax = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_a);
	ay = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_a);
	az = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_a);
	return 1;
}

//------------Read accelerometer when ready--------------//
uint8_t LSM9DS0::read_accel_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin_A) == 1){
			read_raw_accel();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 0;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS0::read_accel_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin_XM, LSM9DS0_STATUS_REG_A);
		if ((STATUS_val & (1 << 3)) == (1 << 3)){
			read_raw_accel();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//--------------------Check biases------------------------//
uint8_t LSM9DS0::check_accel_biases(float bx, float by, float bz){
	float thrs = 60e-3 * INS_G_VAL * 1.1; //typical 60mg zero-G level
	if ((fabs(bx) > thrs) || (fabs(by) > thrs) || (fabs(bz) > thrs)){
		return 0;
	}
	return 1;
}

//---------------Reset high-pass filter-----------------//		
void LSM9DS0::HP_reset_accel(){
	readRegister(_chipSelectPin_XM, LSM9DS0_REFERENCE_X);
	readRegister(_chipSelectPin_XM, LSM9DS0_REFERENCE_Y);
	readRegister(_chipSelectPin_XM, LSM9DS0_REFERENCE_Z);
}

//------------Self-test the accelerometer---------------//
uint8_t LSM9DS0::self_test_accel(uint8_t mode){ //self-test: mode 0 - X, Y, Z positive, mode 1 - X, Y, Z negative
	uint8_t status = 0;
	// Discard the first n measures
	if( ! discard_measures_accel(LSM9DS0_DISCARDED_MEASURES_ST, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (int ii = 0; ii < LSM9DS0_ACCEL_SELF_TEST_MEASURES; ii++){
		read_accel_STATUS(LSM9DS0_DISCARD_TIMEOUT);
		x_pre += ax;
		y_pre += ay;
		z_pre += az;
	}
	x_pre /= LSM9DS0_ACCEL_SELF_TEST_MEASURES;
	y_pre /= LSM9DS0_ACCEL_SELF_TEST_MEASURES;
	z_pre /= LSM9DS0_ACCEL_SELF_TEST_MEASURES;
	// Turn off the sensor and setup self-test
	turn_off_accel();
	uint8_t CTRL2_val = readRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG2_XM);
	if (mode == 0){
		CTRL2_val |= (1 << 1); // Self-test mode 0 (positive)
	}
	else {
		CTRL2_val |= (1 << 2); // Self-test mode 1 (negative)
	}
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG2_XM, CTRL2_val);
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(LSM9DS0_DISCARDED_MEASURES_ST,LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (int ii = 0; ii < LSM9DS0_ACCEL_SELF_TEST_MEASURES; ii++){
		read_accel_STATUS(LSM9DS0_DISCARD_TIMEOUT);
		x_post += ax;
		y_post += ay;
		z_post += az;
	}
	x_post /= LSM9DS0_ACCEL_SELF_TEST_MEASURES;
	y_post /= LSM9DS0_ACCEL_SELF_TEST_MEASURES;
	z_post /= LSM9DS0_ACCEL_SELF_TEST_MEASURES;
	// Define Threshold based on the Full-Scale value
	float thrs_min, thrs_max;
	if ((_sc_fact_a - INS_G_VAL * 0.00006103515625) < 1e-5){
		thrs_min = 60e-3 * INS_G_VAL;
		thrs_max = 1700e-3 * INS_G_VAL;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.0001220703125) < 1e-5){
		thrs_min = 60e-3 * INS_G_VAL;
		thrs_max = 1700e-3 * INS_G_VAL;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.00018310546875) < 1e-5){
		thrs_min = 60e-3 * INS_G_VAL;
		thrs_max = 1700e-3 * INS_G_VAL;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.000244140625) < 1e-5){
		thrs_min = 60e-3 * INS_G_VAL;
		thrs_max = 1700e-3 * INS_G_VAL;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.000732421875) < 1e-5){
		thrs_min = 60e-3 * INS_G_VAL;
		thrs_max = 1700e-3 * INS_G_VAL;
	}
	else {
		return 0;
	}
	// Check if values are bigger than the threshold
	if (ch_st(x_pre, x_post, thrs_min, thrs_max) && ch_st(y_pre, y_post, thrs_min, thrs_max) && ch_st(z_pre, z_post, thrs_min, thrs_max)) {
		status = 1;
	}
	/*if (mode == 0){
		if (((ax_post - ax_pre) > thrs_xy) && ((ay_post - ay_pre) > thrs_xy) && ((az_post - az_pre) > thrs_z)){
			status = 1;
		}
	}
	else {
		if (((ax_post - ax_pre) < - thrs_xy) && ((ay_post - ay_pre) < - thrs_xy) && ((az_post - az_pre) < - thrs_z)){
			status = 1;
		}
	}*/
	turn_off_accel();
	CTRL2_val &= 0xF9; // Removes Self-Test
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG2_XM, CTRL2_val);
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(LSM9DS0_DISCARDED_MEASURES_ST, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Accel Status------------------------//
uint8_t LSM9DS0::status_accel(){
	return readRegister(_chipSelectPin_XM, LSM9DS0_STATUS_REG_A);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS0::discard_measures_accel(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	while (count < (number_of_measures * 0.5)){
		uint8_t STATUS_value = status_accel();
		if ((STATUS_value & (1 << 7)) == (1 << 7)){
			read_raw_accel();
			now = micros();
			count++;
		}
		if ((micros()-now) > timeout){
			return 0;
		}
		else if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 1;
}

//=============================Public Members Magnetometer===================================//
//-----------------Turn on magnetometer----------------//
void LSM9DS0::turn_on_mag(){
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG7_XM, _CTRL7_val_XM);
	delay(200);
}

//-----------------Turn off magnetometer---------------//
void LSM9DS0::turn_off_mag(){
	writeRegister(_chipSelectPin_XM, LSM9DS0_CTRL_REG7_XM, (_CTRL7_val_XM | 0x3));
}

//------------------Read magnetometer------------------//
uint8_t LSM9DS0::read_raw_mag(){
	uint8_t buffer[6];
	readMultipleRegisters(_chipSelectPin_XM, buffer, 6, LSM9DS0_OUT_X_L_M);
	mx = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_m);
	my = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_m);
	mz = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_m);
}

//------------Read magnetometer when ready-------------//
uint8_t LSM9DS0::read_mag_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin_M) == 1){
			read_raw_mag();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 0;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS0::read_mag_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin_XM, LSM9DS0_STATUS_REG_M);
		if ((STATUS_val & (1 << 3)) == (1 << 3)){
			read_raw_mag();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//-----------------------Mag Status------------------------//
uint8_t LSM9DS0::status_mag(){
	return readRegister(_chipSelectPin_XM, LSM9DS0_STATUS_REG_M);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS0::discard_measures_mag(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	uint32_t now = micros();
	while (count < (number_of_measures * 0.5)){
		uint8_t STATUS_value = status_mag();
		if ((STATUS_value & (1 << 7)) == (1 << 7)){
			read_raw_mag();
			now = micros();
			count++;
		}
		if ((micros() - now) > timeout){
			return 0;
		}
		else if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 1;
}

//=============================Public Members Temperature====================================//
//------------------------Read data----------------------//
uint8_t LSM9DS0::read_raw_thermo(){
	uint8_t buffer[2];
	readMultipleRegisters(_chipSelectPin_XM,buffer, 2, LSM9DS0_OUT_TEMP_L_XM);
	int16_t temperature_tmp = (((int16_t) buffer[1] << 12) | buffer[0] << 4) >>4;
	temperature = (float) 20 + temperature_tmp * 0.125; // Guessing that the intercept is at about 20°C 
	return 1;
}

//------------------Read data when ready-----------------//
uint8_t LSM9DS0::read_thermo_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin_M) == 1){
			read_raw_thermo();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
	return 0;
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS0::read_thermo_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin_XM, LSM9DS0_STATUS_REG_M);
		if ((STATUS_val & (1 << 3)) == (1 << 3)){
			read_raw_thermo();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
return 0;
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS0::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	while (count < (number_of_measures)){
		if(read_thermo_STATUS(timeout)){
			count++;
		}
		else{
			return 0;
		}
	}
	return 1;
}