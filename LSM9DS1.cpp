//
// LSM9DS1.cpp
//
//
// Created by Andrea Vivani on 11/7/15.
// Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "LSM9DS1.h"
#include "Arduino.h"
#include <SPI.h>

//====================================Registers Addresses=========================================// 
//ACCELEROMETER AND GYROSCOPE
#define LSM9DS1_ACT_THS 		0x04
#define LSM9DS1_ACT_DUR 		0x05
#define LSM9DS1_INT_GEN_CFG_XL	0x06
#define LSM9DS1_INT_GEN_THS_X_XL	0x07
#define LSM9DS1_INT_GEN_THS_Y_XL	0x08
#define LSM9DS1_INT_GEN_THS_Z_XL	0x09
#define LSM9DS1_INT_GEN_DUR_XL	0x0A
#define LSM9DS1_REFERENCE_G 	0x0B
#define LSM9DS1_INT1_CTRL		0x0C
#define LSM9DS1_INT2_CTRL		0x0D
#define LSM9DS1_WHO_AM_I 		0x0F
#define LSM9DS1_CTRL_REG1_G 	0x10
#define LSM9DS1_CTRL_REG2_G 	0x11
#define LSM9DS1_CTRL_REG3_G 	0x12
#define LSM9DS1_ORIENT_CFG_G 	0x13
#define LSM9DS1_INT_GEN_SRC_G	0x14
#define LSM9DS1_OUT_TEMP_L 		0x15
#define LSM9DS1_OUT_TEMP_H 		0x16
#define LSM9DS1_STATUS_REG		0x17
#define LSM9DS1_OUT_X_L_G		0x18
#define LSM9DS1_OUT_X_H_G		0x19
#define LSM9DS1_OUT_Y_L_G		0x1A
#define LSM9DS1_OUT_Y_H_G		0x1B
#define LSM9DS1_OUT_Z_L_G		0x1C
#define LSM9DS1_OUT_Z_H_G		0x1D
#define LSM9DS1_CTRL_REG4 		0x1E
#define LSM9DS1_CTRL_REG5_XL 	0x1F
#define LSM9DS1_CTRL_REG6_XL 	0x20
#define LSM9DS1_CTRL_REG7_XL 	0x21
#define LSM9DS1_CTRL_REG8 		0x22
#define LSM9DS1_CTRL_REG9		0x23
#define LSM9DS1_CTRL_REG10 		0x24
#define LSM9DS1_INT_GEN_SRC_XL	0x26
#define LSM9DS1_STATUS_REG_XL 	0x27
#define LSM9DS1_OUT_X_L_XL		0x28
#define LSM9DS1_OUT_X_H_XL		0x29
#define LSM9DS1_OUT_Y_L_XL		0x2A
#define LSM9DS1_OUT_Y_H_XL		0x2B
#define LSM9DS1_OUT_Z_L_XL		0x2C
#define LSM9DS1_OUT_Z_H_XL		0x2D
#define LSM9DS1_FIFO_CTRL		0x2E
#define LSM9DS1_FIFO_SRC		0x2F
#define LSM9DS1_INT_GEN_CFG_G	0x30
#define LSM9DS1_INT_GEN_THS_XH_G	0x31
#define LSM9DS1_INT_GEN_THS_XL_G	0x32
#define LSM9DS1_INT_GEN_THS_YH_G	0x33
#define LSM9DS1_INT_GEN_THS_YL_G	0x34
#define LSM9DS1_INT_GEN_THS_ZH_G	0x35
#define LSM9DS1_INT_GEN_THS_ZL_G	0x36
#define LSM9DS1_INT_GEN_DUR_G	0x37
//MAGNETOMETER
#define LSM9DS1_OFFSET_X_REG_L_M	0x05
#define LSM9DS1_OFFSET_X_REG_H_M	0x06
#define LSM9DS1_OFFSET_Y_REG_L_M	0x07
#define LSM9DS1_OFFSET_Y_REG_H_M	0x08
#define LSM9DS1_OFFSET_Z_REG_L_M	0x09
#define LSM9DS1_OFFSET_Z_REG_H_M	0x0A
#define LSM9DS1_WHO_AM_I_M 		0x0F
#define LSM9DS1_CTRL_REG1_M 	0x20
#define LSM9DS1_CTRL_REG2_M		0x21
#define LSM9DS1_CTRL_REG3_M 	0x22
#define LSM9DS1_CTRL_REG4_M 	0x23
#define LSM9DS1_CTRL_REG5_M 	0x24
#define LSM9DS1_STATUS_REG_M 	0x27
#define LSM9DS1_OUT_X_L_M		0x28
#define LSM9DS1_OUT_X_H_M		0x29
#define LSM9DS1_OUT_Y_L_M		0x2A
#define LSM9DS1_OUT_Y_H_M		0x2B
#define LSM9DS1_OUT_Z_L_M		0x2C
#define LSM9DS1_OUT_Z_H_M		0x2D
#define LSM9DS1_INT_CFG_M		0x30
#define LSM9DS1_INT_SRC_M		0x31
#define LSM9DS1_INT_THS_L_M		0x32
#define LSM9DS1_INT_THS_H_M		0x33
//=======================================Constants=============================================// 
#define LSM9DS1_ID_XG		0x68
#define LSM9DS1_ID_M		0x3D
#define LSM9DS1_READ		0x80
#define LSM9DS1_MULT		0x40
//==================================Auxiliary Functions========================================//
//---------------Read one register from the SPI-----------------//
uint8_t LSM9DS1::readRegister(uint8_t chipSelectPin, uint8_t thisRegister) {
	uint8_t inByte = 0;   	// incoming byte
	thisRegister |= LSM9DS1_READ;		// register in read mode
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(thisRegister);		// send the command to read thisRegister
	inByte = SPI.transfer(0x00);		// send 0x00 in order to read the incoming byte
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	return(inByte);			// return the read byte
}

//---Read multiple registers from the SPI (only Magnetometer)---//
void LSM9DS1::readMultipleRegisters_M(uint8_t chipSelectPin, uint8_t *buffer, uint8_t number_of_registers, uint8_t startRegister) {
	startRegister |= (LSM9DS1_READ | LSM9DS1_MULT);// register in multiple read mode
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(startRegister);		// send the command to read thisRegister
	for (uint8_t ii = 0; ii < number_of_registers; ii++){
		buffer[ii] = SPI.transfer(0x00);
	}
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	return;
}

//--Read multiple registers from the SPI (except Magnetometer)--//
void LSM9DS1::readMultipleRegisters(uint8_t chipSelectPin, uint8_t *buffer, uint8_t number_of_registers, uint8_t startRegister) {
	startRegister |= LSM9DS1_READ;// register in multiple read mode
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(startRegister);		// send the command to read thisRegister
	for (uint8_t ii = 0; ii < number_of_registers; ii++){
		buffer[ii] = SPI.transfer(0x00);
	}
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	return;
}

//---------------Write one register on the SPI-----------------//
void LSM9DS1::writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue) {
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(thisRegister); 		// send register location
	SPI.transfer(thisValue); 		// send value to record into register
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	return;
}

//-----------------Check values for self-test-------------------//
uint8_t LSM9DS1::ch_st (const double val1, const double val2, const double lim1, const double lim2){
    if (fabs(lim1) > fabs(lim2)){
        return ((fabs(val2 - val1) >= fabs(lim2)) && (fabs(val2 - val1) <= fabs(lim1)));
    }
    return ((fabs(val2 - val1) >= fabs(lim1)) && (fabs(val2 - val1) <= fabs(lim2)));
    
}

//=====================================Constructors==========================================//
LSM9DS1::LSM9DS1 (uint8_t CS_pin_XG, uint8_t CS_pin_M):InertialSensor(){
	_chipSelectPin_XG = CS_pin_XG;
	_chipSelectPin_M = CS_pin_M;
	_DRDY_pin_G = 0;
	_DRDY_pin_A = 0;
	_DRDY_pin_M = 0;
}

LSM9DS1::LSM9DS1 (uint8_t CS_pin_XG, uint8_t CS_pin_M, uint8_t DRDY_pin_G, uint8_t DRDY_pin_A, uint8_t DRDY_pin_M):InertialSensor(){
	_chipSelectPin_XG = CS_pin_XG;
	_chipSelectPin_M = CS_pin_M;
	_DRDY_pin_G = DRDY_pin_G;
	_DRDY_pin_A = DRDY_pin_A;
	_DRDY_pin_M = DRDY_pin_M;
}

//-----------------------Initialization-----------------------//
void LSM9DS1::init(){
	pinMode(_chipSelectPin_XG,OUTPUT);
	pinMode(_chipSelectPin_M,OUTPUT);
	digitalWrite(_chipSelectPin_XG,HIGH);
	digitalWrite(_chipSelectPin_M,HIGH);
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
uint8_t LSM9DS1::config_accel_gyro(uint8_t gyro_range, uint8_t gyro_odr, uint8_t LPF2_enable_gyro, uint8_t HP_enable_gyro, uint8_t HP_freq_gyro, uint8_t accel_range, uint8_t accel_odr, uint8_t LPF_enable_accel, uint8_t LPF_freq_accel, uint8_t HP_enable_accel, uint8_t HP_freq_accel){
	init();
	//
	// Trash the first reading
	readRegister(_chipSelectPin_XG, LSM9DS1_WHO_AM_I);
	//
	// Check if the device ID is correct
	if (readRegister(_chipSelectPin_XG, LSM9DS1_WHO_AM_I) != LSM9DS1_ID_XG){
		return 0;
	}
	//
	// Gyroscope DRDY on INT1
	uint8_t INT1_CTRL_val = (1 << 1);
	writeRegister(_chipSelectPin_XG, LSM9DS1_INT1_CTRL, INT1_CTRL_val);
	//
	// Accelerometer DRDY on INT2
	uint8_t INT2_CTRL_val = 0x01;
	writeRegister(_chipSelectPin_XG, LSM9DS1_INT2_CTRL, INT2_CTRL_val);
	//
	uint8_t CTRL2_val;
	uint8_t CTRL3_val;
	if (HP_enable_gyro){
		//high-power mode, set High-pass filter frequency
		CTRL3_val = (1 << 6) | HP_freq_gyro;
		//high-pass filter on both output and interrupt
		CTRL2_val = (1 << 2) | 0x01;
	}
	else{
		//high-power mode, no High-pass filter
		CTRL2_val = 0x00;
		//no high-pass filter
		CTRL3_val = 0x00;
	}
	//
	if (LPF2_enable_gyro){
		CTRL2_val |= (1 << 3) | (1 << 1);
	}
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG2_G, CTRL2_val);
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG3_G, CTRL3_val);
	//
	// Enable X, Y, and Z gyro axes
	uint8_t CTRL4_val = (1 << 3) | (1 << 4) | (1 << 5);
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG4, CTRL4_val);
	//
	// No decimation, enable X, Y, Z accel axes
	uint8_t CTRL5_val = (1 << 3) | (1 << 4) | (1 << 5);
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG5_XL, CTRL5_val);
	//
	// Continuous update, interrupt active high, push-pull, SPI 4-wire, Auto-increment, Little endian
	uint8_t CTRL8_val = (1 << 2);
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG8, CTRL8_val);
	//
	// disabled I2C
	uint8_t CTRL9_val = (1 << 2);
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG9, CTRL9_val);
	//
	// CTRL_REG10 is kept as default to 0x00 (no self-test)
	//
	// Selected gyro ODR and range
	switch (gyro_range){
		case (LSM9DS1_RANGE_G_245):
			_sc_fact_g = 8.75e-3 * INS_TORAD;
			break;
		case (LSM9DS1_RANGE_G_500):
			_sc_fact_g = 17.5e-3 * INS_TORAD;
			break;
		case (LSM9DS1_RANGE_G_2000):
			_sc_fact_g = 70e-3 * INS_TORAD;
			break;
		default:
			return 2;
	}
	// selected accelerometer filter configuration
	uint8_t CTRL7_val_XL = 0;
	if (HP_enable_accel){ //high-pass filter enabled
		CTRL7_val_XL = (1 << 7) | (1 << 2) | 0x01 | HP_freq_accel;
	}
	else {
		CTRL7_val_XL = (1 << 7);
	}
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG7_XL, CTRL7_val_XL);
	// selected accelerometer ODR, range and auto anti-aliasing
	switch (accel_range){
		case (LSM9DS1_RANGE_A_2):
			_sc_fact_a = INS_G_VAL * 0.00006103515625;
			break;
		case (LSM9DS1_RANGE_A_4):
			_sc_fact_a = INS_G_VAL * 0.0001220703125;
			break;
		case (LSM9DS1_RANGE_A_8):
			_sc_fact_a = INS_G_VAL * 0.000244140625;
			break;
		case (LSM9DS1_RANGE_A_16):
			_sc_fact_a = INS_G_VAL * 0.000732421875;
			break;
		default:
			return 2;
	}
	_CTRL1_val_XG = gyro_range | gyro_odr;
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG1_G, _CTRL1_val_XG);
	//
	// gyro and accel operate at the same ODR if CTRL1 is used and CTRL6 is in Power-down
	_CTRL6_val_XL = accel_range | accel_odr;
	if (LPF_enable_accel){
		_CTRL6_val_XL |= (1 << 2) | LPF_freq_accel;
	}
	else {
		_CTRL6_val_XL &= 0xF8; 
	}
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG6_XL, _CTRL6_val_XL);
	delay(200);
	//
	// Discard the first n gyro measures
	if(! discard_measures_gyro(LSM9DS1_DISCARDED_MEASURES, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Discard the first n accel measures
	if(! discard_measures_accel(LSM9DS1_DISCARDED_MEASURES, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-------------------------Turn on---------------------------//
void LSM9DS1::turn_on_gyro(){
	uint8_t CTRL_REG9_val = readRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG9);
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG9, (CTRL_REG9_val & 0xBF));
	uint8_t CTRL9_val = readRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG9);
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG9, (CTRL9_val & 0x1F));
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG1_G, _CTRL1_val_XG);
	delay(200);
}

//------------------------Turn off---------------------------//
void LSM9DS1::turn_off_gyro(){
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG1_G,0x00);
}

//-------------------------Sleep-----------------------------//
void LSM9DS1::sleep_gyro(){
	uint8_t CTRL_REG9_val = readRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG9);
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG9, (CTRL_REG9_val | (1 << 6)));
}

//------------------------Read data-------------------------//
uint8_t LSM9DS1::read_raw_gyro(){
	uint8_t buffer[6];
	readMultipleRegisters(_chipSelectPin_XG, buffer, 6, LSM9DS1_OUT_X_L_G);
	gx = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_g);
	gy = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_g);
	gz = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_g);
	return 1;
}

//------------------Read data when ready--------------------//
uint8_t LSM9DS1::read_gyro_DRDY(uint32_t timeout){
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
uint8_t LSM9DS1::read_gyro_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin_XG, LSM9DS1_STATUS_REG);
		if ((STATUS_val & (1 << 1)) == (1 << 1)){
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
uint8_t LSM9DS1::check_gyro_biases(float bx, float by, float bz){
// Define Threshold based on Full-Scale value
	float thrs;
	if ((_sc_fact_g - 8.75e-3 * INS_TORAD) < 1e-5){
		thrs = 10 * INS_TORAD * 1.2; //typical 10dps offset
	}
	else if ((_sc_fact_g - 17.5e-3 * INS_TORAD) < 1e-5){
		thrs = 15 * INS_TORAD * 1.2;	//typical 15dps offset
	}
	else if ((_sc_fact_g - 70e-3 * INS_TORAD) < 1e-5){
		thrs = 30 * INS_TORAD * 1.2;	//typical 25dps offset
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
void LSM9DS1::HP_reset_gyro(){
	readRegister(_chipSelectPin_XG, LSM9DS1_REFERENCE_G);
}

//-----------------------Self-Test-------------------------//
uint8_t LSM9DS1::self_test_gyro(){
	uint8_t status = 0;
	// Discard the first n measures
	if(!discard_measures_gyro(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (int ii = 0; ii < LSM9DS1_GYRO_SELF_TEST_MEASURES; ii++){
		read_gyro_STATUS(LSM9DS1_DISCARD_TIMEOUT);
		x_pre += gx;
		y_pre += gy;
		z_pre += gz;
	}
	x_pre /= LSM9DS1_GYRO_SELF_TEST_MEASURES;
	y_pre /= LSM9DS1_GYRO_SELF_TEST_MEASURES;
	z_pre /= LSM9DS1_GYRO_SELF_TEST_MEASURES;
	// Turn on self-test
	turn_off_gyro();
	uint8_t CTRL10_val = readRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG10);
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG10, (CTRL10_val | (1 << 2)));
	turn_on_gyro();
	// Discard the first n measures 
	if(! discard_measures_gyro(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (int ii = 0; ii < LSM9DS1_GYRO_SELF_TEST_MEASURES; ii++){
		read_gyro_STATUS(LSM9DS1_DISCARD_TIMEOUT);
		x_post += gx;
		y_post += gy;
		z_post += gz;
	}
	x_post /= LSM9DS1_GYRO_SELF_TEST_MEASURES;
	y_post /= LSM9DS1_GYRO_SELF_TEST_MEASURES;
	z_post /= LSM9DS1_GYRO_SELF_TEST_MEASURES;
	// Define threshold based on the Full-Scale value
	float thrs;
	if ((_sc_fact_g - 8.75e-3 * INS_TORAD) < 1e-5){
		thrs = 20 * INS_TORAD; 
	}
	else if ((_sc_fact_g - 17.5e-3 * INS_TORAD) < 1e-5){
		thrs = 70 * INS_TORAD;
	}
	else if ((_sc_fact_g - 70e-3 * INS_TORAD) < 1e-5){
		thrs = 150 * INS_TORAD;
	}
	else {
		return 0;
	}
	// Check if values are bigger than the threshold
	if (ch_st(x_pre, x_post, (0.6 * thrs), (1.4 * thrs)) && ch_st(y_pre, y_post, (0.6 * thrs), (1.4 * thrs)) && ch_st(z_pre, z_post, (0.6 * thrs), (1.4 * thrs))) {
		status = 1;
	}
	/*if ((gx > thrs) && (gy < - thrs) && (gz < - thrs)){
		status = 1;
	}*/
	turn_off_gyro();
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG10, CTRL10_val);
	turn_on_gyro();
	// Discard the first n measures
	if(! discard_measures_gyro(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Gyro Status------------------------//
uint8_t LSM9DS1::status_gyro(){
	return readRegister(_chipSelectPin_XG, LSM9DS1_STATUS_REG);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS1::discard_measures_gyro(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	while (count < (number_of_measures)){
		if(! read_gyro_STATUS(timeout)){
			return 0;
		}
		count++;
	}
	return 1;
}

//=============================Public Members Accelerometer==================================== //
//----------------Turn on accelerometer----------------//
void LSM9DS1::turn_on_accel(){
	turn_on_gyro();
}

//----------------Turn off accelerometer---------------//
void LSM9DS1::turn_off_accel(){
	turn_off_gyro();
}

//-----------------Read accelerometer------------------//
uint8_t LSM9DS1::read_raw_accel(){
	uint8_t buffer[6];
	readMultipleRegisters(_chipSelectPin_XG, buffer, 6, LSM9DS1_OUT_X_L_XL);
	ax = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_a);
	ay = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_a);
	az = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_a);
	return 1;
}

//------------Read accelerometer when ready--------------//
uint8_t LSM9DS1::read_accel_DRDY(uint32_t timeout){
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
uint8_t LSM9DS1::read_accel_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin_XG, LSM9DS1_STATUS_REG);
		if ((STATUS_val & 0x01) == 0x01){
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
uint8_t LSM9DS1::check_accel_biases(float bx, float by, float bz){
	float thrs = 90e-3 * INS_G_VAL * 1.1; //typical 90mg zero-G level
	if ((fabs(bx) > thrs) || (fabs(by) > thrs) || (fabs(bz) > thrs)){
		return 0;
	}
	return 1;
}

//------------Self-test the accelerometer---------------//
uint8_t LSM9DS1::self_test_accel(){
	uint8_t status = 0;
	// Discard the first n measures
	if(! discard_measures_accel(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (int ii = 0; ii < LSM9DS1_ACCEL_SELF_TEST_MEASURES; ii++){
		read_accel_STATUS(LSM9DS1_DISCARD_TIMEOUT);
		x_pre += ax;
		y_pre += ay;
		z_pre += az;
	}
	x_pre /= LSM9DS1_ACCEL_SELF_TEST_MEASURES;
	y_pre /= LSM9DS1_ACCEL_SELF_TEST_MEASURES;
	z_pre /= LSM9DS1_ACCEL_SELF_TEST_MEASURES;
	// Turn off the sensor and setup self-test
	turn_off_accel();
	uint8_t CTRL10_val = readRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG10);
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG10, (CTRL10_val | 0x01));
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(LSM9DS1_DISCARDED_MEASURES_ST,LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (int ii = 0; ii < LSM9DS1_ACCEL_SELF_TEST_MEASURES; ii++){
		read_accel_STATUS(LSM9DS1_DISCARD_TIMEOUT);
		x_post += ax;
		y_post += ay;
		z_post += az;
	}
	x_post /= LSM9DS1_ACCEL_SELF_TEST_MEASURES;
	y_post /= LSM9DS1_ACCEL_SELF_TEST_MEASURES;
	z_post /= LSM9DS1_ACCEL_SELF_TEST_MEASURES;
	// Define Threshold based on the Full-Scale value
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
	/*float thrs_xy, thrs_z;
	if ((_sc_fact_a - INS_G_VAL * 0.00006103515625) < 1e-5){
		thrs_xy = 4;
		thrs_z = 2;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.0001220703125) < 1e-5){
		thrs_xy = 4;
		thrs_z = 2;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.000244140625) < 1e-5){
		thrs_xy = 4;
		thrs_z = 2;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.000732421875) < 1e-5){
		thrs_xy = 4;
		thrs_z = 2;
	}
	else {
		return 0;
	}*/
	// Check if values are bigger than the threshold
	if (ch_st(x_pre, x_post, thrs_min, thrs_max) && ch_st(y_pre, y_post, thrs_min, thrs_max) && ch_st(z_pre, z_post, thrs_min, thrs_max)) {
		status = 1;
	}
	/*if (((ax_post - ax_pre) > thrs_xy) && ((ay_post - ay_pre) > thrs_xy) && ((az_post - az_pre) > thrs_z)){
		status = 1;
	}*/
	turn_off_accel();
	writeRegister(_chipSelectPin_XG, LSM9DS1_CTRL_REG10, CTRL10_val);
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Accel Status------------------------//
uint8_t LSM9DS1::status_accel(){
	return readRegister(_chipSelectPin_XG, LSM9DS1_STATUS_REG);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS1::discard_measures_accel(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	while (count < (number_of_measures)){
		if(! read_accel_STATUS(timeout)){
			return 0;
		}
		count++;
	}
	return 1;
}

//=============================Public Members Magnetometer===================================//
//--------------------Configuration--------------------//
uint8_t LSM9DS1::config_mag(uint8_t range_conf, uint8_t odr_conf){
	init();
	// Trash the first reading
	readRegister(_chipSelectPin_M, LSM9DS1_WHO_AM_I_M);
	// Check if the device ID is correct
	if (readRegister(_chipSelectPin_M, LSM9DS1_WHO_AM_I_M) != LSM9DS1_ID_M){
		return 0;
	}
	//
	//selected range
	uint8_t CTRL2_val = range_conf & 0x60;
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG2_M, CTRL2_val);
	switch (range_conf){
		case (LSM9DS1_RANGE_M_4):
			_sc_fact_m = 1.0f / 6842.0;
			break;
		case (LSM9DS1_RANGE_M_8):
			_sc_fact_m = 1.0f / 3421.0;
			break;
		case (LSM9DS1_RANGE_M_12):
			_sc_fact_m = 1.0f / 2281.0;
			break;
		case (LSM9DS1_RANGE_M_16):
			_sc_fact_m = 1.0f / 1711.0;
			break;
		default:
		return 2;
	}
	//
	//continuous update
	uint8_t CTRL5_val = 0x00;
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG5_M, CTRL5_val);
	//
	//Z-axis on ultra-high performance mode, little endian
	uint8_t CTRL4_val = (1 << 3) | (1 << 2);
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG4_M, CTRL4_val);
	//
	//temperature enable, ultra-high perfomance mode, selected ODR, no self test
	uint8_t CTRL1_val = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 1) | odr_conf;
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG1_M, CTRL1_val);
	//
	//power on, SPI 4 wire, Continuous conversion mode
	_CTRL3_val_M = (1 << 7);
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG3_M, _CTRL3_val_M);
	delay(200);
	// Discard the first n measures
	if(! discard_measures_mag(LSM9DS1_DISCARDED_MEASURES, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-----------------Turn on magnetometer----------------//
void LSM9DS1::turn_on_mag(){
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG3_M, _CTRL3_val_M);
	delay(200);
}

//-----------------Turn off magnetometer---------------//
void LSM9DS1::turn_off_mag(){
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG3_M, (_CTRL3_val_M | 0x3));
}

//------------------Read magnetometer------------------//
uint8_t LSM9DS1::read_raw_mag(){
	uint8_t buffer[6];
	readMultipleRegisters_M(_chipSelectPin_M, buffer, 6, LSM9DS1_OUT_X_L_M);
	mx = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_m);
	my = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_m);
	mz = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_m);
}

//------------Read magnetometer when ready-------------//
uint8_t LSM9DS1::read_mag_DRDY(uint32_t timeout){
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
uint8_t LSM9DS1::read_mag_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin_M, LSM9DS1_STATUS_REG_M);
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

uint8_t LSM9DS1::self_test_mag(){
	uint8_t status = 0;
	// Use FS = 12 Gauss
	uint8_t CTRL2_val_old = readRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG2_M);
	uint8_t CTRL2_val = LSM9DS1_RANGE_M_12 & 0x60;
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG2_M, CTRL2_val);
	// Discard the first n measures
	if(!discard_measures_mag(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (int ii = 0; ii < LSM9DS1_MAG_SELF_TEST_MEASURES; ii++){
		read_mag_STATUS(LSM9DS1_DISCARD_TIMEOUT);
		x_pre += mx;
		y_pre += my;
		z_pre += mz;
	}
	x_pre /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0); // average and revert to FS = 12 Gauss
	y_pre /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0); // average and revert to FS = 12 Gauss
	z_pre /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0); // average and revert to FS = 12 Gauss
	// Turn on self-test
	turn_off_mag();
	// Enable the self-test
	uint8_t CTRL1_val = readRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG1_M);
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG1_M, (CTRL1_val | 0x01));
	turn_on_mag();
	// Discard the first n measures 
	if(! discard_measures_mag(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (int ii = 0; ii < LSM9DS1_MAG_SELF_TEST_MEASURES; ii++){
		read_mag_STATUS(LSM9DS1_DISCARD_TIMEOUT);
		x_post += mx;
		y_post += my;
		z_post += mz;
	}
	x_post /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0); // average and revert to FS = 12 Gauss
	y_post /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0); // average and revert to FS = 12 Gauss
	z_post /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0); // average and revert to FS = 12 Gauss
	float thrs_xy_min = 1.0;
	float thrs_xy_max = 3.0;
	float thrs_z_min = 0.1;
	float thrs_z_max = 1.0;
	if (ch_st(x_pre, x_post, thrs_xy_min, thrs_xy_max) && ch_st(y_pre, y_post, thrs_xy_min, thrs_xy_max) && ch_st(z_pre, z_post, thrs_z_min, thrs_z_max)) {
		status = 1;
	}

	// Define Threshold based on datasheet (quite high...it could be between 1 and 3 (x and y) and between 0.1 and 1 (z))
	/*float thrs_xy = 2;
	float thrs_z = 0.5;
	// Check if values are bigger than the threshold
	if ((fabs(mx) > thrs_xy) && (fabs(my) > thrs_xy) && (fabs(mz) > thrs_z)){
		status = 1;
	}*/
	turn_off_mag();
	//remove self test
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG1_M, CTRL1_val);
	// Reset correct FS value
	writeRegister(_chipSelectPin_M, LSM9DS1_CTRL_REG2_M, CTRL2_val_old);
	turn_on_mag();
	// Discard the first n measures
	if(! discard_measures_mag(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//-----------------------Mag Status------------------------//
uint8_t LSM9DS1::status_mag(){
	return readRegister(_chipSelectPin_M, LSM9DS1_STATUS_REG_M);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS1::discard_measures_mag(uint8_t number_of_measures, uint32_t timeout){
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
uint8_t LSM9DS1::read_raw_thermo(){
	uint8_t buffer[2];
	readMultipleRegisters(_chipSelectPin_XG, buffer, 2, LSM9DS1_OUT_TEMP_L);
	int16_t temperature_tmp = (((int16_t) buffer[1] << 12) | buffer[0] << 4) >>4;
	temperature = (float) 20 + temperature_tmp * 0.125; // Guessing that the intercept is at about 20°C 
	return 1;
}

//------------------Read data when ready-----------------//
uint8_t LSM9DS1::read_thermo_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin_A) == 1){
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
uint8_t LSM9DS1::read_thermo_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin_XG, LSM9DS1_STATUS_REG);
		if ((STATUS_val & (1 << 2)) == (1 << 2)){
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
uint8_t LSM9DS1::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	while (count < (number_of_measures)){
		if(! read_thermo_STATUS(timeout)){
			return 0;
		}
		count++;
	}
	return 1;
}