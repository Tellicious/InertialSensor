//
// LSM6DS3.cpp
//
//
// Created by Andrea Vivani on 22/8/15.
// Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "LSM6DS3.h"
#include "Arduino.h"
#include <SPI.h>
//====================================Registers Addresses=========================================// 
#define LSM6DS3_FUNC_CFG_ACCESS 		0x01
#define LSM6DS3_SENSOR_SYNC_TIME_FRAME 		0x04
#define LSM6DS3_FIFO_CTRL1			0x06
#define LSM6DS3_FIFO_CTRL2			0x07
#define LSM6DS3_FIFO_CTRL3			0x08
#define LSM6DS3_FIFO_CTRL4			0x09
#define LSM6DS3_FIFO_CTRL5			0x0A
#define LSM6DS3_ORIENT_CFG_G 			0x0B
#define LSM6DS3_INT1_CTRL			0x0D
#define LSM6DS3_INT2_CTRL			0x0E
#define LSM6DS3_WHO_AM_I 			0x0F
#define LSM6DS3_CTRL1_XL 			0x10
#define LSM6DS3_CTRL2_G 			0x11
#define LSM6DS3_CTRL3_C 			0x12
#define LSM6DS3_CTRL4_C 			0x13
#define LSM6DS3_CTRL5_C				0x14
#define LSM6DS3_CTRL6_C 			0x15
#define LSM6DS3_CTRL7_G 			0x16
#define LSM6DS3_CTRL8_XL			0x17
#define LSM6DS3_CTRL9_XL			0x18
#define LSM6DS3_CTRL10_C			0x19
#define LSM6DS3_MASTER_CONFIG			0x1A
#define LSM6DS3_WAKE_UP_SRC			0x1B
#define LSM6DS3_TAP_SRC				0x1C
#define LSM6DS3_D6D_SRC				0x1D
#define LSM6DS3_STATUS_REG 			0x1E
#define LSM6DS3_OUT_TEMP_L 			0x20
#define LSM6DS3_OUT_TEMP_H 			0x21
#define LSM6DS3_OUTX_L_G 			0x22
#define LSM6DS3_OUTX_H_G			0x23
#define LSM6DS3_OUTY_L_G 			0x24
#define LSM6DS3_OUTY_H_G 			0x25
#define LSM6DS3_OUTZ_L_G			0x26
#define LSM6DS3_OUTZ_H_G 			0x27
#define LSM6DS3_OUTX_L_XL			0x28
#define LSM6DS3_OUTX_H_XL			0x29
#define LSM6DS3_OUTY_L_XL			0x2A
#define LSM6DS3_OUTY_H_XL			0x2B
#define LSM6DS3_OUTZ_L_XL			0x2C
#define LSM6DS3_OUTZ_H_XL			0x2D
#define LSM6DS3_SENSORHUB1_REG			0x2E
#define LSM6DS3_SENSORHUB2_REG			0x2F
#define LSM6DS3_SENSORHUB3_REG			0x30
#define LSM6DS3_SENSORHUB4_REG			0x31
#define LSM6DS3_SENSORHUB5_REG			0x32
#define LSM6DS3_SENSORHUB6_REG			0x33
#define LSM6DS3_SENSORHUB7_REG			0x34
#define LSM6DS3_SENSORHUB8_REG			0x35
#define LSM6DS3_SENSORHUB9_REG			0x36
#define LSM6DS3_SENSORHUB10_REG			0x37
#define LSM6DS3_SENSORHUB11_REG			0x38
#define LSM6DS3_SENSORHUB12_REG			0x39
#define LSM6DS3_FIFO_STATUS1			0x3A
#define LSM6DS3_FIFO_STATUS2			0x3B
#define LSM6DS3_FIFO_STATUS3			0x3C
#define LSM6DS3_FIFO_STATUS4			0x3D
#define LSM6DS3_FIFO_DATA_OUT_L			0x3E
#define LSM6DS3_FIFO_DATA_OUT_H			0x3F
#define LSM6DS3_TIMESTAMP0_REG			0x40
#define LSM6DS3_TIMESTAMP1_REG			0x41
#define LSM6DS3_TIMESTAMP2_REG			0x42
#define LSM6DS3_STEP_TIMESTAMP_L		0x49
#define LSM6DS3_STEP_TIMESTAMP_H		0x4A
#define LSM6DS3_STEP_COUNTER_L			0x4B
#define LSM6DS3_STEP_COUNTER_H			0x4C
#define LSM6DS3_SENSORHUB13_REG			0x4D
#define LSM6DS3_SENSORHUB14_REG			0x4E
#define LSM6DS3_SENSORHUB15_REG			0x4F
#define LSM6DS3_SENSORHUB16_REG			0x50
#define LSM6DS3_SENSORHUB17_REG			0x51
#define LSM6DS3_SENSORHUB18_REG			0x52
#define LSM6DS3_FUNC_SRC			0x53
#define LSM6DS3_TAP_CFG				0x58
#define LSM6DS3_TAP_THS_6D			0x59
#define LSM6DS3_INT_DUR2			0x5A
#define LSM6DS3_WAKE_UP_THS			0x5B
#define LSM6DS3_WAKE_UP_DUR			0x5C
#define LSM6DS3_FREE_FALL			0x5D
#define LSM6DS3_MD1_CFG				0x5E
#define LSM6DS3_MD2_CFG				0x5F
#define LSM6DS3_OUT_MAG_RAW_X_L			0x66
#define LSM6DS3_OUT_MAG_RAW_X_H			0x67
#define LSM6DS3_OUT_MAG_RAW_Y_L			0x68
#define LSM6DS3_OUT_MAG_RAW_Y_H			0x69
#define LSM6DS3_OUT_MAG_RAW_Z_L			0x6A
#define LSM6DS3_OUT_MAG_RAW_Z_H			0x6B
//=======================================Constants=============================================// 
#define LSM6DS3_ID				0x69
#define LSM6DS3_READ				0x80
//==================================Auxiliary Functions========================================//
//---------------Read one register from the SPI-----------------//
uint8_t LSM6DS3::readRegister(uint8_t chipSelectPin, uint8_t thisRegister) {
	uint8_t inByte = 0;   	// incoming byte
	thisRegister |= LSM6DS3_READ;		// register in read mode
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(thisRegister);		// send the command to read thisRegister
	inByte = SPI.transfer(0x00);		// send 0x00 in order to read the incoming byte
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	return(inByte);			// return the read byte
}

//------------Read multiple registers from the SPI--------------//
void LSM6DS3::readMultipleRegisters(uint8_t chipSelectPin, uint8_t * buffer, uint8_t number_of_registers, uint8_t startRegister) {
	startRegister |= (LSM6DS3_READ);// register in multiple read mode
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(startRegister);		// send the command to read thisRegister
	while (number_of_registers--){
		*buffer++ = SPI.transfer(0x00);
	}
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	return;
}

//---------------Write one register on the SPI-----------------//
void LSM6DS3::writeRegister(uint8_t chipSelectPin, uint8_t thisRegister, const uint8_t thisValue) {
	digitalWrite(chipSelectPin, LOW);	// ChipSelect low to select the chip
	SPI.transfer(thisRegister); 		// send register location
	SPI.transfer(thisValue); 		// send value to record into register
	digitalWrite(chipSelectPin, HIGH);	// ChipSelect high to deselect the chip
	return;
}

//-----------------Check values for self-test-------------------//
uint8_t LSM6DS3::ch_st (const float val1, const float val2, const float lim1, const float lim2){
    if (fabs(lim1) > fabs(lim2)){
        return ((fabs(val2 - val1) >= fabs(lim2)) && (fabs(val2 - val1) <= fabs(lim1)));
    }
    return ((fabs(val2 - val1) >= fabs(lim1)) && (fabs(val2 - val1) <= fabs(lim2)));
    
}

//=====================================Constructors==========================================//
LSM6DS3::LSM6DS3 (uint8_t CS_pin):InertialSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin_G = 0;
	_DRDY_pin_A = 0;
}

LSM6DS3::LSM6DS3 (uint8_t CS_pin, uint8_t DRDY_pin_G, uint8_t DRDY_pin_A):InertialSensor(){
	_chipSelectPin = CS_pin;
	_DRDY_pin_G = DRDY_pin_G;
	_DRDY_pin_A = DRDY_pin_A;
}

//-----------------------Initialization-----------------------//
void LSM6DS3::init(){
	pinMode(_chipSelectPin,OUTPUT);
	digitalWrite(_chipSelectPin,HIGH);
	if ((_DRDY_pin_G != 0) || (_DRDY_pin_A != 0)){
		pinMode(_DRDY_pin_G,INPUT);
		pinMode(_DRDY_pin_A,INPUT);
	}
	// initialize variables
	gx = 0;
	gy = 0;
	gz = 0;
	ax = 0;
	ay = 0;
	az = 0;
	temperature = 0;
}

//=================================Public Members Gyro=======================================//
//-----------------------Configuration-----------------------//
uint8_t LSM6DS3::config_accel_gyro(uint8_t gyro_range, uint8_t gyro_odr, uint8_t HP_enable_gyro, uint8_t HP_freq_gyro, uint8_t accel_range, uint8_t accel_odr, uint8_t LPF_enable_accel, uint8_t LPF_freq_accel, uint8_t HP_enable_accel, uint8_t HP_freq_accel){
	init();
	//
	// Trash the first reading
	readRegister(_chipSelectPin, LSM6DS3_WHO_AM_I);
	//
	// Check if the device ID is correct
	if (readRegister(_chipSelectPin, LSM6DS3_WHO_AM_I) != LSM6DS3_ID){
		delay(50);
		return 0;
	}
	// Disable access to embedded functions
	writeRegister(_chipSelectPin, LSM6DS3_FUNC_CFG_ACCESS, 0x00);
	// Disable FIFO
	writeRegister(_chipSelectPin, LSM6DS3_FIFO_CTRL5, 0x00);
	//
	// Gyroscope DRDY on INT1
	uint8_t INT1_CTRL_val = (1 << 1);
	writeRegister(_chipSelectPin, LSM6DS3_INT1_CTRL, INT1_CTRL_val);
	//
	// Accelerometer DRDY on INT2
	uint8_t INT2_CTRL_val = 0x01;
	writeRegister(_chipSelectPin, LSM6DS3_INT2_CTRL, INT2_CTRL_val);
	//
	// Enable X, Y, and Z gyro axes
	uint8_t CTRL10_val = (1 << 3) | (1 << 4) | (1 << 5);
	writeRegister (_chipSelectPin, LSM6DS3_CTRL10_C, CTRL10_val);
	//
	// Enable X, Y, and Z accel axes
	uint8_t CTRL9_val = (1 << 3) | (1 << 4) | (1 << 5);
	writeRegister (_chipSelectPin, LSM6DS3_CTRL9_XL, CTRL9_val);
	//
	// Continuous update, interrupt active high, push-pull, SPI 4-wire, Auto-increment, Little endian
	uint8_t CTRL3_val = (1 << 2);
	writeRegister (_chipSelectPin, LSM6DS3_CTRL3_C, CTRL3_val);
	//
	// Self-test disabled
	uint8_t CTRL5_val = 0x00;
	writeRegister (_chipSelectPin, LSM6DS3_CTRL5_C, CTRL5_val);
	//
	// Accelerometer in High-Perf mode
	uint8_t CTRL6_val = 0x00;
	writeRegister (_chipSelectPin, LSM6DS3_CTRL6_C, CTRL6_val);
	//
	// enable or disable high-pass filter on the accelerometer
	if (HP_enable_accel){
		uint8_t CTRL8_val = HP_freq_gyro | (1 << 2);
	}
	else{
		uint8_t CTRL8_val = 0x00;
		writeRegister(_chipSelectPin, LSM6DS3_CTRL8_XL, CTRL8_val);
	}
	//
	// enable or disable high-pass filter on the gyroscope
	uint8_t CTRL7_val = 0x00;
	if (HP_enable_gyro){
		CTRL7_val = (1 << 6) | HP_freq_gyro;
	}
	writeRegister(_chipSelectPin, LSM6DS3_CTRL7_G, CTRL7_val);
	//
	// selected accelerometer ODR, range and auto anti-aliasing
	switch (accel_range){
		case (LSM6DS3_RANGE_A_2):
			_sc_fact_a = INS_G_VAL * 0.00006103515625f;
			break;
		case (LSM6DS3_RANGE_A_4):
			_sc_fact_a = INS_G_VAL * 0.0001220703125f;
			break;
		case (LSM6DS3_RANGE_A_8):
			_sc_fact_a = INS_G_VAL * 0.000244140625f;
			break;
		case (LSM6DS3_RANGE_A_16):
			_sc_fact_a = INS_G_VAL * 0.00048828125f;
			break;
		default:
			return 2;
	}
	if (LPF_enable_accel){
		//
		// Manual filter config, I2C disabled
		uint8_t CTRL4_val = (1 << 7) | (1 << 2);
		writeRegister (_chipSelectPin, LSM6DS3_CTRL4_C, CTRL4_val);
	}
	else {
		//
		// Auto filter config, I2C disabled
		uint8_t CTRL4_val = (1 << 2);
		writeRegister (_chipSelectPin, LSM6DS3_CTRL4_C, CTRL4_val);
	}
	_CTRL1_val = accel_odr | accel_range | LPF_freq_accel;
	writeRegister (_chipSelectPin, LSM6DS3_CTRL1_XL, _CTRL1_val);
	//
	// selected gyro ODR and full scale
	// Selected gyro ODR and range
	switch (gyro_range){
		case (LSM6DS3_RANGE_G_125):
			_sc_fact_g = 4.375e-3f * INS_TORAD;
			break;
		case (LSM6DS3_RANGE_G_245):
			_sc_fact_g = 8.75e-3f * INS_TORAD;
			break;
		case (LSM6DS3_RANGE_G_500):
			_sc_fact_g = 17.5e-3f * INS_TORAD;
			break;
		case (LSM6DS3_RANGE_G_1000):
			_sc_fact_g = 35e-3f * INS_TORAD;
			break;
		case (LSM6DS3_RANGE_G_2000):
			_sc_fact_g = 70e-3f * INS_TORAD;
			break;
		default:
			return 2;
	}
	_CTRL2_val = gyro_odr | gyro_range;
	writeRegister(_chipSelectPin, LSM6DS3_CTRL2_G, _CTRL2_val);
	//
	// Discard the first n gyro measures
	if(! discard_measures_gyro(LSM6DS3_DISCARDED_MEASURES, LSM6DS3_DISCARD_TIMEOUT)){
		return 0;
	}
	//
	// Discard the first n accel measures
	if(! discard_measures_accel(LSM6DS3_DISCARDED_MEASURES, LSM6DS3_DISCARD_TIMEOUT)){
		delay(50);
		return 0;
	}
	return 1;
}

//-------------------------Turn on---------------------------//
void LSM6DS3::turn_on_gyro(){
	uint8_t CTRL4_val = readRegister(_chipSelectPin, LSM6DS3_CTRL4_C);
	writeRegister(_chipSelectPin, LSM6DS3_CTRL4_C, (CTRL4_val & 0xBF));
	writeRegister(_chipSelectPin, LSM6DS3_CTRL2_G, _CTRL2_val);
	delay(200);
}

//------------------------Turn off---------------------------//
void LSM6DS3::turn_off_gyro(){
	uint8_t CTRL2_val = readRegister(_chipSelectPin, LSM6DS3_CTRL2_G);
	writeRegister(_chipSelectPin, LSM6DS3_CTRL2_G, (CTRL2_val & 0x0F));
}

//-------------------------Sleep-----------------------------//
void LSM6DS3::sleep_gyro(){
	uint8_t CTRL4_val = readRegister(_chipSelectPin, LSM6DS3_CTRL4_C);
	writeRegister(_chipSelectPin, LSM6DS3_CTRL4_C, (CTRL4_val | (1 << 6)));
}

//------------------------Read data-------------------------//
uint8_t LSM6DS3::read_raw_gyro(){
	uint8_t buffer[6];
	readMultipleRegisters(_chipSelectPin, buffer, 6, LSM6DS3_OUTX_L_G);
	gx = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_g);
	gy = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_g);
	gz = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_g);
	return 1;
}

//------------------Read data when ready--------------------//
uint8_t LSM6DS3::read_gyro_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin_G)){
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
uint8_t LSM6DS3::read_gyro_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin, LSM6DS3_STATUS_REG);
		if (STATUS_val & (1 << 1)){
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
uint8_t LSM6DS3::check_gyro_biases(float bx, float by, float bz){
// Define Threshold based on Full-Scale value
	float thrs;
	if ((_sc_fact_g - 4.375e-3f * INS_TORAD) < 1e-5f){
		thrs = 10 * INS_TORAD * 1.2f; 	//typical 10dps offset
	}
	else if ((_sc_fact_g - 8.75e-3f * INS_TORAD) < 1e-5f){
		thrs = 10 * INS_TORAD * 1.2f;	//typical 10dps offset
	}
	else if ((_sc_fact_g - 17.5e-3f * INS_TORAD) < 1e-5f){
		thrs = 10 * INS_TORAD * 1.2f;	//typical 10dps offset
	}
	else if ((_sc_fact_g - 35e-3f * INS_TORAD) < 1e-5f){
		thrs = 10 * INS_TORAD * 1.2f;	//typical 10dps offset
	}
	else if ((_sc_fact_g - 70e-3f * INS_TORAD) < 1e-5f){
		thrs = 10 * INS_TORAD * 1.2f;	//typical 10dps offset
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
void LSM6DS3::HP_reset_gyro(){
	uint8_t CTRL7_val = readRegister(_chipSelectPin, LSM6DS3_CTRL7_G);
	writeRegister(_chipSelectPin, LSM6DS3_CTRL7_G, (CTRL7_val | 0x08));
}

//-----------------------Self-Test-------------------------//
uint8_t LSM6DS3::self_test_gyro(uint8_t mode){
	uint8_t status = 0;
	// Discard the first n measures
	if(!discard_measures_gyro(LSM6DS3_DISCARDED_MEASURES_ST, LSM6DS3_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (uint8_t ii = 0; ii < LSM6DS3_GYRO_SELF_TEST_MEASURES; ii++){
		read_gyro_STATUS(LSM6DS3_DISCARD_TIMEOUT);
		x_pre += gx;
		y_pre += gy;
		z_pre += gz;
	}
	x_pre /= LSM6DS3_GYRO_SELF_TEST_MEASURES;
	y_pre /= LSM6DS3_GYRO_SELF_TEST_MEASURES;
	z_pre /= LSM6DS3_GYRO_SELF_TEST_MEASURES;
	// Turn off the sensor and setup self-test
	turn_off_gyro();
	uint8_t CTRL5_val = readRegister(_chipSelectPin, LSM6DS3_CTRL5_C);
	if (!mode){
		CTRL5_val |= 0x04; // Self-test mode 0
	}
	else {
		CTRL5_val |= 0x0C; // Self-test mode 1
	}
	writeRegister(_chipSelectPin, LSM6DS3_CTRL5_C, CTRL5_val);
	turn_on_gyro();
	// Discard the first n measures 
	if(! discard_measures_gyro(LSM6DS3_DISCARDED_MEASURES_ST, LSM6DS3_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (uint8_t ii = 0; ii < LSM6DS3_GYRO_SELF_TEST_MEASURES; ii++){
		read_gyro_STATUS(LSM6DS3_DISCARD_TIMEOUT);
		x_post += gx;
		y_post += gy;
		z_post += gz;
	}
	x_post /= LSM6DS3_GYRO_SELF_TEST_MEASURES;
	y_post /= LSM6DS3_GYRO_SELF_TEST_MEASURES;
	z_post /= LSM6DS3_GYRO_SELF_TEST_MEASURES;
	// Define threshold based on the Full-Scale value
	float thrs;
	if ((_sc_fact_g - 8.75e-3f * INS_TORAD) < 1e-5f){
		thrs = 20 * INS_TORAD; 
	}
	else if ((_sc_fact_g - 17.5e-3f * INS_TORAD) < 1e-5f){
		thrs = 70 * INS_TORAD;
	}
	else if ((_sc_fact_g - 70e-3f * INS_TORAD) < 1e-5f){
		thrs = 150 * INS_TORAD;
	}
	else {
		return 0;
	}
	// Check if values are bigger than the threshold
	if (ch_st(x_pre, x_post, (0.6f * thrs), (1.4f * thrs)) && ch_st(y_pre, y_post, (0.6f * thrs), (1.4f * thrs)) && ch_st(z_pre, z_post, (0.6f * thrs), (1.4f * thrs))) {
		status = 1;
	}
	turn_off_gyro();
	writeRegister(_chipSelectPin, LSM6DS3_CTRL5_C, (CTRL5_val & 0xF0));
	turn_on_gyro();
	// Discard the first n measures
	if(! discard_measures_gyro(LSM6DS3_DISCARDED_MEASURES_ST, LSM6DS3_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Gyro Status------------------------//
uint8_t LSM6DS3::status_gyro(){
	return readRegister(_chipSelectPin, LSM6DS3_STATUS_REG);
}

//-------------------Discard measures----------------------//
uint8_t LSM6DS3::discard_measures_gyro(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	while (count < (number_of_measures)){
		if(! read_gyro_STATUS(timeout)){
			return 0;
		}
		count++;
	}
	return 1;
}

//=============================Public Members Accelerometer====================================//
//----------------Turn on accelerometer----------------//
void LSM6DS3::turn_on_accel(){
	writeRegister(_chipSelectPin, LSM6DS3_CTRL1_XL, _CTRL1_val);
	delay(200);
}

//----------------Turn off accelerometer---------------//
void LSM6DS3::turn_off_accel(){
	uint8_t CTRL1_val = readRegister(_chipSelectPin, LSM6DS3_CTRL1_XL);
	writeRegister(_chipSelectPin, LSM6DS3_CTRL1_XL, (CTRL1_val & 0x0F));
}

//-----------------Read accelerometer------------------//
uint8_t LSM6DS3::read_raw_accel(){
	uint8_t buffer[6];
	readMultipleRegisters(_chipSelectPin, buffer, 6, LSM6DS3_OUTX_L_XL);
	ax = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_a);
	ay = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_a);
	az = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_a);
	return 1;
}

//------------Read accelerometer when ready--------------//
uint8_t LSM6DS3::read_accel_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin_A)){
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
uint8_t LSM6DS3::read_accel_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin, LSM6DS3_STATUS_REG);
		if (STATUS_val & 0x01){
			read_raw_accel();
			return 1;
		}
		if ((micros() - now) < 0){
			now = 0L;
		}
	}
	delay(50);
return 0;
}

//--------------------Check biases------------------------//
uint8_t LSM6DS3::check_accel_biases(float bx, float by, float bz){
	float thrs = 40e-3f * INS_G_VAL * 1.1f; //typical 40mg zero-G level
	if ((fabs(bx) > thrs) || (fabs(by) > thrs) || (fabs(bz) > thrs)){
		return 0;
	}
	return 1;
}

//------------Self-test the accelerometer---------------//
uint8_t LSM6DS3::self_test_accel(uint8_t mode){
	uint8_t status = 0;
	// Discard the first n measures
	if(! discard_measures_accel(LSM6DS3_DISCARDED_MEASURES_ST, LSM6DS3_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (uint8_t ii = 0; ii < LSM6DS3_ACCEL_SELF_TEST_MEASURES; ii++){
		read_accel_STATUS(LSM6DS3_DISCARD_TIMEOUT);
		x_pre += ax;
		y_pre += ay;
		z_pre += az;
	}
	x_pre /= LSM6DS3_ACCEL_SELF_TEST_MEASURES;
	y_pre /= LSM6DS3_ACCEL_SELF_TEST_MEASURES;
	z_pre /= LSM6DS3_ACCEL_SELF_TEST_MEASURES;
	// Turn off the sensor and setup self-test
	turn_off_accel();
	uint8_t CTRL5_val = readRegister(_chipSelectPin, LSM6DS3_CTRL5_C);
	if (!mode){
		CTRL5_val |= 0x01; // Self-test mode 0
	}
	else {
		CTRL5_val |= 0x02; // Self-test mode 1
	}
	writeRegister(_chipSelectPin, LSM6DS3_CTRL5_C, CTRL5_val);
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(LSM6DS3_DISCARDED_MEASURES_ST,LSM6DS3_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (uint8_t ii = 0; ii < LSM6DS3_ACCEL_SELF_TEST_MEASURES; ii++){
		read_accel_STATUS(LSM6DS3_DISCARD_TIMEOUT);
		x_post += ax;
		y_post += ay;
		z_post += az;
	}
	x_post /= LSM6DS3_ACCEL_SELF_TEST_MEASURES;
	y_post /= LSM6DS3_ACCEL_SELF_TEST_MEASURES;
	z_post /= LSM6DS3_ACCEL_SELF_TEST_MEASURES;
	// Define Threshold based on the Full-Scale value
	float thrs_min = 60e-3f * INS_G_VAL;
	float thrs_max = 1700e-3f * INS_G_VAL;
	// Check if values are bigger than the threshold
	if (ch_st(x_pre, x_post, thrs_min, thrs_max) && ch_st(y_pre, y_post, thrs_min, thrs_max) && ch_st(z_pre, z_post, thrs_min, thrs_max)) {
		status = 1;
	}
	turn_off_accel();
	writeRegister(_chipSelectPin, LSM6DS3_CTRL5_C, (CTRL5_val & 0xF0));
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(LSM6DS3_DISCARDED_MEASURES_ST, LSM6DS3_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Accel Status------------------------//
uint8_t LSM6DS3::status_accel(){
	return readRegister(_chipSelectPin, LSM6DS3_STATUS_REG);
}

//-------------------Discard measures----------------------//
uint8_t LSM6DS3::discard_measures_accel(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	while (count < (number_of_measures)){
		if(! read_accel_STATUS(timeout)){
			return 0;
		}
		count++;
	}
	return 1;
}

//=============================Public Members Temperature====================================//
//------------------------Read data----------------------//
uint8_t LSM6DS3::read_raw_thermo(){
	uint8_t buffer[2];
	readMultipleRegisters(_chipSelectPin, buffer, 2, LSM6DS3_OUT_TEMP_L);
	temperature = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * 0.0625f) + 25.0f;
	return 1;
}

//------------------Read data when ready-----------------//
uint8_t LSM6DS3::read_thermo_DRDY(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		if (digitalRead(_DRDY_pin_A)){
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
uint8_t LSM6DS3::read_thermo_STATUS(uint32_t timeout){
	uint32_t now = micros();
	while((micros() - now) < timeout){
		uint8_t STATUS_val = readRegister(_chipSelectPin, LSM6DS3_STATUS_REG);
		if (STATUS_val & (1 << 2)){
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
uint8_t LSM6DS3::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
	uint8_t count = 0;
	while (count < (number_of_measures)){
		if(! read_thermo_STATUS(timeout)){
			return 0;
		}
		count++;
	}
	return 1;
}
