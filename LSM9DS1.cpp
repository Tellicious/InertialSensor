//
// LSM9DS1.cpp
//
//
// Created by Andrea Vivani on 11/7/15.
// Copyright (c) 2015 Andrea Vivani. All rights reserved.
//
#include "LSM9DS1.h"
#include "INS_AuxFun.h"
#ifdef INS_ARDUINO
#include "Arduino.h"
#include <SPI.h>
#endif
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
#ifdef INS_ARDUINO
  #define LSM9DS1_READ_REGISTER_XG(reg) INS_SPI_readRegister(_chipSelectPin_XG, reg, LSM9DS1_READ)
  #define LSM9DS1_READ_MULTIPLE_REGISTERS_XG(buf, num, start) INS_SPI_readMultipleRegisters(_chipSelectPin_XG, buf, num, startRegister, LSM9DS1_READ)
  #define LSM9DS1_WRITE_REGISTER_XG(reg, val) INS_SPI_writeRegister(_chipSelectPin_XG, reg, val, 0x00)
  #define LSM9DS1_READ_REGISTER_M(reg) INS_SPI_readRegister(_chipSelectPin_M, reg, LSM9DS1_READ)
  #define LSM9DS1_READ_MULTIPLE_REGISTERS_M(buf, num, start) INS_SPI_readMultipleRegisters(_chipSelectPin_M, buf, num, startRegister, (LSM9DS1_READ | LSM9DS1_MULT))
  #define LSM9DS1_WRITE_REGISTER_M(reg, val) INS_SPI_writeRegister(_chipSelectPin_M, reg, val, 0x00)
#elif defined(INS_CHIBIOS)
  #define LSM9DS1_READ_REGISTER_XG(reg) INS_SPI_readRegister(_SPI_int, _spicfg_XG, reg, LSM9DS1_READ)
  #define LSM9DS1_READ_MULTIPLE_REGISTERS_XG(buf, num, start) INS_SPI_readMultipleRegisters(_SPI_int, _spicfg_XG, buf, num, start, LSM9DS1_READ)
  #define LSM9DS1_WRITE_REGISTER_XG(reg, val) INS_SPI_writeRegister(_SPI_int, _spicfg_XG, reg, val, 0x00)
    #define LSM9DS1_READ_REGISTER_M(reg) INS_SPI_readRegister(_SPI_int, _spicfg_M, reg, LSM9DS1_READ)
  #define LSM9DS1_READ_MULTIPLE_REGISTERS_M(buf, num, start) INS_SPI_readMultipleRegisters(_SPI_int, _spicfg_M, buf, num, start, (LSM9DS1_READ | LSM9DS1_MULT))
  #define LSM9DS1_WRITE_REGISTER_M(reg, val) INS_SPI_writeRegister(_SPI_int, _spicfg_M, reg, val, 0x00)
#endif
//=====================================Constructors==========================================//
#ifdef INS_ARDUINO
LSM9DS1::LSM9DS1 (uint8_t CS_pin_XG, uint8_t CS_pin_M):InertialSensor(), AccelerometerSensor(), GyroscopeSensor(), MagnetometerSensor(), ThermometerSensor(){
	_chipSelectPin_XG = CS_pin_XG;
	_chipSelectPin_M = CS_pin_M;
	_DRDY_pin_G = 0;
	_DRDY_pin_A = 0;
	_DRDY_pin_M = 0;
}

LSM9DS1::LSM9DS1 (uint8_t CS_pin_XG, uint8_t CS_pin_M, uint8_t DRDY_pin_G, uint8_t DRDY_pin_A, uint8_t DRDY_pin_M):InertialSensor(), AccelerometerSensor(), GyroscopeSensor(), MagnetometerSensor(), ThermometerSensor(){
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
#elif defined(INS_CHIBIOS)
LSM9DS1::LSM9DS1 (SPIDriver* SPI, SPIConfig* spicfg_XG, SPIConfig* spicfg_M):InertialSensor(), AccelerometerSensor(), GyroscopeSensor(), MagnetometerSensor(), ThermometerSensor(){
	_SPI_int = SPI;
	_spicfg_XG = spicfg_XG;
	_spicfg_M = spicfg_M;
	_DRDY_pin_G = 0;
	_DRDY_pin_A = 0;
	_DRDY_pin_M = 0;
	init();
}

LSM9DS1::LSM9DS1 (SPIDriver* SPI, SPIConfig* spicfg_XG, SPIConfig* spicfg_M, ioportid_t gpio_DRDY_G, uint8_t DRDY_pin_G, ioportid_t gpio_DRDY_A, uint8_t DRDY_pin_A, ioportid_t gpio_DRDY_M, uint8_t DRDY_pin_M):InertialSensor(), AccelerometerSensor(), GyroscopeSensor(), MagnetometerSensor(), ThermometerSensor(){
	_SPI_int = SPI;
	_spicfg_XG = spicfg_XG;
	_spicfg_M = spicfg_M;
	_gpio_DRDY_G = gpio_DRDY_G;
	_DRDY_pin_G = DRDY_pin_G;
	_gpio_DRDY_A = gpio_DRDY_A;
	_DRDY_pin_A = DRDY_pin_A;
	_gpio_DRDY_M = gpio_DRDY_M;
	_DRDY_pin_M = DRDY_pin_M;
	init();
}

//-----------------------Initialization-----------------------//
void LSM9DS1::init(){
	palSetPad(_spicfg_XG->ssport, _spicfg_XG->sspad);
	palSetPadMode(_spicfg_XG->ssport, _spicfg_XG->sspad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(_spicfg_M->ssport, _spicfg_M->sspad);
	palSetPadMode(_spicfg_M->ssport, _spicfg_M->sspad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	if ((_DRDY_pin_G != 0) || (_DRDY_pin_A != 0) || (_DRDY_pin_M != 0)){
		palSetPadMode(_gpio_DRDY_G, _DRDY_pin_G, PAL_MODE_INPUT);
		palSetPadMode(_gpio_DRDY_A, _DRDY_pin_A, PAL_MODE_INPUT);
		palSetPadMode(_gpio_DRDY_M, _DRDY_pin_M, PAL_MODE_INPUT);
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
#endif

//=================================Public Members Gyro=======================================//
//-----------------------Configuration-----------------------//
uint8_t LSM9DS1::config_accel_gyro(uint8_t gyro_range, uint8_t gyro_odr, uint8_t LPF2_enable_gyro, uint8_t HP_enable_gyro, uint8_t HP_freq_gyro, uint8_t accel_range, uint8_t accel_odr, uint8_t LPF_enable_accel, uint8_t LPF_freq_accel, uint8_t HP_enable_accel, uint8_t HP_freq_accel){
	init();
	//
	// Trash the first reading
	LSM9DS1_READ_REGISTER_XG(LSM9DS1_WHO_AM_I);
	//
	// Check if the device ID is correct
	if (LSM9DS1_READ_REGISTER_XG(LSM9DS1_WHO_AM_I) != LSM9DS1_ID_XG){
		return 0;
	}
	//
	// Gyroscope DRDY on INT1
	uint8_t INT1_CTRL_val = (1 << 1);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_INT1_CTRL, INT1_CTRL_val);
	//
	// Accelerometer DRDY on INT2
	uint8_t INT2_CTRL_val = 0x01;
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_INT2_CTRL, INT2_CTRL_val);
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
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG2_G, CTRL2_val);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG3_G, CTRL3_val);
	//
	// Enable X, Y, and Z gyro axes
	uint8_t CTRL4_val = (1 << 3) | (1 << 4) | (1 << 5);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG4, CTRL4_val);
	//
	// No decimation, enable X, Y, Z accel axes
	uint8_t CTRL5_val = (1 << 3) | (1 << 4) | (1 << 5);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG5_XL, CTRL5_val);
	//
	// Continuous update, interrupt active high, push-pull, SPI 4-wire, Auto-increment, Little endian
	uint8_t CTRL8_val = (1 << 2);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG8, CTRL8_val);
	//
	// disabled I2C
	uint8_t CTRL9_val = (1 << 2);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG9, CTRL9_val);
	//
	// CTRL_REG10 is kept as default to 0x00 (no self-test)
	//
	// Selected gyro ODR and range
	switch (gyro_range){
		case (LSM9DS1_RANGE_G_245):
			_sc_fact_g = 8.75e-3f * INS_TORAD;
			break;
		case (LSM9DS1_RANGE_G_500):
			_sc_fact_g = 17.5e-3f * INS_TORAD;
			break;
		case (LSM9DS1_RANGE_G_2000):
			_sc_fact_g = 70e-3f * INS_TORAD;
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
		CTRL7_val_XL = 0x00;
	}
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG7_XL, CTRL7_val_XL);
	// selected accelerometer ODR, range and auto anti-aliasing
	switch (accel_range){
		case (LSM9DS1_RANGE_A_2):
			_sc_fact_a = INS_G_VAL * 0.00006103515625f;
			break;
		case (LSM9DS1_RANGE_A_4):
			_sc_fact_a = INS_G_VAL * 0.0001220703125f;
			break;
		case (LSM9DS1_RANGE_A_8):
			_sc_fact_a = INS_G_VAL * 0.000244140625f;
			break;
		case (LSM9DS1_RANGE_A_16):
			_sc_fact_a = INS_G_VAL * 0.000732421875f;
			break;
		default:
			return 2;
	}
	_CTRL1_val_XG = gyro_range | gyro_odr;
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG1_G, _CTRL1_val_XG);
	//
	// gyro and accel operate at the same ODR if CTRL1 is used and CTRL6 is in Power-down
	_CTRL6_val_XL = accel_range | accel_odr;
	if (LPF_enable_accel){
		_CTRL6_val_XL |= (1 << 2) | LPF_freq_accel;
	}
	else {
		_CTRL6_val_XL &= 0xF8; 
	}
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG6_XL, _CTRL6_val_XL);
#ifdef INS_ARDUINO
	delay(200);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(200);
#endif
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
	uint8_t CTRL_REG9_val = LSM9DS1_READ_REGISTER_XG(LSM9DS1_CTRL_REG9);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG9, (CTRL_REG9_val & 0xBF));
	uint8_t CTRL9_val = LSM9DS1_READ_REGISTER_XG(LSM9DS1_CTRL_REG9);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG9, (CTRL9_val & 0x1F));
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG1_G, _CTRL1_val_XG);
#ifdef INS_ARDUINO
	delay(200);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(200);
#endif
}

//------------------------Turn off---------------------------//
void LSM9DS1::turn_off_gyro(){
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG1_G,0x00);
}

//-------------------------Sleep-----------------------------//
void LSM9DS1::sleep_gyro(){
	uint8_t CTRL_REG9_val = LSM9DS1_READ_REGISTER_XG(LSM9DS1_CTRL_REG9);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG9, (CTRL_REG9_val | (1 << 6)));
}

//------------------------Read data-------------------------//
uint8_t LSM9DS1::read_raw_gyro(){
	uint8_t buffer[6];
	LSM9DS1_READ_MULTIPLE_REGISTERS_XG(buffer, 6, LSM9DS1_OUT_X_L_G);
	gx = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_g);
	gy = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_g);
	gz = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_g);
	return 1;
}

//------------------Read data when ready--------------------//
uint8_t LSM9DS1::read_gyro_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
	INS_read_DRDY(timeout, read_raw_gyro, _DRDY_pin_G);
#elif defined(INS_CHIBIOS)
	INS_read_DRDY(timeout, read_raw_gyro, _gpio_DRDY_G, _DRDY_pin_G);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS1::read_gyro_STATUS(uint32_t timeout){
	INS_read_STATUS(timeout, read_raw_gyro, status_gyro, (1 << 1));
}

//--------------------Check biases------------------------//
uint8_t LSM9DS1::check_gyro_biases(float bx, float by, float bz){
// Define Threshold based on Full-Scale value
	float thrs;
	if ((_sc_fact_g - 8.75e-3f * INS_TORAD) < 1e-5f){
		thrs = 10 * INS_TORAD * 1.2f; //typical 10dps offset
	}
	else if ((_sc_fact_g - 17.5e-3f * INS_TORAD) < 1e-5f){
		thrs = 15 * INS_TORAD * 1.2f;	//typical 15dps offset
	}
	else if ((_sc_fact_g - 70e-3f * INS_TORAD) < 1e-5f){
		thrs = 30 * INS_TORAD * 1.2f;	//typical 25dps offset
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
	LSM9DS1_READ_REGISTER_XG(LSM9DS1_REFERENCE_G);
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
	for (uint8_t ii = 0; ii < LSM9DS1_GYRO_SELF_TEST_MEASURES; ii++){
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
	uint8_t CTRL10_val = LSM9DS1_READ_REGISTER_XG(LSM9DS1_CTRL_REG10);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG10, (CTRL10_val | (1 << 2)));
	turn_on_gyro();
	// Discard the first n measures 
	if(! discard_measures_gyro(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (uint8_t ii = 0; ii < LSM9DS1_GYRO_SELF_TEST_MEASURES; ii++){
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
	if ((_sc_fact_g - 8.75e-3f * INS_TORAD) < 1e-5f){
		thrs = 95 * INS_TORAD; 
	}
	else if ((_sc_fact_g - 17.5e-3f * INS_TORAD) < 1e-5f){
		thrs = 190 * INS_TORAD;
	}
	else if ((_sc_fact_g - 70e-3f * INS_TORAD) < 1e-5f){
		thrs = 380 * INS_TORAD;
	}
	else {
		return 0;
	}
	// Check if values are bigger than the threshold
	if (INS_ch_st(x_pre, x_post, (0.6f * thrs), (1.4f * thrs)) && INS_ch_st(y_pre, y_post, (0.6f * thrs), (1.4f * thrs)) && INS_ch_st(z_pre, z_post, (0.6f * thrs), (1.4f * thrs))) {
		status = 1;
	}
	turn_off_gyro();
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG10, CTRL10_val);
	turn_on_gyro();
	// Discard the first n measures
	if(! discard_measures_gyro(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Gyro Status------------------------//
uint8_t LSM9DS1::status_gyro(){
	return LSM9DS1_READ_REGISTER_XG(LSM9DS1_STATUS_REG);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS1::discard_measures_gyro(uint8_t number_of_measures, uint32_t timeout){
	INS_discard_measures(number_of_measures, timeout, read_gyro_STATUS);
}

//=============================Public Members Accelerometer====================================//
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
	LSM9DS1_READ_MULTIPLE_REGISTERS_XG(buffer, 6, LSM9DS1_OUT_X_L_XL);
	ax = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_a);
	ay = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_a);
	az = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_a);
	return 1;
}

//------------Read accelerometer when ready--------------//
uint8_t LSM9DS1::read_accel_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
	INS_read_DRDY(timeout, read_raw_accel, _DRDY_pin_A);
#elif defined(INS_CHIBIOS)
	INS_read_DRDY(timeout, read_raw_accel, _gpio_DRDY_A, _DRDY_pin_A);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS1::read_accel_STATUS(uint32_t timeout){
	INS_read_STATUS(timeout, read_raw_accel, status_accel, 0x01);
}

//--------------------Check biases------------------------//
uint8_t LSM9DS1::check_accel_biases(float bx, float by, float bz){
	float thrs = 90e-3f * INS_G_VAL * 1.1f; //typical 90mg zero-G level
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
	for (uint8_t ii = 0; ii < LSM9DS1_ACCEL_SELF_TEST_MEASURES; ii++){
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
	uint8_t CTRL10_val = LSM9DS1_READ_REGISTER_XG(LSM9DS1_CTRL_REG10);
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG10, (CTRL10_val | 0x01));
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(LSM9DS1_DISCARDED_MEASURES_ST,LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (uint8_t ii = 0; ii < LSM9DS1_ACCEL_SELF_TEST_MEASURES; ii++){
		read_accel_STATUS(LSM9DS1_DISCARD_TIMEOUT);
		x_post += ax;
		y_post += ay;
		z_post += az;
	}
	x_post /= LSM9DS1_ACCEL_SELF_TEST_MEASURES;
	y_post /= LSM9DS1_ACCEL_SELF_TEST_MEASURES;
	z_post /= LSM9DS1_ACCEL_SELF_TEST_MEASURES;
	// Define Threshold based on the Full-Scale value
	float thrs_min = 60e-3f * INS_G_VAL;
	float thrs_max = 1700e-3f * INS_G_VAL;
	// Check if values are bigger than the threshold
	if (INS_ch_st(x_pre, x_post, thrs_min, thrs_max) && INS_ch_st(y_pre, y_post, thrs_min, thrs_max) && INS_ch_st(z_pre, z_post, thrs_min, thrs_max)) {
		status = 1;
	}
	turn_off_accel();
	LSM9DS1_WRITE_REGISTER_XG(LSM9DS1_CTRL_REG10, CTRL10_val);
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Accel Status------------------------//
uint8_t LSM9DS1::status_accel(){
	return LSM9DS1_READ_REGISTER_XG(LSM9DS1_STATUS_REG);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS1::discard_measures_accel(uint8_t number_of_measures, uint32_t timeout){
	INS_discard_measures(number_of_measures, timeout, read_accel_STATUS);
}

//=============================Public Members Magnetometer===================================//
//--------------------Configuration--------------------//
uint8_t LSM9DS1::config_mag(uint8_t range_conf, uint8_t odr_conf){
	init();
	// Trash the first reading
	LSM9DS1_READ_REGISTER_M(LSM9DS1_WHO_AM_I_M);
	// Check if the device ID is correct
	if (LSM9DS1_READ_REGISTER_M(LSM9DS1_WHO_AM_I_M) != LSM9DS1_ID_M){
		return 0;
	}
	//
	//selected range
	uint8_t CTRL2_val = range_conf & 0x60;
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG2_M, CTRL2_val);
	switch (range_conf){
		case (LSM9DS1_RANGE_M_4):
			_sc_fact_m = 1.0f / 6842.0f;
			break;
		case (LSM9DS1_RANGE_M_8):
			_sc_fact_m = 1.0f / 3421.0f;
			break;
		case (LSM9DS1_RANGE_M_12):
			_sc_fact_m = 1.0f / 2281.0f;
			break;
		case (LSM9DS1_RANGE_M_16):
			_sc_fact_m = 1.0f / 1711.0f;
			break;
		default:
		return 2;
	}
	//
	//continuous update
	uint8_t CTRL5_val = 0x00;
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG5_M, CTRL5_val);
	//
	//Z-axis on ultra-high performance mode, little endian
	uint8_t CTRL4_val = (1 << 3) | (1 << 2);
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG4_M, CTRL4_val);
	//
	//temperature enable, ultra-high perfomance mode, selected ODR, no self test
	uint8_t CTRL1_val = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 1) | odr_conf;
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG1_M, CTRL1_val);
	//
	//power on, SPI 4 wire, Continuous conversion mode
	_CTRL3_val_M = (1 << 7);
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG3_M, _CTRL3_val_M);
#ifdef INS_ARDUINO
	delay(200);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(200);
#endif
	// Discard the first n measures
	if(! discard_measures_mag(LSM9DS1_DISCARDED_MEASURES, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-----------------Turn on magnetometer----------------//
void LSM9DS1::turn_on_mag(){
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG3_M, _CTRL3_val_M);
#ifdef INS_ARDUINO
	delay(200);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(200);
#endif
}

//-----------------Turn off magnetometer---------------//
void LSM9DS1::turn_off_mag(){
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG3_M, (_CTRL3_val_M | 0x3));
}

//------------------Read magnetometer------------------//
uint8_t LSM9DS1::read_raw_mag(){
	uint8_t buffer[6];
	LSM9DS1_READ_MULTIPLE_REGISTERS_M(buffer, 6, LSM9DS1_OUT_X_L_M);
	mx = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_m);
	my = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_m);
	mz = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_m);
	return 1;
}

//------------Read magnetometer when ready-------------//
uint8_t LSM9DS1::read_mag_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
	INS_read_DRDY(timeout, read_raw_mag, _DRDY_pin_M);
#elif defined(INS_CHIBIOS)
	INS_read_DRDY(timeout, read_raw_mag, _gpio_DRDY_M, _DRDY_pin_M);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS1::read_mag_STATUS(uint32_t timeout){
	INS_read_STATUS(timeout, read_raw_mag, status_mag, (1 << 3));
}

uint8_t LSM9DS1::self_test_mag(){
	uint8_t status = 0;
	// Use FS = 12 Gauss
	uint8_t CTRL2_val_old = LSM9DS1_READ_REGISTER_M(LSM9DS1_CTRL_REG2_M);
	uint8_t CTRL2_val = LSM9DS1_RANGE_M_12 & 0x60;
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG2_M, CTRL2_val);
	// Discard the first n measures
	if(!discard_measures_mag(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_pre = 0;
	float y_pre = 0;
	float z_pre = 0;
	for (uint8_t ii = 0; ii < LSM9DS1_MAG_SELF_TEST_MEASURES; ii++){
		read_mag_STATUS(LSM9DS1_DISCARD_TIMEOUT);
		x_pre += mx;
		y_pre += my;
		z_pre += mz;
	}
	x_pre /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0f); // average and revert to FS = 12 Gauss
	y_pre /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0f); // average and revert to FS = 12 Gauss
	z_pre /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0f); // average and revert to FS = 12 Gauss
	// Turn on self-test
	turn_off_mag();
	// Enable the self-test
	uint8_t CTRL1_val = LSM9DS1_READ_REGISTER_M(LSM9DS1_CTRL_REG1_M);
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG1_M, (CTRL1_val | 0x01));
	turn_on_mag();
	// Discard the first n measures 
	if(! discard_measures_mag(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (uint8_t ii = 0; ii < LSM9DS1_MAG_SELF_TEST_MEASURES; ii++){
		read_mag_STATUS(LSM9DS1_DISCARD_TIMEOUT);
		x_post += mx;
		y_post += my;
		z_post += mz;
	}
	x_post /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0f); // average and revert to FS = 12 Gauss
	y_post /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0f); // average and revert to FS = 12 Gauss
	z_post /= (LSM9DS1_MAG_SELF_TEST_MEASURES * _sc_fact_m * 2281.0f); // average and revert to FS = 12 Gauss
	float thrs_xy_min = 1.0f;
	float thrs_xy_max = 3.0f;
	float thrs_z_min = 0.1f;
	float thrs_z_max = 1.0f;
	if (INS_ch_st(x_pre, x_post, thrs_xy_min, thrs_xy_max) && INS_ch_st(y_pre, y_post, thrs_xy_min, thrs_xy_max) && INS_ch_st(z_pre, z_post, thrs_z_min, thrs_z_max)) {
		status = 1;
	}
	turn_off_mag();
	//remove self test
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG1_M, CTRL1_val);
	// Reset correct FS value
	LSM9DS1_WRITE_REGISTER_M(LSM9DS1_CTRL_REG2_M, CTRL2_val_old);
	turn_on_mag();
	// Discard the first n measures
	if(! discard_measures_mag(LSM9DS1_DISCARDED_MEASURES_ST, LSM9DS1_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//-----------------------Mag Status------------------------//
uint8_t LSM9DS1::status_mag(){
	return LSM9DS1_READ_REGISTER_M(LSM9DS1_STATUS_REG_M);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS1::discard_measures_mag(uint8_t number_of_measures, uint32_t timeout){
	INS_discard_measures_over(number_of_measures, timeout, read_raw_mag, status_mag, (1 << 7));
}

//=============================Public Members Temperature====================================//
//------------------------Read data----------------------//
uint8_t LSM9DS1::read_raw_thermo(){
	uint8_t buffer[2];
	LSM9DS1_READ_MULTIPLE_REGISTERS_XG(buffer, 2, LSM9DS1_OUT_TEMP_L);
	int16_t temperature_tmp = (((int16_t) buffer[1] << 12) | buffer[0] << 4) >> 4;
	temperature = (float) 20 + temperature_tmp * 0.125f; // Guessing that the intercept is at about 20°C 
	return 1;
}

//------------------Read data when ready-----------------//
uint8_t LSM9DS1::read_thermo_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
	INS_read_DRDY(timeout, read_raw_thermo, _DRDY_pin_A);
#elif defined(INS_CHIBIOS)
	INS_read_DRDY(timeout, read_raw_thermo, _gpio_DRDY_A, _DRDY_pin_A);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS1::read_thermo_STATUS(uint32_t timeout){
	INS_read_STATUS(timeout, read_raw_thermo, status_accel, (1 << 2));
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS1::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
	INS_discard_measures(number_of_measures, timeout, read_thermo_STATUS);
}
