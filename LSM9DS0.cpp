//
// LSM9DS0.cpp
//
//
// Created by Andrea Vivani on 28/3/15.
// Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "LSM9DS0.h"
#include "INS_AuxFun.h"
#ifdef INS_ARDUINO
#include "Arduino.h"
#include <SPI.h>
#endif
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
#ifdef INS_ARDUINO
  #define LSM9DS0_READ_REGISTER_G(reg) INS_SPI_readRegister(_chipSelectPin_G, reg, LSM9DS0_READ)
  #define LSM9DS0_READ_MULTIPLE_REGISTERS_G(buf, num, start) INS_SPI_readMultipleRegisters(_chipSelectPin_G, buf, num, startRegister, (LSM9DS0_READ | LSM9DS0_MULT))
  #define LSM9DS0_WRITE_REGISTER_G(reg, val) INS_SPI_writeRegister(_chipSelectPin_G, reg, val, 0x00)
  #define LSM9DS0_READ_REGISTER_XM(reg) INS_SPI_readRegister(_chipSelectPin_XM, reg, LSM9DS0_READ)
  #define LSM9DS0_READ_MULTIPLE_REGISTERS_XM(buf, num, start) INS_SPI_readMultipleRegisters(_chipSelectPin_XM, buf, num, startRegister, (LSM9DS0_READ | LSM9DS0_MULT))
  #define LSM9DS0_WRITE_REGISTER_XM(reg, val) INS_SPI_writeRegister(_chipSelectPin_XM, reg, val, 0x00)
#elif defined(INS_CHIBIOS)
  #define LSM9DS0_READ_REGISTER_G(reg) INS_SPI_readRegister(_SPI_int, _spicfg_G, reg, LSM9DS0_READ)
  #define LSM9DS0_READ_MULTIPLE_REGISTERS_G(buf, num, start) INS_SPI_readMultipleRegisters(_SPI_int, _spicfg_G, buf, num, start, (LSM9DS0_READ | LSM9DS0_MULT))
  #define LSM9DS0_WRITE_REGISTER_G(reg, val) INS_SPI_writeRegister(_SPI_int, _spicfg_G, reg, val, 0x00)
  #define LSM9DS0_READ_REGISTER_XM(reg) INS_SPI_readRegister(_SPI_int, _spicfg_XM, reg, LSM9DS0_READ)
  #define LSM9DS0_READ_MULTIPLE_REGISTERS_XM(buf, num, start) INS_SPI_readMultipleRegisters(_SPI_int, _spicfg_XM, buf, num, start, (LSM9DS0_READ | LSM9DS0_MULT))
  #define LSM9DS0_WRITE_REGISTER_XM(reg, val) INS_SPI_writeRegister(_SPI_int, _spicfg_XM, reg, val, 0x00)
#endif
//=====================================Constructors==========================================//
#ifdef INS_ARDUINO
LSM9DS0::LSM9DS0 (uint8_t CS_pin_G, uint8_t CS_pin_XM):InertialSensor(), AccelerometerSensor(), GyroscopeSensor(), MagnetometerSensor(), ThermometerSensor(){
	_chipSelectPin_G = CS_pin_G;
	_chipSelectPin_XM = CS_pin_XM;
	_DRDY_pin_G = 0;
	_DRDY_pin_A = 0;
	_DRDY_pin_M = 0;
}

LSM9DS0::LSM9DS0 (uint8_t CS_pin_G, uint8_t CS_pin_XM, uint8_t DRDY_pin_G, uint8_t DRDY_pin_A, uint8_t DRDY_pin_M):InertialSensor(), AccelerometerSensor(), GyroscopeSensor(), MagnetometerSensor(), ThermometerSensor(){
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
#elif defined(INS_CHIBIOS)
LSM9DS0::LSM9DS0 (SPIDriver* SPI, SPIConfig* spicfg_G, SPIConfig* spicfg_XM):InertialSensor(), AccelerometerSensor(), GyroscopeSensor(), MagnetometerSensor(), ThermometerSensor(){
	_SPI_int = SPI;
	_spicfg_G = spicfg_G;
	_spicfg_XM = spicfg_XM;
	_DRDY_pin_G = 0;
	_DRDY_pin_A = 0;
	_DRDY_pin_M = 0;
	init();
}

LSM9DS0::LSM9DS0 (SPIDriver* SPI, SPIConfig* spicfg_G, SPIConfig* spicfg_XM, ioportid_t gpio_DRDY_G, uint8_t DRDY_pin_G, ioportid_t gpio_DRDY_A, uint8_t DRDY_pin_A, ioportid_t gpio_DRDY_M, uint8_t DRDY_pin_M):InertialSensor(), AccelerometerSensor(), GyroscopeSensor(), MagnetometerSensor(), ThermometerSensor(){
	_SPI_int = SPI;
	_spicfg_G = spicfg_G;
	_spicfg_XM = spicfg_XM;
	_gpio_DRDY_G = gpio_DRDY_G;
	_DRDY_pin_G = DRDY_pin_G;
	_gpio_DRDY_A = gpio_DRDY_A;
	_DRDY_pin_A = DRDY_pin_A;
	_gpio_DRDY_M = gpio_DRDY_M;
	_DRDY_pin_M = DRDY_pin_M;
	init();
}

//-----------------------Initialization-----------------------//
void LSM9DS0::init(){
	palSetPad(_spicfg_G->ssport, _spicfg_G->sspad);
	palSetPadMode(_spicfg_G->ssport, _spicfg_G->sspad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(_spicfg_XM->ssport, _spicfg_XM->sspad);
	palSetPadMode(_spicfg_XM->ssport, _spicfg_XM->sspad, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
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
uint8_t LSM9DS0::config_gyro(uint8_t gyro_range, uint8_t gyro_odr, uint8_t LPF2_enable, uint8_t HP_enable, uint8_t HP_freq){
	init();
	// Trash the first reading
	LSM9DS0_READ_REGISTER_G(LSM9DS0_WHO_AM_I_G);
	// Check if the device ID is correct
	if (LSM9DS0_READ_REGISTER_G(LSM9DS0_WHO_AM_I_G) != LSM9DS0_ID_G){
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
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_CTRL_REG2_G, CTRL2_val);
	//
	//Data ready on DRDY_G
	uint8_t CTRL3_val = (1 << 3);
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_CTRL_REG3_G, CTRL3_val);
	//
	//Continuous update, Little endian, selected range, normal self-test, SPI 4-wire
	uint8_t CTRL4_val = (0 << 7) | gyro_range; 
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_CTRL_REG4_G, CTRL4_val);
	//
	//FIFO control on Bypass Mode
	uint8_t FIFO_CTRL_val = 0x00;
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_FIFO_CTRL_REG_G, FIFO_CTRL_val);
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
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_CTRL_REG5_G, CTRL5_val);
	//
	switch (gyro_range){
		case (LSM9DS0_RANGE_G_245):
			_sc_fact_g = 8.75e-3f * INS_TORAD;
			break;
		case (LSM9DS0_RANGE_G_500):
			_sc_fact_g = 17.5e-3f * INS_TORAD;
			break;
		case (LSM9DS0_RANGE_G_2000):
			_sc_fact_g = 70e-3f * INS_TORAD;
			break;
		default:
			return 2;
	}
	//
	//Clear the Reference register
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_REFERENCE_G, 0x00);
	//Selected ODR, power on, 3-axis enabled
	_CTRL1_val_G = (gyro_odr << 4) | (1 << 3) | 0x7;
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_CTRL_REG1_G, _CTRL1_val_G);
#ifdef INS_ARDUINO
	delay(200);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(200);
#endif
	// Discard the first n measures
	if(! discard_measures_gyro(LSM9DS0_DISCARDED_MEASURES, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	return 1;
}

//-------------------------Turn on---------------------------//
void LSM9DS0::turn_on_gyro(){
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_CTRL_REG1_G,_CTRL1_val_G);
#ifdef INS_ARDUINO
	delay(200);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(200);
#endif
}

//------------------------Turn off---------------------------//
void LSM9DS0::turn_off_gyro(){
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_CTRL_REG1_G,(_CTRL1_val_G & 0xF7));
}

//-------------------------Sleep-----------------------------//
void LSM9DS0::sleep_gyro(){
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_CTRL_REG1_G,(_CTRL1_val_G & 0xF8));
}

//------------------------Read data-------------------------//
uint8_t LSM9DS0::read_raw_gyro(){
	uint8_t buffer[6];
	LSM9DS0_READ_MULTIPLE_REGISTERS_G(buffer, 6, LSM9DS0_OUT_X_L_G);
	gx = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_g);
	gy = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_g);
	gz = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_g);
	return 1;
}

//------------------Read data when ready--------------------//
uint8_t LSM9DS0::read_gyro_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
	INS_read_DRDY(timeout, read_raw_gyro, _DRDY_pin_G);
#elif defined(INS_CHIBIOS)
	INS_read_DRDY(timeout, read_raw_gyro, _gpio_DRDY_G, _DRDY_pin_G);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS0::read_gyro_STATUS(uint32_t timeout){
	INS_read_STATUS(timeout, read_raw_gyro, status_gyro, (1 << 3));
}

//--------------------Check biases------------------------//
uint8_t LSM9DS0::check_gyro_biases(float bx, float by, float bz){
// Define Threshold based on Full-Scale value
	float thrs;
	if ((_sc_fact_g - 8.75e-3f * INS_TORAD) < 1e-5f){
		thrs = 10 * INS_TORAD * 1.2f; //typical 10dps offset
	}
	else if ((_sc_fact_g - 17.5e-3f * INS_TORAD) < 1e-5f){
		thrs = 15 * INS_TORAD * 1.2f;	//typical 15dps offset
	}
	else if ((_sc_fact_g - 70e-3f * INS_TORAD) < 1e-5f){
		thrs = 25 * INS_TORAD * 1.2f;	//typical 25dps offset
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
	LSM9DS0_READ_REGISTER_G(LSM9DS0_REFERENCE_G);
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
	for (uint8_t ii = 0; ii < LSM9DS0_GYRO_SELF_TEST_MEASURES; ii++){
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
	uint8_t CTRL4_val = LSM9DS0_READ_REGISTER_G(LSM9DS0_CTRL_REG4_G);
	if (!mode){
		CTRL4_val |= (1 << 1); // Self-test mode 0
	}
	else {
		CTRL4_val |= ((1 << 1) | (1 << 2)); // Self-test mode 1
	}
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_CTRL_REG4_G, CTRL4_val);
	turn_on_gyro();
	// Discard the first n measures 
	if( ! discard_measures_gyro(LSM9DS0_DISCARDED_MEASURES_ST, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (uint8_t ii = 0; ii < LSM9DS0_GYRO_SELF_TEST_MEASURES; ii++){
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
	if ((_sc_fact_g - 8.75e-3f * INS_TORAD) < 1e-5f){
		thrs_min = 20.0f * INS_TORAD;
		thrs_max = 250.0f * INS_TORAD;
	}
	else if ((_sc_fact_g - 17.5e-3f * INS_TORAD) < 1e-5f){
		thrs_min = 70.0f * INS_TORAD;
		thrs_max = 400.0f * INS_TORAD;
	}
	else if ((_sc_fact_g - 70e-3f * INS_TORAD) < 1e-5f){
		thrs_min = 150.0f * INS_TORAD;
		thrs_max = 1000.0f * INS_TORAD;
	}
	else {
		return 0;
	}
	// Check if values are bigger than the threshold
	if (INS_ch_st(x_pre, x_post, thrs_min, thrs_max) && INS_ch_st(y_pre, y_post, thrs_min, thrs_max) && INS_ch_st(z_pre, z_post, thrs_min, thrs_max)) {
		status = 1;
	}
	turn_off_gyro();
	CTRL4_val &= 0xF9; // Removes Self-Test
	LSM9DS0_WRITE_REGISTER_G(LSM9DS0_CTRL_REG4_G, CTRL4_val);
	turn_on_gyro();
	// Discard the first n measures
	if(! discard_measures_gyro(LSM9DS0_DISCARDED_MEASURES_ST, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Gyro Status------------------------//
uint8_t LSM9DS0::status_gyro(){
	return LSM9DS0_READ_REGISTER_G(LSM9DS0_STATUS_REG_G);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS0::discard_measures_gyro(uint8_t number_of_measures, uint32_t timeout){
	INS_discard_measures_over(number_of_measures, timeout, read_raw_gyro, status_gyro, (1 << 7));
}

//=============================Public Members Accelerometer====================================//
//-----------------------Configuration-----------------------//
uint8_t LSM9DS0::config_accel_mag(uint8_t accel_range, uint8_t accel_odr, uint8_t accel_bw, uint8_t mag_range, uint8_t mag_odr,uint8_t HP_accel_enable){
	init();
	// Trash the first reading
	LSM9DS0_READ_REGISTER_XM(LSM9DS0_WHO_AM_I_XM);
	// Check if the device ID is correct
	if (LSM9DS0_READ_REGISTER_XM(LSM9DS0_WHO_AM_I_XM) != LSM9DS0_ID_XM){
		return 0;
	}
	//
	// Enable FIFO Mode
	uint8_t CTRL0_val = 0x0|(1 << 6);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG0_XM, CTRL0_val);
	//
	switch (accel_range){
		case (LSM9DS0_RANGE_A_2):
			_sc_fact_a = INS_G_VAL * 0.00006103515625f;
			break;
		case (LSM9DS0_RANGE_A_4):
			_sc_fact_a = INS_G_VAL * 0.0001220703125f;
			break;
		case (LSM9DS0_RANGE_A_6):
			_sc_fact_a = INS_G_VAL * 0.00018310546875f;
			break;
		case (LSM9DS0_RANGE_A_8):
			_sc_fact_a = INS_G_VAL * 0.000244140625f;
			break;
		case (LSM9DS0_RANGE_A_16):
			_sc_fact_a = INS_G_VAL * 0.000732421875f;
			break;
		default:
			return 0;
	}
	//
	// Set selcted anti-alias and full-scale (accelerometer), disable self-test, 4-wire SPI
	uint8_t CTRL2_val = (accel_bw << 6) | (accel_range << 3);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG2_XM, CTRL2_val);
	//
	// Accelerometer data ready on interrupt 1
	uint8_t CTRL3_val = (1 << 2);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG3_XM, CTRL3_val);
	//
	// Magnetometer data ready on interrupt 2
	uint8_t CTRL4_val = (1 << 2);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG4_XM, CTRL4_val);
	//
	// Enable temperature sensor, set high-resolution for magnetometer, set selected ODR (magnetometer)
	uint8_t CTRL5_val = (1 << 7) | (1 << 6) | (1 << 5) | (mag_odr << 2);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG5_XM, CTRL5_val);
	//
	switch (mag_range){
		case (LSM9DS0_RANGE_M_2):
			_sc_fact_m = 0.08e-3f;
			break;
		case (LSM9DS0_RANGE_M_4):
			_sc_fact_m = 0.16e-3f;
			break;
		case (LSM9DS0_RANGE_M_8):
			_sc_fact_m = 0.32e-3f;
			break;
		case (LSM9DS0_RANGE_M_12):
			_sc_fact_m = 0.48e-3f;
			break;
		default:
			return 0;
	}
	//
	// Set selected full-scale (magnetometer)
	uint8_t CTRL6_val = (mag_range << 5);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG6_XM, CTRL6_val);
	//
	// Bypass mode on FIFO
	uint8_t FIFO_CTRL_val = 0x0;
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_FIFO_CTRL_REG, FIFO_CTRL_val);
	//
	// Clear the Reference/Offset registers
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_OFFSET_X_L_M, 0x00);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_OFFSET_X_H_M, 0x00);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_OFFSET_Y_L_M, 0x00);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_OFFSET_Y_H_M, 0x00);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_OFFSET_Z_L_M, 0x00);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_OFFSET_Z_H_M, 0x00);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_REFERENCE_X, 0x00);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_REFERENCE_Y, 0x00);
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_REFERENCE_Z, 0x00);
	// Set HP filter on accelerometer, magnetometer on continuous conversion
	_CTRL7_val_XM = 0x0;
	if (HP_accel_enable){
		_CTRL7_val_XM = (1 << 5);
	}
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG7_XM, _CTRL7_val_XM);
	//
	// Set selected ODR (accelerometer), continuous update, turn on the accelerometer
	_CTRL1_val_XM = (accel_odr << 4) | 0x7;
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG1_XM, _CTRL1_val_XM);
#ifdef INS_ARDUINO
	delay(200);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(200);
#endif
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
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG1_XM, _CTRL1_val_XM);
#ifdef INS_ARDUINO
	delay(200);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(200);
#endif
}

//----------------Turn off accelerometer---------------//
void LSM9DS0::turn_off_accel(){
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG1_XM, (_CTRL1_val_XM & 0xF));
}

//-----------------Read accelerometer------------------//
uint8_t LSM9DS0::read_raw_accel(){
	uint8_t buffer[6];
	LSM9DS0_READ_MULTIPLE_REGISTERS_XM(buffer, 6, LSM9DS0_OUT_X_L_A);
	ax = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_a);
	ay = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_a);
	az = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_a);
	return 1;
}

//------------Read accelerometer when ready--------------//
uint8_t LSM9DS0::read_accel_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
	INS_read_DRDY(timeout, read_raw_accel, _DRDY_pin_A);
#elif defined(INS_CHIBIOS)
	INS_read_DRDY(timeout, read_raw_accel, _gpio_DRDY_A, _DRDY_pin_A);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS0::read_accel_STATUS(uint32_t timeout){
	INS_read_STATUS(timeout, read_raw_accel, status_accel, (1 << 3));
}

//--------------------Check biases------------------------//
uint8_t LSM9DS0::check_accel_biases(float bx, float by, float bz){
	float thrs = 60e-3f * INS_G_VAL * 1.1f; //typical 60mg zero-G level
	if ((fabs(bx) > thrs) || (fabs(by) > thrs) || (fabs(bz) > thrs)){
		return 0;
	}
	return 1;
}

//---------------Reset high-pass filter-----------------//		
void LSM9DS0::HP_reset_accel(){
	LSM9DS0_READ_REGISTER_XM(LSM9DS0_REFERENCE_X);
	LSM9DS0_READ_REGISTER_XM(LSM9DS0_REFERENCE_Y);
	LSM9DS0_READ_REGISTER_XM(LSM9DS0_REFERENCE_Z);
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
	for (uint16_t ii = 0; ii < LSM9DS0_ACCEL_SELF_TEST_MEASURES; ii++){
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
	uint8_t CTRL2_val = LSM9DS0_READ_REGISTER_XM(LSM9DS0_CTRL_REG2_XM);
	if (!mode){
		CTRL2_val |= (1 << 1); // Self-test mode 0 (positive)
	}
	else {
		CTRL2_val |= (1 << 2); // Self-test mode 1 (negative)
	}
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG2_XM, CTRL2_val);
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(LSM9DS0_DISCARDED_MEASURES_ST,LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	// Average n samples
	float x_post = 0;
	float y_post = 0;
	float z_post = 0;
	for (uint16_t ii = 0; ii < LSM9DS0_ACCEL_SELF_TEST_MEASURES; ii++){
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
	if ((_sc_fact_a - INS_G_VAL * 0.00006103515625f) < 1e-5f){
		thrs_min = 60e-3f * INS_G_VAL;
		thrs_max = 1700e-3f * INS_G_VAL;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.0001220703125f) < 1e-5f){
		thrs_min = 60e-3f * INS_G_VAL;
		thrs_max = 1700e-3f * INS_G_VAL;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.00018310546875f) < 1e-5f){
		thrs_min = 60e-3f * INS_G_VAL;
		thrs_max = 1700e-3f * INS_G_VAL;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.000244140625f) < 1e-5f){
		thrs_min = 60e-3f * INS_G_VAL;
		thrs_max = 1700e-3f * INS_G_VAL;
	}
	else if ((_sc_fact_a - INS_G_VAL * 0.000732421875f) < 1e-5f){
		thrs_min = 60e-3f * INS_G_VAL;
		thrs_max = 1700e-3f * INS_G_VAL;
	}
	else {
		return 0;
	}
	// Check if values are bigger than the threshold
	if (INS_ch_st(x_pre, x_post, thrs_min, thrs_max) && INS_ch_st(y_pre, y_post, thrs_min, thrs_max) && INS_ch_st(z_pre, z_post, thrs_min, thrs_max)) {
		status = 1;
	}
	turn_off_accel();
	CTRL2_val &= 0xF9; // Removes Self-Test
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG2_XM, CTRL2_val);
	turn_on_accel();
	// Discard the first n measures
	if(! discard_measures_accel(LSM9DS0_DISCARDED_MEASURES_ST, LSM9DS0_DISCARD_TIMEOUT)){
		return 0;
	}
	return status;
}

//----------------------Accel Status------------------------//
uint8_t LSM9DS0::status_accel(){
	return LSM9DS0_READ_REGISTER_XM(LSM9DS0_STATUS_REG_A);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS0::discard_measures_accel(uint8_t number_of_measures, uint32_t timeout){
	INS_discard_measures_over(number_of_measures, timeout, read_raw_accel, status_accel, (1 << 7));
}

//=============================Public Members Magnetometer===================================//
//-----------------Turn on magnetometer----------------//
void LSM9DS0::turn_on_mag(){
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG7_XM, _CTRL7_val_XM);
#ifdef INS_ARDUINO
	delay(200);
#elif defined(INS_CHIBIOS)
	chThdSleepMilliseconds(200);
#endif
}

//-----------------Turn off magnetometer---------------//
void LSM9DS0::turn_off_mag(){
	LSM9DS0_WRITE_REGISTER_XM(LSM9DS0_CTRL_REG7_XM, (_CTRL7_val_XM | 0x3));
}

//------------------Read magnetometer------------------//
uint8_t LSM9DS0::read_raw_mag(){
	uint8_t buffer[6];
	LSM9DS0_READ_MULTIPLE_REGISTERS_XM(buffer, 6, LSM9DS0_OUT_X_L_M);
	mx = (float) (((int16_t) (buffer[1] << 8) | buffer[0]) * _sc_fact_m);
	my = (float) (((int16_t) (buffer[3] << 8) | buffer[2]) * _sc_fact_m);
	mz = (float) (((int16_t) (buffer[5] << 8) | buffer[4]) * _sc_fact_m);
	return 1;
}

//------------Read magnetometer when ready-------------//
uint8_t LSM9DS0::read_mag_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
	INS_read_DRDY(timeout, read_raw_mag, _DRDY_pin_M);
#elif defined(INS_CHIBIOS)
	INS_read_DRDY(timeout, read_raw_mag, _gpio_DRDY_M, _DRDY_pin_M);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS0::read_mag_STATUS(uint32_t timeout){
	INS_read_STATUS(timeout, read_raw_mag, status_mag, (1 << 3));
}

//-----------------------Mag Status------------------------//
uint8_t LSM9DS0::status_mag(){
	return LSM9DS0_READ_REGISTER_XM(LSM9DS0_STATUS_REG_M);
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS0::discard_measures_mag(uint8_t number_of_measures, uint32_t timeout){
	INS_discard_measures_over(number_of_measures, timeout, read_raw_mag, status_mag, (1 << 7));
}

//=============================Public Members Temperature====================================//
//------------------------Read data----------------------//
uint8_t LSM9DS0::read_raw_thermo(){
	uint8_t buffer[2];
	LSM9DS0_READ_MULTIPLE_REGISTERS_XM(buffer, 2, LSM9DS0_OUT_TEMP_L_XM);
	int16_t temperature_tmp = (((int16_t) buffer[1] << 12) | buffer[0] << 4) >> 4;
	temperature = (float) 20 + temperature_tmp * 0.125f; // Guessing that the intercept is at about 20°C 
	return 1;
}

//------------------Read data when ready-----------------//
uint8_t LSM9DS0::read_thermo_DRDY(uint32_t timeout){
#ifdef INS_ARDUINO
	INS_read_DRDY(timeout, read_raw_thermo, _DRDY_pin_M);
#elif defined(INS_CHIBIOS)
	INS_read_DRDY(timeout, read_raw_thermo, _gpio_DRDY_M, _DRDY_pin_M);
#endif
}

//---------Read data when ready (STATUS register)-----------//
uint8_t LSM9DS0::read_thermo_STATUS(uint32_t timeout){
	INS_read_STATUS(timeout, read_raw_thermo, status_mag, (1 << 3));
}

//-------------------Discard measures----------------------//
uint8_t LSM9DS0::discard_measures_thermo(uint8_t number_of_measures, uint32_t timeout){
	INS_discard_measures(number_of_measures, timeout, read_thermo_STATUS);
}
