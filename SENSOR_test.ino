#include <SPI.h>
#include "MatLib.h"
#include "InertialSensor.h"
//#include "InertialEstimator.h"
#define T 40 //loop time (milliseconds)
uint32_t last_time = millis();

LSM9DS0 lsm(8, 9);
//L3GD20H l3g(7);
INS_Gyro Gyro1(lsm, lsm.gx, lsm.gy, lsm.gz,X_FRONT_Z_UP);
//INS_Gyro Gyro2(l3g, l3g.x, l3g.y, l3g.z,X_FRONT_Z_UP);
INS_Accel Accel1(lsm, lsm.ax, lsm.ay, lsm.az,X_FRONT_Z_UP);
INS_Mag Mag(lsm, lsm.mx, lsm.my, lsm.mz,X_FRONT_Z_UP);
INS_Thermo Thermo(lsm, lsm.temperature);

void setup() {
  pinMode(10, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  Serial.begin(115200);
  Serial.setTimeout(200);
  lsm.init();
  Serial.println("ciao");
  uint8_t stat_gyro_1 = lsm.config_gyro(LSM9DS0_RANGE_G_500, LSM9DS0_ODR_G_190_C0_70, 0, 0, 0);
  uint8_t stat_xm = lsm.config_accel_mag(LSM9DS0_RANGE_A_4, LSM9DS0_ODR_A_200, LSM9DS0_BW_A_50, LSM9DS0_RANGE_M_8, LSM9DS0_ODR_M_100, 0);
  //uint8_t stat_gyro_2 = l3g.config_gyro(L3GD20H_RANGE_245, L3GD20H_ODR_200_CO_n, 0, 0, 0); //range 500 DPS, ODR 200 without LPF2, LPF2 disabled, HPF disabled, HPF freq 0

  if (stat_gyro_1 == 1) {
    Serial.println("Gyro initialized");
  }
  else {
    Serial.println("Gyro not initialized!!!");
    while (1);
  }
  if (stat_xm == 1) {
    Serial.println("Accelerometer and magnetometer initialized");
  }
  else {
    Serial.println("Accelerometer and magnetometer not initialized!!!");
    while (1);
  }
  if (Gyro1.calibrate(200, 2e6)) {
    Serial.print("Bias X: ");
    Serial.print(Gyro1.bx, 4);
    Serial.print(", Bias Y: ");
    Serial.print(Gyro1.by, 4);
    Serial.print(", Bias Z: ");
    Serial.println(Gyro1.bz, 4);
  }
  Accel1.calibrate_average();
}

void loop() {
  uint32_t now = millis();
  if ((now - last_time) >= T) {
    last_time = millis();
    if (Gyro1.read(5e4)) {
      Serial.print("Gyro X: ");
      Serial.print(Gyro1.x, 4);
      Serial.print(", Gyro Y: ");
      Serial.print(Gyro1.y, 4);
      Serial.print(", Gyro Z: ");
      Serial.print(Gyro1.z, 4);
    }
    if (Accel1.read(5e4)) {
      Serial.print(", Accel X: ");
      Serial.print(Accel1.x, 4);
      Serial.print(", Accel Y: ");
      Serial.print(Accel1.y, 4);
      Serial.print(", Accel Z: ");
      Serial.println(Accel1.z, 4);
    }
  }
}
