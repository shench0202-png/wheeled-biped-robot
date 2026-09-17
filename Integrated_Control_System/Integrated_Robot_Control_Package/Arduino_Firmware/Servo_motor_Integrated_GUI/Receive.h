#pragma once

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Wire.h>
#include <cppQueue.h>
#include <math.h>

#define IMU_SENSOR_MPU6050_QMC5883L 0
#define IMU_SENSOR_ICM20948 1

#ifndef ACTIVE_IMU_SENSOR
#define ACTIVE_IMU_SENSOR IMU_SENSOR_MPU6050_QMC5883L
#endif

#if ACTIVE_IMU_SENSOR == IMU_SENSOR_ICM20948
#include <ICM20948_WE.h>
#endif

class Receive {
private:

  float Acc_b[3], Rate_b[3];
  float Acc_a[3], Rate_a[3];
  float Acc_ab[3], Rate_ab[3];
  float Acc_raw[3], Rate_raw[3];

  const int MPU_addr = 0x68;
  const int QMC5883L_addr = 0x0D;
#if ACTIVE_IMU_SENSOR == IMU_SENSOR_ICM20948
  ICM20948_WE icm;
#endif
  bool icm_ready = false;

  float a[2] = {1.64927209f, -0.70219636f};
  float b[3] = {0.01323107f, 0.02646213f, 0.01323107f};
  float accX[6] = {0}, accY[6] = {0}, accZ[6] = {0};
  float gyroX[6] = {0}, gyroY[6] = {0}, gyroZ[6] = {0};

  float offsetRate[3] = {0};
  float offset_Arate[3] = {0};
  float offset_ABrate[3] = {0};
  float offsetRate_Raw[3] = {0};
  float offsetAcc[3] = {0};
  float offset_Aacc[3] = {0};
  float offset_ABacc[3] = {0};
  float offsetAcc_Raw[3] = {0};

  float alpha = 0.1f;
  float Beta = 0.005f;
  float a_accx[2] = {0}, a_accy[2] = {0}, a_accz[2] = {0};
  float a_ratex[2] = {0}, a_ratey[2] = {0}, a_ratez[2] = {0};
  float ab_accx[2] = {0}, ab_accy[2] = {0}, ab_accz[2] = {0};
  float ab_ratex[2] = {0}, ab_ratey[2] = {0}, ab_ratez[2] = {0};
  float abs_accx[2] = {0}, abs_accy[2] = {0}, abs_accz[2] = {0};
  float abs_ratex[2] = {0}, abs_ratey[2] = {0}, abs_ratez[2] = {0};

  float Mag_raw[3] = {0}, Mag_cal[3] = {0};












  float mag_bias[3] = {-913.9419702, 653.5418367, 604.2331443};
  float mag_soft_iron[3][3] = {{   0.0008217,   -0.0000397,   -0.0000785},
                               {  -0.0000397,    0.0009159,   -0.0001533},
                               {  -0.0000785,   -0.0001533,    0.0006701}};

  float ab_mx[2] = {0}, ab_my[2] = {0}, ab_mz[2] = {0};
  float abs_mx[2] = {0}, abs_my[2] = {0}, abs_mz[2] = {0};


public:
  bool mag_detected = false;
  bool mag_ok = false;
  float mag_norm_ref = 0.0f;
  float mag_norm_tol = 0.35f;
  float still_gyro_sum_thr = 3.0f;
  int mag_still_count = 0;
  const int STILL_CONFIRM_N = 10;

private:
  float *Butterworth_filter(float value, float arr[]);
  void Alpha_filter(float raw_data, float data[]);
  void Alpha_Beta_filter(float raw_data, float data_one[], float data_two[],
                         float dt);
  void resetFilterState();
  bool qmc_read_raw(float &mx, float &my, float &mz);
  bool isUsingICM20948() const { return ACTIVE_IMU_SENSOR == IMU_SENSOR_ICM20948; }

public:
  Receive();

  bool begin();
  void Offset();
  void DataRead(float dt);
  void Receive_get(float Rate[], float Acc[], int choose);

  void qmc_init();
  void getCalibratedMag(float mag_out[3], int choose);
  void getRawMag(float mag_out[3]);
  void applyMagCalibration();
  bool read_magnetometer(float dt);
  void mag_calibrate(uint32_t duration_ms);

  bool isMagOK() const { return mag_detected && mag_ok; }
  float getMagNorm() const;
};

