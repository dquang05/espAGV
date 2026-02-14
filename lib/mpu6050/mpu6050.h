#pragma once

#include <stdint.h>
#include <stddef.h>

extern "C" {
#include "driver/i2c.h"
#include "esp_err.h"
}

typedef struct {
  int16_t ax, ay, az;
  int16_t temp;
  int16_t gx, gy, gz;
} mpu6050_raw_t;

typedef struct {
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;
  float temp_c;
} mpu6050_scaled_t;

typedef enum {
  MPU_ACCEL_2G  = 0,
  MPU_ACCEL_4G  = 1,
  MPU_ACCEL_8G  = 2,
  MPU_ACCEL_16G = 3
} mpu_accel_fs_t;

typedef enum {
  MPU_GYRO_250DPS  = 0,
  MPU_GYRO_500DPS  = 1,
  MPU_GYRO_1000DPS = 2,
  MPU_GYRO_2000DPS = 3
} mpu_gyro_fs_t;

// Axis mapping 
enum Axis : uint8_t { AX_X = 0, AX_Y = 1, AX_Z = 2 };

struct AxisMap {
  Axis  src[3];   // bodyX/bodyY/bodyZ 
  int8_t sign[3]; // +1 hoặc -1
};

class MPU6050 {
public:
  MPU6050(i2c_port_t port, uint8_t addr = 0x68);

  esp_err_t begin();

  esp_err_t setSampleRate(uint16_t hz);
  esp_err_t setDLPF(uint8_t dlpf_cfg);
  esp_err_t setAccelRange(mpu_accel_fs_t fs);
  esp_err_t setGyroRange(mpu_gyro_fs_t fs);

  // Debug / health check (public wrapper, KHÔNG lộ readReg_)
  esp_err_t whoAmI(uint8_t* out);

  // Read
  esp_err_t readRaw(mpu6050_raw_t* out);
  esp_err_t readScaled(mpu6050_scaled_t* out);

  // Read + map theo AxisMap ra body axes
  esp_err_t readScaledBody(mpu6050_scaled_t* out_body);

  // Filter config + state
  void setAlpha(float a);
  float getAlpha() const { return alpha_; }
  void resetAngles(float roll_deg = 0.0f, float pitch_deg = 0.0f);

  // Compute angles (NO I2C) from already-read body data
  esp_err_t computeAnglesFromScaled(const mpu6050_scaled_t& s_body, float* roll, float* pitch, float dt_s);

  // Read once + compute angles 
  esp_err_t readScaledAndAngles(mpu6050_scaled_t* out_s_body, float* roll, float* pitch, float dt_s);

  // Compute angles (I2C inside)
  esp_err_t computeAngles(float* roll_deg, float* pitch_deg, float dt_s);

  // Calibration
  esp_err_t calibrateGyro(uint16_t samples, uint16_t sample_delay_ms = 5);

  // Bias + mapping
  void setAccelBiasRaw(int16_t bax, int16_t bay, int16_t baz);
  void setGyroBiasRaw(int16_t bgx, int16_t bgy, int16_t bgz);
  void getGyroBiasRaw(int16_t& bgx, int16_t& bgy, int16_t& bgz) const;
  void getAccelBiasRaw(int16_t& bax, int16_t& bay, int16_t& baz) const;

  void setAxisMap(const AxisMap& m);
  AxisMap getAxisMap() const { return map_; }

  float rollDeg() const { return roll_deg_; }
  float pitchDeg() const { return pitch_deg_; }

private:
  i2c_port_t port_;
  uint8_t addr_;

  mpu_accel_fs_t accel_fs_ = MPU_ACCEL_2G;
  mpu_gyro_fs_t  gyro_fs_  = MPU_GYRO_250DPS;

  // Bias RAW 
  int16_t accel_bias_[3] = {0,0,0}; 
  int16_t gyro_bias_[3]  = {0,0,0};

  AxisMap map_{{AX_X, AX_Y, AX_Z}, {-1, -1, +1}};

  float alpha_ = 0.96f;
  float roll_deg_ = 0.0f;
  float pitch_deg_ = 0.0f;

  // Helpers
  float accelLSBperG_() const;
  float gyroLSBperDPS_() const;

  static void applyMap_(const AxisMap& m,
                        float sx, float sy, float sz,
                        float& bx, float& by, float& bz);

  esp_err_t writeReg_(uint8_t reg, uint8_t val);
  esp_err_t readReg_(uint8_t reg, uint8_t* val);
  esp_err_t readBytes_(uint8_t reg, uint8_t* out, size_t len);
};
