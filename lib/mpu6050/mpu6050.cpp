#include "mpu6050.h"
#include "twi_esp32.h"
#include <math.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

static constexpr float RAD2DEG = 57.2957795f;

MPU6050::MPU6050(i2c_port_t port, uint8_t addr) : port_(port), addr_(addr) {}

esp_err_t MPU6050::writeReg_(uint8_t reg, uint8_t val) {
  return twi_write_reg(port_, addr_, reg, val);
}
esp_err_t MPU6050::readReg_(uint8_t reg, uint8_t* val) {
  return twi_read_reg(port_, addr_, reg, val);
}
esp_err_t MPU6050::readBytes_(uint8_t reg, uint8_t* out, size_t len) {
  return twi_read_bytes(port_, addr_, reg, out, len);
}

esp_err_t MPU6050::begin()
{
  uint8_t who = 0;
  esp_err_t err = readReg_(REG_WHO_AM_I, &who);
  if (err != ESP_OK) return err;
  if (who != 0x68 && who != 0x69) return ESP_ERR_NOT_FOUND;

  // Wake up
  err = writeReg_(REG_PWR_MGMT_1, 0x00);
  if (err != ESP_OK) return err;
  vTaskDelay(pdMS_TO_TICKS(50));

  // PLL clock (X gyro) for stability
  err = writeReg_(REG_PWR_MGMT_1, 0x01);
  if (err != ESP_OK) return err;

  // Defaults options
  setDLPF(3);            // popular value
  setSampleRate(100);    // 100 Hz
  setAccelRange(accel_fs_);
  setGyroRange(gyro_fs_);

  return ESP_OK;
}

esp_err_t MPU6050::setSampleRate(uint16_t hz)
{
  if (hz == 0) return ESP_ERR_INVALID_ARG;

  // Khi DLPF ON (CONFIG[2:0] != 0), internal rate ~ 1kHz
  // SMPLRT_DIV = (1000/hz) - 1
  uint16_t div = (1000 / hz);
  if (div == 0) div = 1;
  uint8_t reg = (uint8_t)(div - 1);
  return writeReg_(REG_SMPLRT_DIV, reg);
}

esp_err_t MPU6050::setDLPF(uint8_t dlpf_cfg)
{
  dlpf_cfg &= 0x07;
  return writeReg_(REG_CONFIG, dlpf_cfg);
}

esp_err_t MPU6050::setAccelRange(mpu_accel_fs_t fs)
{
  accel_fs_ = fs;
  uint8_t val = (uint8_t)(fs << 3);
  return writeReg_(REG_ACCEL_CONFIG, val);
}

esp_err_t MPU6050::setGyroRange(mpu_gyro_fs_t fs)
{
  gyro_fs_ = fs;
  uint8_t val = (uint8_t)(fs << 3);
  return writeReg_(REG_GYRO_CONFIG, val);
}

float MPU6050::accelLSBperG_() const
{
  switch (accel_fs_) {
    case MPU_ACCEL_2G:  return 16384.0f;
    case MPU_ACCEL_4G:  return 8192.0f;
    case MPU_ACCEL_8G:  return 4096.0f;
    case MPU_ACCEL_16G: return 2048.0f;
    default: return 16384.0f;
  }
}

float MPU6050::gyroLSBperDPS_() const
{
  switch (gyro_fs_) {
    case MPU_GYRO_250DPS:  return 131.0f;
    case MPU_GYRO_500DPS:  return 65.5f;
    case MPU_GYRO_1000DPS: return 32.8f;
    case MPU_GYRO_2000DPS: return 16.4f;
    default: return 131.0f;
  }
}

esp_err_t MPU6050::readRaw(mpu6050_raw_t* out)
{
  if (!out) return ESP_ERR_INVALID_ARG;

  uint8_t b[14];
  esp_err_t err = readBytes_(REG_ACCEL_XOUT_H, b, sizeof(b));
  if (err != ESP_OK) return err;

  auto be16 = [](uint8_t hi, uint8_t lo) -> int16_t {
    return (int16_t)((hi << 8) | lo);
  };

  out->ax   = be16(b[0],  b[1]);
  out->ay   = be16(b[2],  b[3]);
  out->az   = be16(b[4],  b[5]);
  out->temp = be16(b[6],  b[7]);
  out->gx   = be16(b[8],  b[9]);
  out->gy   = be16(b[10], b[11]);
  out->gz   = be16(b[12], b[13]);

  return ESP_OK;
}

esp_err_t MPU6050::readScaled(mpu6050_scaled_t* out)
{
  if (!out) return ESP_ERR_INVALID_ARG;

  mpu6050_raw_t r{};
  esp_err_t err = readRaw(&r);
  if (err != ESP_OK) return err;

  // Minus bias raw
  r.ax -= accel_bias_[0]; r.ay -= accel_bias_[1]; r.az -= accel_bias_[2];
  r.gx -= gyro_bias_[0];  r.gy -= gyro_bias_[1];  r.gz -= gyro_bias_[2];

  const float a_lsb = accelLSBperG_();
  const float g_lsb = gyroLSBperDPS_();

  out->ax_g = (float)r.ax / a_lsb;
  out->ay_g = (float)r.ay / a_lsb;
  out->az_g = (float)r.az / a_lsb;

  out->gx_dps = (float)r.gx / g_lsb;
  out->gy_dps = (float)r.gy / g_lsb;
  out->gz_dps = (float)r.gz / g_lsb;

  out->temp_c = ((float)r.temp / 340.0f) + 36.53f;
  return ESP_OK;
}

esp_err_t MPU6050::calibrateGyro(uint16_t samples, uint16_t sample_delay_ms)
{
  if (samples == 0) return ESP_ERR_INVALID_ARG;

  int64_t sgx=0, sgy=0, sgz=0;
  for (uint16_t i=0; i<samples; i++) {
    mpu6050_raw_t r{};
    esp_err_t err = readRaw(&r);
    if (err != ESP_OK) return err;

    sgx += r.gx; sgy += r.gy; sgz += r.gz;
    vTaskDelay(pdMS_TO_TICKS(sample_delay_ms));
  }

  gyro_bias_[0] = (int16_t)(sgx / samples);
  gyro_bias_[1] = (int16_t)(sgy / samples);
  gyro_bias_[2] = (int16_t)(sgz / samples);

  return ESP_OK;
}

void MPU6050::setAccelBiasRaw(int16_t bax, int16_t bay, int16_t baz)
{
  accel_bias_[0] = bax;
  accel_bias_[1] = bay;
  accel_bias_[2] = baz;
}

void MPU6050::setGyroBiasRaw(int16_t bgx, int16_t bgy, int16_t bgz)
{
  gyro_bias_[0] = bgx;
  gyro_bias_[1] = bgy;
  gyro_bias_[2] = bgz;
}

void MPU6050::getGyroBiasRaw(int16_t& bgx, int16_t& bgy, int16_t& bgz) const
{
  bgx = gyro_bias_[0];
  bgy = gyro_bias_[1];
  bgz = gyro_bias_[2];
}

void MPU6050::getAccelBiasRaw(int16_t& bax, int16_t& bay, int16_t& baz) const
{
  bax = accel_bias_[0];
  bay = accel_bias_[1];
  baz = accel_bias_[2];
}

void MPU6050::setAxisMap(const AxisMap& m)
{
  map_ = m;
}

void MPU6050::setAlpha(float a)
{
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  alpha_ = a;
}

void MPU6050::resetAngles(float roll_deg, float pitch_deg)
{
  roll_deg_ = roll_deg;
  pitch_deg_ = pitch_deg;
}

void MPU6050::applyMap_(const AxisMap& m,
                        float sx, float sy, float sz,
                        float& bx, float& by, float& bz)
{
  float v[3] = {sx, sy, sz};
  bx = (float)m.sign[0] * v[m.src[0]];
  by = (float)m.sign[1] * v[m.src[1]];
  bz = (float)m.sign[2] * v[m.src[2]];
}

esp_err_t MPU6050::computeAngles(float* roll_deg, float* pitch_deg, float dt_s)
{
  if (dt_s <= 0.0f) return ESP_ERR_INVALID_ARG;
  if (dt_s > 1.0f)  dt_s = 1.0f; // clamp max when delay too long

  // read scaled (already minus bias gyro/accel raw if you set)
  mpu6050_scaled_t s{};
  esp_err_t err = readScaled(&s);
  if (err != ESP_OK) return err;

  // Map sensor axes -> body axes (considering installation orientation)
  float ax, ay, az;
  float gx, gy, gz;
  applyMap_(map_, s.ax_g,   s.ay_g,   s.az_g,   ax, ay, az);
  applyMap_(map_, s.gx_dps, s.gy_dps, s.gz_dps, gx, gy, gz);

  // Angles from accel
  // roll  = atan2(ay, az)
  // pitch = atan2(-ax, sqrt(ay^2 + az^2))
  float roll_acc  = atan2f(ay, az) * RAD2DEG;
  float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD2DEG;

  // Complementary filter
  roll_deg_  = alpha_ * (roll_deg_  + gx * dt_s) + (1.0f - alpha_) * roll_acc;
  pitch_deg_ = alpha_ * (pitch_deg_ + gy * dt_s) + (1.0f - alpha_) * pitch_acc;

  if (roll_deg)  *roll_deg  = roll_deg_;
  if (pitch_deg) *pitch_deg = pitch_deg_;
  return ESP_OK;
}
