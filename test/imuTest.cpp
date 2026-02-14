#include <Arduino.h>

extern "C" {
#include "esp_timer.h"
#include "driver/i2c.h"
}

#include "twi_esp32.h"
#include "mpu6050.h"

static const gpio_num_t SDA_PIN = GPIO_NUM_21;
static const gpio_num_t SCL_PIN = GPIO_NUM_22;

// ====== I2C BUS CONFIG ======
static twi_bus_t bus = {
  .port   = I2C_NUM_0,
  .sda    = SDA_PIN,
  .scl    = SCL_PIN,
  .clk_hz = 400000,   // try 1000000 for long wires or noisy env
  .pullup = true
};

// ====== IMU OBJECT ======
static MPU6050 imu(I2C_NUM_0, 0x68);

// ====== OPTIONAL AXIS MAPPING ======
// static AxisMap map_cw90 = {{AX_Y, AX_X, AX_Z}, {+1, -1, +1}};

static uint64_t last_us = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("Setup here!");

  // Init I2C
  esp_err_t err = twi_init(&bus);
  if (err != ESP_OK) {
    Serial.printf("twi_init FAIL: %d\n", (int)err);
    while (1) delay(1000);
  }

  // Init MPU
  err = imu.begin();
  if (err != ESP_OK) {
    Serial.printf("MPU6050::begin FAIL: %d\n", (int)err);
    while (1) delay(1000);
  }

  // Basic config (optional)
  imu.setAccelRange(MPU_ACCEL_4G);
  imu.setGyroRange(MPU_GYRO_500DPS);
  imu.setDLPF(3);
  imu.setSampleRate(100);
  imu.setAlpha(0.96f);
  imu.resetAngles(0, 0);

  // Optional: set axis mapping 
  // imu.setAxisMap(map_cw90);

  err = imu.calibrateGyro(400, 5);
  Serial.printf("calibrateGyro: %d\n", (int)err);

  // Print bias
  int16_t bgx, bgy, bgz;
  imu.getGyroBiasRaw(bgx, bgy, bgz);
  Serial.printf("Gyro bias raw: %d, %d, %d\n", bgx, bgy, bgz);

  last_us = (uint64_t)esp_timer_get_time();

}

void loop() {
  uint64_t now_us = (uint64_t)esp_timer_get_time();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;

  // read scaled
  mpu6050_scaled_t s{};
  float roll=0, pitch=0;

  esp_err_t e1 = imu.readScaled(&s);
  esp_err_t e2 = imu.computeAngles(&roll, &pitch, dt);

  if (e1 == ESP_OK && e2 == ESP_OK) {
    uint32_t t_ms = (uint32_t)(now_us / 1000ULL);
    Serial.printf("%lu,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.2f,%.2f\n",
                  (unsigned long)t_ms,
                  s.ax_g, s.ay_g, s.az_g,
                  s.gx_dps, s.gy_dps, s.gz_dps,
                  roll, pitch);
  } else {
    Serial.printf("read err: %d %d\n", (int)e1, (int)e2);
    delay(200);
  }

  // ~100Hz
  delay(10);
}
