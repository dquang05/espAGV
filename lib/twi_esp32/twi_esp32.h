#pragma once

extern "C" {
#include "driver/i2c.h"
#include "esp_err.h"
}

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  i2c_port_t port;
  gpio_num_t sda;
  gpio_num_t scl;
  uint32_t   clk_hz;     // 100000 / 400000 ...
  bool       pullup;     // internal pullup enable/disable
} twi_bus_t;

// Init + optional scan
esp_err_t twi_init(const twi_bus_t* bus);
esp_err_t twi_deinit(i2c_port_t port);

esp_err_t twi_write_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg, uint8_t val);
esp_err_t twi_write_bytes(i2c_port_t port, uint8_t dev_addr, uint8_t reg, const uint8_t* data, size_t len);

esp_err_t twi_read_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg, uint8_t* out_val);
esp_err_t twi_read_bytes(i2c_port_t port, uint8_t dev_addr, uint8_t reg, uint8_t* out, size_t len);

// Utility: scan 0x03..0x77, call cb(addr, ok)
typedef void (*twi_scan_cb_t)(uint8_t addr, bool ok);
esp_err_t twi_scan(i2c_port_t port, twi_scan_cb_t cb);

#ifdef __cplusplus
}
#endif
