#include "twi_esp32.h"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

static SemaphoreHandle_t s_i2c_mu = nullptr;
static StaticSemaphore_t s_i2c_mu_buffer;

// Some helper inline functions
static inline void lock_bus()   { if (s_i2c_mu) xSemaphoreTake(s_i2c_mu, portMAX_DELAY); }
static inline void unlock_bus() { if (s_i2c_mu) xSemaphoreGive(s_i2c_mu); }

esp_err_t twi_init(const twi_bus_t* bus)
{
    if (!bus) return ESP_ERR_INVALID_ARG;

    // More safe init
    if (!s_i2c_mu) {
        s_i2c_mu = xSemaphoreCreateMutexStatic(&s_i2c_mu_buffer);
    }

    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = bus->sda;
    conf.scl_io_num = bus->scl;
    conf.sda_pullup_en = bus->pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = bus->pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = bus->clk_hz;

    esp_err_t err = i2c_param_config(bus->port, &conf);
    if (err != ESP_OK) return err;

    return i2c_driver_install(bus->port, conf.mode, 0, 0, 0);
}

//Core function
esp_err_t twi_write_bytes(i2c_port_t port, uint8_t dev_addr, uint8_t reg, const uint8_t* data, size_t len)
{
    lock_bus();
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (data && len > 0) {
        i2c_master_write(cmd, (uint8_t*)data, len, true);
    }
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    unlock_bus();
    return err;
}

esp_err_t twi_read_bytes(i2c_port_t port, uint8_t dev_addr, uint8_t reg, uint8_t* out, size_t len)
{
    if (!out || len == 0) return ESP_ERR_INVALID_ARG;

    lock_bus();
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Phase 1: write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    // Phase 2: Repeated Start to read data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, out, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, out + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    unlock_bus();
    return err;
}

// Wrapper for convenience
esp_err_t twi_write_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg, uint8_t val) {
    return twi_write_bytes(port, dev_addr, reg, &val, 1);
}

esp_err_t twi_read_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg, uint8_t* out_val) {
    return twi_read_bytes(port, dev_addr, reg, out_val, 1);
}