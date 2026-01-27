#include "lidar.h"
#include <string.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
static const char* TAG_LIDAR = "LIDAR";


static constexpr int UART_RX_BUF = 8 * 1024;
static constexpr int UART_TX_BUF = 2 * 1024;

float LidarA1::clamp_duty(float d)
{
    if (d < 0) return 0;
    if (d > 100) return 100;
    return d;
}

esp_err_t LidarA1::uart_init_(uart_port_t uart_port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud)
{
    uart_config_t cfg = {};
    cfg.baud_rate = baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;

    // Cài driver UART
    esp_err_t e = uart_driver_install(uart_port, UART_RX_BUF, UART_TX_BUF, 0, nullptr, 0);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        return e;
    }

    ESP_ERROR_CHECK(uart_param_config(uart_port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_ = uart_port;
    tx_ = tx_pin;
    rx_ = rx_pin;
    uart_ready_ = true;
    return ESP_OK;
}

esp_err_t LidarA1::begin(uart_port_t uart_port,
                        gpio_num_t tx_pin,
                        gpio_num_t rx_pin,
                        int baud,
                        int pwm_case,
                        uint32_t pwm_freq,
                        float duty)
{
    // 1) init UART
    esp_err_t e = uart_init_(uart_port, tx_pin, rx_pin, baud);
    if (e != ESP_OK) return e;

    // 2) init motor PWM
    duty = clamp_duty(duty);
    motor_pwm_.begin(pwm_case, pwm_freq, duty);
    motor_pwm_.start_pwm();

    return ESP_OK;
}

/* ===== Motor wrappers ===== */

void LidarA1::motor_set(float duty)
{
    motor_pwm_.set_duty(clamp_duty(duty));
}

void LidarA1::motor_set_freq(uint32_t freq)
{
    motor_pwm_.set_frequency(freq);
}

void LidarA1::motor_start()
{
    motor_pwm_.start_pwm();
}

void LidarA1::motor_stop()
{
    motor_pwm_.stop_pwm();
}

/* ===== UART raw + commands ===== */

int LidarA1::uart_write_bytes_raw(const uint8_t* data, int len)
{
    if (!uart_ready_ || !data || len <= 0) return -1;
    return uart_write_bytes(uart_, (const char*)data, len);
}

int LidarA1::send_cmd(uint8_t cmd)
{
    uint8_t pkt[2] = {0xA5, cmd};
    int n = uart_write_bytes_raw(pkt, 2);
    ESP_LOGI(TAG_LIDAR, "send_cmd 0xA5 0x%02X -> wrote %d bytes", cmd, n);
    return n;
    
}

int LidarA1::stop()
{
    return send_cmd(0x25);
}

int LidarA1::reset()
{
    return send_cmd(0x40);
}

int LidarA1::start_scan()
{
    return send_cmd(0x20);
}

// Some checking func
int LidarA1::get_info()   { return send_cmd(0x50); }
int LidarA1::get_health() { return send_cmd(0x52); }
