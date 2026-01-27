#pragma once
#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#include "pwmgen.h"
#define LIDAR_UART   UART_NUM_2
#define UART_TX_PIN  GPIO_NUM_26
#define UART_RX_PIN  GPIO_NUM_33

class LidarA1
{
public:
    /**
     * @param uart_port  UART_NUM_1 hoặc UART_NUM_2 (khuyên UART_NUM_2)
     * @param tx_pin     ESP TX -> Lidar RX (ví dụ GPIO26)
     * @param rx_pin     ESP RX <- Lidar TX (ví dụ GPIO33)
     * @param baud       115200
     * @param pwm_case   1/2/3 theo pwm_gen::begin() (case 3 là GPIO19)
     * @param pwm_freq   ví dụ 20000
     * @param duty       0..100 (gợi ý 50~70)
     */
    esp_err_t begin(uart_port_t uart_port,
                    gpio_num_t tx_pin,
                    gpio_num_t rx_pin,
                    int baud,
                    int pwm_case,
                    uint32_t pwm_freq,
                    float duty);

    // Motor control (qua pwm_gen)
    void motor_set(float duty);
    void motor_set_freq(uint32_t freq);
    void motor_start();
    void motor_stop();

    // Checking func
    int get_info();   // 0x50
    int get_health(); // 0x52

    // UART raw
    int uart_write_bytes_raw(const uint8_t* data, int len);

    // Minimal RPLidar commands (0xA5 + cmd)
    int send_cmd(uint8_t cmd);
    int stop();        // 0x25
    int reset();       // 0x40
    int start_scan();  // 0x20

private:
    static float clamp_duty(float d);

    esp_err_t uart_init_(uart_port_t uart_port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud);

private:
    uart_port_t uart_ = UART_NUM_2;
    gpio_num_t tx_ = GPIO_NUM_NC;
    gpio_num_t rx_ = GPIO_NUM_NC;

    pwm_gen motor_pwm_;
    bool uart_ready_ = false;
};

#endif