#pragma once
#ifndef PWMGEN_H
#define PWMGEN_H
#include "driver/mcpwm.h"

struct chan
{
    // PWM setting
    int num;
    mcpwm_unit_t unit = MCPWM_UNIT_0;
    mcpwm_io_signals_t phase;
    mcpwm_generator_t generator = MCPWM_GEN_A;
    gpio_num_t pwm_pin;
    mcpwm_timer_t timer;
    // Capture setting
    gpio_num_t cap_pin;
    mcpwm_capture_on_edge_t cap_edg = MCPWM_POS_EDGE;
    mcpwm_capture_signal_t cap_sig;
};

extern volatile uint32_t signal_count_0;
extern volatile uint32_t signal_count_1;
extern volatile uint32_t total_count_0;
extern volatile uint32_t total_count_1;

class pwm_gen
{
public:
    void begin(int channel, uint32_t frequency, float duty);
    void capture_config(int unit);
    void set_frequency(uint32_t frequency);
    void set_duty(float duty);
    void start_pwm(void);
    void stop_pwm(void);
    uint32_t take_value(void);
    volatile uint32_t speed;

private:
    mcpwm_config_t pwm_config;
    mcpwm_capture_config_t cap_conf;
    volatile uint32_t pre_capture = 0;
    chan channel;
};
#endif