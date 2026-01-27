#include <Arduino.h>
#include "pwmgen.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CORE_TO_BIND 1
#define PCNT_H_LIM 32767
#define PCNT_L_LIM -32768

static inline float clamp_duty(float d){
    if (d < 0) return 0;
    if (d > 100) return 100;
    return d;
}

void pwm_gen::begin(int chan, uint32_t frequency, float duty)
{   
    duty = clamp_duty(duty);
    pwm_config = {};
    channel = {};
    channel.unit = MCPWM_UNIT_0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    pwm_config.frequency = frequency;
    pwm_config.cmpr_a = duty;
    switch (chan)
    {
    case 1:
        channel.num = 1;
        channel.phase = MCPWM0A;
        channel.generator = MCPWM_GEN_A;
        channel.pwm_pin = GPIO_NUM_27;
        channel.timer = MCPWM_TIMER_0;
        break;

    case 2:
        channel.num = 2;
        channel.phase = MCPWM1A;
        channel.generator = MCPWM_GEN_A;
        channel.pwm_pin = GPIO_NUM_17;
        channel.timer = MCPWM_TIMER_1;
        break;

    case 3:
        channel.num = 3;
        channel.phase = MCPWM2A;
        channel.generator = MCPWM_GEN_A;
        channel.pwm_pin = GPIO_NUM_19;
        channel.timer = MCPWM_TIMER_2;
        break;

    default:
        break;
    }
    mcpwm_gpio_init(channel.unit, channel.phase, channel.pwm_pin);
    mcpwm_init(channel.unit, channel.timer, &pwm_config);
    mcpwm_set_duty_type(channel.unit, channel.timer, channel.generator, MCPWM_DUTY_MODE_0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void pwm_gen::set_frequency(uint32_t frequency)
{
    mcpwm_set_frequency(channel.unit, channel.timer, frequency);
}

void pwm_gen::set_duty(float duty)
{
    duty = clamp_duty(duty);
    mcpwm_set_duty(channel.unit, channel.timer, channel.generator, duty);
}

void pwm_gen::start_pwm(void)
{
    mcpwm_start(channel.unit, channel.timer);
}

void pwm_gen::stop_pwm(void)
{
    mcpwm_set_signal_low(channel.unit, channel.timer, channel.generator);
    mcpwm_stop(channel.unit, channel.timer);
}
