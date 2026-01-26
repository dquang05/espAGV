#include <Arduino.h>
#include "pid.h"
#include "driver/timer.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

pid_conf _conf;
FuncPtr fn;

timer_conf _timer;

volatile uint32_t isrCounter = 0;
static TaskHandle_t s_control_task = NULL;

void pid_set_control_task_handle(TaskHandle_t h) {
    s_control_task = h;
}

void IRAM_ATTR pid_control_tick_isr(void) {
    if (!s_control_task) return;
    isrCounter++;
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static bool IRAM_ATTR timer_isr_2(void *para) {
    timer_spinlock_take(_timer.timer_gp);

    timer_group_clr_intr_status_in_isr(_timer.timer_gp, _timer.timer_idx);
    timer_group_enable_alarm_in_isr(_timer.timer_gp, _timer.timer_idx);
    timer_spinlock_give(_timer.timer_gp);
    if (fn) fn();   // giờ fn = pid_control_tick_isr()
    return false;
}


void pid_timer_set(timer_conf timer, FuncPtr cfn, float sampT){
    timer_config_t timer_conf{
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80 
    };

    _timer = timer;

    timer_init(_timer.timer_gp, _timer.timer_idx, &timer_conf);
    timer_set_counter_value(_timer.timer_gp, _timer.timer_idx, 0);
    timer_set_alarm_value(_timer.timer_gp, _timer.timer_idx, uint64_t(sampT*TIMER_BASE_CLK/timer_conf.divider));
    timer_set_alarm(_timer.timer_gp, _timer.timer_idx, TIMER_ALARM_EN);
    timer_enable_intr(_timer.timer_gp, _timer.timer_idx);

    fn = cfn;

    timer_isr_callback_add(_timer.timer_gp, _timer.timer_idx, timer_isr_2, NULL, 0);
}

void pid_start(void){
    timer_start(_timer.timer_gp, _timer.timer_idx);
}

void pid_stop(void){
    timer_pause(_timer.timer_gp, _timer.timer_idx);
}

void pid_gen::pid_begin(float sampT, float KP, float KI, float KD, volatile float *desire, volatile float *actual, float down_lim, float up_lim){
    _conf.sampT = sampT;
    _conf.Kp = KP;
    _conf.Ki = KI;
    _conf.Kd = KD;
    _conf.desire = desire;
    _conf.actual = actual;
    _conf.up_lim = up_lim;
    _conf.down_lim = down_lim;
    output = 0.0f;  // Reset output
    _conf.error = _conf.error1 = _conf.error2 = 0.0f;  // Reset errors
    pid_coef();   
}

void pid_gen::pid_coef(void){
    _conf.alpha = 2*_conf.sampT*_conf.Kp+_conf.Ki*_conf.sampT*_conf.sampT+2*_conf.Kd;
	_conf.beta = _conf.sampT*_conf.sampT*_conf.Ki -4*_conf.Kd -2*_conf.sampT*_conf.Kp;
	_conf.gama = 2*_conf.Kd;
}

float_t pid_gen::pid_calculate(float e, float e1,float e2){
	float cur_out = (_conf.alpha*e+_conf.beta*e1+_conf.gama*e2+2*_conf.sampT*output)/(2*_conf.sampT);
    return cur_out;
}

void pid_gen::pid_func(void){
    _conf.error = (*(_conf.desire)) - (*(_conf.actual));
    float val = pid_calculate(_conf.error,_conf.error1,_conf.error2);
    if (val > _conf.up_lim){
        output =  _conf.up_lim;
    }
    else if (val < _conf.down_lim){
        output = _conf.down_lim;
    }
    else{
        output = val;
    }
    
    _conf.error2 = _conf.error1;
    _conf.error1 = _conf.error;
}