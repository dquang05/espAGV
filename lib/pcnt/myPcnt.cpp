
/*NOTE: CHANGE 
c0.lctrl_mode = PCNT_MODE_KEEP;
c0.hctrl_mode = PCNT_MODE_REVERSE;
TO REVERSE THE DIRECTION OF ENCODER COUNTING
 */

#include "myPcnt.h"
#include "driver/pcnt.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"

// Để đọc/clear interrupt status register trực tiếp (ESP32)
#include "soc/pcnt_struct.h"
#include "Arduino.h"
#include "soc/pcnt_reg.h"

#define ENC0_A GPIO_NUM_36
#define ENC0_B GPIO_NUM_39
#define ENC1_A GPIO_NUM_34
#define ENC1_B GPIO_NUM_35

#define PCNT_H_LIM 32767
#define PCNT_L_LIM -32768

static volatile int32_t s_accum0 = 0;
static volatile int32_t s_accum1 = 0;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ISR handler (ESP-IDF v4.4 style: pcnt_isr_register)
static void IRAM_ATTR pcnt_isr_handler(void *arg)
{
    (void)arg;
    uint32_t intr_status = PCNT.int_st.val;
    // UNIT 0
    if (intr_status & BIT(PCNT_UNIT_0))
    {
        uint32_t evt = 0;
        pcnt_get_event_status(PCNT_UNIT_0, &evt);

        portENTER_CRITICAL_ISR(&s_mux);
        if (evt & PCNT_EVT_H_LIM)
            s_accum0 += PCNT_H_LIM;
        if (evt & PCNT_EVT_L_LIM)
            s_accum0 += PCNT_L_LIM;
        portEXIT_CRITICAL_ISR(&s_mux);
        pcnt_counter_clear(PCNT_UNIT_0);   

        PCNT.int_clr.val = BIT(PCNT_UNIT_0);
    }

    // UNIT 1
    if (intr_status & BIT(PCNT_UNIT_1))
    {
        uint32_t evt = 0;
        pcnt_get_event_status(PCNT_UNIT_1, &evt);

        portENTER_CRITICAL_ISR(&s_mux);
        if (evt & PCNT_EVT_H_LIM)
            s_accum1 += PCNT_H_LIM;
        if (evt & PCNT_EVT_L_LIM)
            s_accum1 += PCNT_L_LIM;
        portEXIT_CRITICAL_ISR(&s_mux);
        pcnt_counter_clear(PCNT_UNIT_1);
        PCNT.int_clr.val = BIT(PCNT_UNIT_1);
    }
}

static void pcnt_setup_unit_quadrature(pcnt_unit_t unit, gpio_num_t pinA, gpio_num_t pinB)
{
    // Channel 0: A = pulse, B = ctrl 
    pcnt_config_t c0 = {};
    c0.pulse_gpio_num = pinA;
    c0.ctrl_gpio_num = pinB;
    c0.unit = unit;
    c0.channel = PCNT_CHANNEL_0;

    // Config counter mode
    c0.pos_mode = PCNT_COUNT_INC;
    c0.neg_mode = PCNT_COUNT_DIS;
    // c0.neg_mode = PCNT_COUNT_INC; // If need both edges

    // Switch control mode to match wiring***
    // c0.lctrl_mode = PCNT_MODE_REVERSE; // B=0 -> reverse
    // c0.hctrl_mode = PCNT_MODE_KEEP;    // B=1 -> keep
    c0.hctrl_mode = PCNT_MODE_REVERSE; 
    c0.lctrl_mode = PCNT_MODE_KEEP;    

    c0.counter_h_lim = PCNT_H_LIM;
    c0.counter_l_lim = PCNT_L_LIM;

    pcnt_unit_config(&c0);

    // Noise filter (adjust depending on encoder; increase if noisy, decrease if high speed misses pulses)
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    // Enable event khi chạm limit để ISR cộng dồn thành 32-bit
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    // Bật interrupt cho unit
    pcnt_intr_enable(unit);

    pcnt_counter_resume(unit);
}

void encoders_begin_v44(void)
{
    pcnt_isr_handle_t handle = NULL;
    pcnt_isr_register(pcnt_isr_handler, NULL, ESP_INTR_FLAG_LEVEL2, &handle);

    pcnt_setup_unit_quadrature(PCNT_UNIT_0, ENC0_A, ENC0_B);
    pcnt_setup_unit_quadrature(PCNT_UNIT_1, ENC1_A, ENC1_B);

    portENTER_CRITICAL(&s_mux);
    s_accum0 = 0;
    s_accum1 = 0;
    portEXIT_CRITICAL(&s_mux);
}

void encoders_reset_v44(void)
{
    static bool inited = false;
    if (!inited) {
        pcnt_isr_handle_t handle = NULL;
        pcnt_isr_register(pcnt_isr_handler, NULL, ESP_INTR_FLAG_LEVEL2, &handle);
        inited = true;
    }
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    portENTER_CRITICAL(&s_mux);
    s_accum0 = 0;
    s_accum1 = 0;
    portEXIT_CRITICAL(&s_mux);
}

int32_t encoder_get_count32_v44(pcnt_unit_t unit)
{
    static int32_t last_count0 = 0;
    static int32_t last_count1 = 0;
    int16_t now16 = 0;
    esp_err_t err = pcnt_get_counter_value(unit, &now16);
    if (err != ESP_OK) {
        return (unit == PCNT_UNIT_0) ? last_count0 : last_count1;  // Giữ cũ
    }

    portENTER_CRITICAL(&s_mux);
    int32_t base = (unit == PCNT_UNIT_0) ? s_accum0 : s_accum1;
    portEXIT_CRITICAL(&s_mux);

    int32_t current = base + (int32_t)now16;
    if (unit == PCNT_UNIT_0) last_count0 = current;
    else last_count1 = current;
    return current;
}
