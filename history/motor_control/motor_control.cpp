#include <Arduino.h>
#include "motor_control.h"
#include "pwmgen.h"
#include "pid.h"
#include "esp_task_wdt.h"

pwm_gen motor1;
pwm_gen motor2;

pid_gen pid_ctr1;
pid_gen pid_ctr2;

const float ALPHA = 0.80f;

volatile bool move_done = false;
volatile bool turn_done = false;

volatile int error1 = 0;
volatile int error2 = 0;

volatile float set_speed1 = 0;
volatile float set_speed2 = 0;

volatile float cur_speed1 = 0;
volatile float cur_speed2 = 0;

float desire = 10;
float actual = 0;

volatile float output1 = 0;
volatile float output2 = 0;

volatile long prev_signal_count_0 = 0;
volatile long prev_signal_count_1 = 0;

volatile long delta_pulse1 = 0;
volatile long delta_pulse2 = 0;

const float PULSES_PER_DEGREE = 1080.0 / 360.0; // Calibration
const float DIST_PER_PULSE_MM = 700.0 / 600.0;  // Calibration

float linear_to_pulse(float v_mps)
{
    const float DIST_PER_PULSE_M = DIST_PER_PULSE_MM / 1000.0f; // convert mm → m
    return v_mps / DIST_PER_PULSE_M;                            // pulse/s
}

const float SAMP_T = 0.05; // 50ms
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void speed_control(void)
{
    portENTER_CRITICAL(&mux);
    long current_count_0 = signal_count_0;
    long current_count_1 = signal_count_1;
    portEXIT_CRITICAL(&mux);

    delta_pulse1 = current_count_0 - prev_signal_count_0;
    delta_pulse2 = current_count_1 - prev_signal_count_1;

    prev_signal_count_0 = current_count_0;
    prev_signal_count_1 = current_count_1;

    cur_speed1 = ((float)delta_pulse1) / (SAMP_T);
    cur_speed2 = ((float)delta_pulse2) / (SAMP_T);
    
    // Speed filtering
    filtered_speed1 = ALPHA * filtered_speed1 + (1 - ALPHA) * cur_speed1;
    filtered_speed2 = ALPHA * filtered_speed2 + (1 - ALPHA) * cur_speed2;

    cur_speed1 = filtered_speed1;
    cur_speed2 = filtered_speed2;

    pid_ctr1.pid_func();
    pid_ctr2.pid_func();

    motor1.set_duty(pid_ctr1.output);
    motor2.set_duty(pid_ctr2.output);

    // signal_count_0 = 0;
    // signal_count_1 = 0;
}

void direc_control(void)
{
}

void motor_begin(void)
{
    // Frequency: 500Hz, Duty: 30%
    motor1.begin(1, 500, 30);
    motor2.begin(2, 500, 30);

    motor1.capture_config(0);
    motor2.capture_config(1);

    pinMode(dir11, OUTPUT);
    pinMode(dir12, OUTPUT);
    pinMode(dir21, OUTPUT);
    pinMode(dir22, OUTPUT);

    digitalWrite(dir11, LOW);
    digitalWrite(dir12, LOW);
    digitalWrite(dir21, LOW);
    digitalWrite(dir22, LOW);

    timer_conf timer;

    // pid_ctr1.pid_begin(SAMP_T, 0.167, 10.2, 100.2, &set_speed1, &cur_speed1, 0, 100);
    // pid_ctr2.pid_begin(SAMP_T, 0.167, 10.2, 100.2, &set_speed2, &cur_speed2, 0, 100);

    // pid_ctr1.pid_begin(0.05, 0.156, 0.52, 0.26, &set_speed1, &cur_speed1,0,100);
    // pid_ctr2.pid_begin(0.05, 0.156, 0.52, 0.26, &set_speed2, &cur_speed2,0,100);

    pid_ctr1.pid_begin(0.05, 0.125, 0.52, 0.05, &set_speed1, &cur_speed1,0,100);
    pid_ctr2.pid_begin(0.05, 0.125, 0.52, 0.05, &set_speed2, &cur_speed2,0,100);

    FuncPtr fn = speed_control;

    pid_timer_set(timer, fn, SAMP_T);

    delay(100);
}

void motor_run(bool en)
{
    if (en)
    {
        // Run
        pid_start();

        motor1.start_pwm();
        motor2.start_pwm();
    }
    else
    {
        // Stop
        digitalWrite(dir11, LOW);
        digitalWrite(dir12, LOW);
        digitalWrite(dir21, LOW);
        digitalWrite(dir22, LOW);

        pid_stop();

        motor1.stop_pwm();
        motor2.stop_pwm();

        pid_ctr1._conf.error = 0;
        pid_ctr1._conf.error1 = 0;
        pid_ctr1._conf.error2 = 0;

        pid_ctr2._conf.error = 0;
        pid_ctr2._conf.error1 = 0;
        pid_ctr2._conf.error2 = 0;
    }
}

void motor_speed_set(float v, float w)
{
    set_speed1 = linear_to_pulse(((2 * v + w * wheel_distance) / 2));
    set_speed2 = linear_to_pulse(((2 * v - w * wheel_distance) / 2));
    if (set_speed1 == 0)
    {
        digitalWrite(dir11, LOW);
        digitalWrite(dir12, LOW);
    }
    else
    {
        if (set_speed1 > 0)
        {
            digitalWrite(dir11, HIGH);
            digitalWrite(dir12, LOW);
        }
        else
        {
            digitalWrite(dir11, LOW);
            digitalWrite(dir12, HIGH);
            set_speed1 = -set_speed1;
        }
    }

    if (set_speed2 == 0)
    {
        digitalWrite(dir21, LOW);
        digitalWrite(dir22, LOW);
    }
    else
    {
        if (set_speed2 > 0)
        {
            digitalWrite(dir21, HIGH);
            digitalWrite(dir22, LOW);
        }
        else
        {
            digitalWrite(dir21, LOW);
            digitalWrite(dir22, HIGH);
            set_speed2 = -set_speed2;
        }
    }
}

void move_straight(float distance_mm, float v_mps)
{
    if (distance_mm == 0)
        return;

    // Reset bộ đếm encoder
    portENTER_CRITICAL(&mux);
    total_count_0 = 0;
    total_count_1 = 0;
    portEXIT_CRITICAL(&mux);

    // Cài đặt tốc độ thẳng
    float v = v_mps; // m/s
    float w = 0.0f;  // quay = 0
    motor_speed_set(v, w);

    // Tính số xung cần đạt
    const long TARGET_PULSES = distance_mm / DIST_PER_PULSE_MM;

    while (1)
    {
        uint32_t count1, count2;
        portENTER_CRITICAL(&mux);
        count1 = total_count_0;
        count2 = total_count_1;
        portEXIT_CRITICAL(&mux);

        float avg_count = (count1 + count2) / 2.0f;
        float traveled_mm = avg_count * DIST_PER_PULSE_MM;

        if (traveled_mm >= distance_mm)  break;
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10)); // tránh chiếm CPU
    }

    motor_speed_set(0, 0); // dừng
    move_done = true;
    Serial.println("Move done");
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ----------------------------------------------------
// Quay một góc angle_deg (độ) với tốc độ góc w_rad (rad/s)
// ----------------------------------------------------
void turn_angle(float angle_deg, float w_rad)
{
    if (angle_deg == 0)
        return;

    // Reset bộ đếm encoder
    portENTER_CRITICAL(&mux);
    total_count_0 = 0;
    total_count_1 = 0;
    portEXIT_CRITICAL(&mux);

    // Cài đặt quay tại chỗ
    float v = 0.0f;
    float w = (angle_deg > 0) ? w_rad : -w_rad;
    motor_speed_set(v, w);

    // Tính quãng đường bánh xe cần đi để quay đúng góc

    const long TURN_PULSES = abs(angle_deg) * PULSES_PER_DEGREE;

    while (1)
    {
        uint32_t count1, count2;
        portENTER_CRITICAL(&mux);
        count1 = total_count_0;
        count2 = total_count_1;
        portEXIT_CRITICAL(&mux);
        float diff_pulse = fabs((int32_t)count1 + (int32_t)count2);
        // if (diff_pulse >= TURN_PULSES)
        // {
        //     break;
        // }
        if (fabs(count1) >= TURN_PULSES && fabs(count2) >= TURN_PULSES) break;
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    motor_speed_set(0, 0); // dừng
    turn_done = true;
    Serial.println("Turn done");
    vTaskDelay(pdMS_TO_TICKS(100));
}