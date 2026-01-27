#include <Arduino.h>
#include "motor_control.h"
#include "pwmgen.h"
#include "pid.h"
#include "esp_task_wdt.h"
#include "myPcnt.h"

pwm_gen motor1;
pwm_gen motor2;

pid_gen pid_ctr1;
pid_gen pid_ctr2;

const float ALPHA = 0.80f;

float filtered_speed1 = 0;
float filtered_speed2 = 0;

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

volatile int32_t prev_cnt0 = 0;
volatile int32_t prev_cnt1 = 0;

volatile long delta_pulse1 = 0;
volatile long delta_pulse2 = 0;

const float PULSES_PER_DEGREE = 1080.0 / 360.0; // Calibration
const float DIST_PER_PULSE_MM = 700.0 / 600.0;  // Calibration

static int s_prev_sign1 = 0;
static int s_prev_sign2 = 0;

static inline int sgnf(float x) { return (x > 0) - (x < 0); }

static inline float linear_mmps_to_pulse(float v_mm_s)
{
    // DIST_PER_PULSE_MM = mm/pulse  => pulse/s = (mm/s) / (mm/pulse)
    return v_mm_s / DIST_PER_PULSE_MM;
}

const float SAMP_T = 0.05; // 50ms
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void speed_control(void)
{
    int32_t cnt0 = encoder_get_count32_v44(PCNT_UNIT_0);
    int32_t cnt1 = encoder_get_count32_v44(PCNT_UNIT_1);

    delta_pulse1 = cnt0 - prev_cnt0;
    delta_pulse2 = cnt1 - prev_cnt1;

    prev_cnt0 = cnt0;
    prev_cnt1 = cnt1;

    cur_speed1 = (float)delta_pulse1 / SAMP_T;
    cur_speed2 = (float)delta_pulse2 / SAMP_T;

    filtered_speed1 = ALPHA * filtered_speed1 + (1 - ALPHA) * cur_speed1;
    filtered_speed2 = ALPHA * filtered_speed2 + (1 - ALPHA) * cur_speed2;
    cur_speed1 = filtered_speed1;
    cur_speed2 = filtered_speed2;

    pid_ctr1.pid_func();
    pid_ctr2.pid_func();

    motor1.set_duty(pid_ctr1.output);
    motor2.set_duty(pid_ctr2.output);
}

void direc_control(void)
{
}

void motor_begin(void)
{
    // Frequency: 20000Hz
    motor1.begin(1, 20000, 0);
    motor2.begin(2, 20000, 0);

    encoders_begin_v44();

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

    pid_ctr1.pid_begin(0.05, 0.125, 0.52, 0.05, &set_speed1, &cur_speed1,0,80);
    pid_ctr2.pid_begin(0.05, 0.125, 0.52, 0.05, &set_speed2, &cur_speed2,0,80);

    FuncPtr fn = pid_control_tick_isr;
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

void motor_speed_set(float v_mm_s, float w_deg_s)
{
    // Deadzone filter
    const float V_EPS = 0.001f;
    const float W_EPS = 0.001f;

    if (fabsf(v_mm_s) < V_EPS) v_mm_s = 0.0f;
    if (fabsf(w_deg_s) < W_EPS) w_deg_s = 0.0f;

    // deg/s -> rad/s
    const float w_rad_s = w_deg_s * (PI / 180.0f);
    const float halfL_mm = wheel_distance_mm * 0.5f;

    // v_wheel = v ± w*(L/2)  (mm/s)
    const float v1_mm_s = v_mm_s + w_rad_s * halfL_mm; // wheel 1
    const float v2_mm_s = v_mm_s - w_rad_s * halfL_mm; // wheel 2

    // setpoint PID
    set_speed1 = linear_mmps_to_pulse(v1_mm_s);
    set_speed2 = linear_mmps_to_pulse(v2_mm_s);

    // Change direction handling
    int8_t s1 = sgnf(set_speed1);
    int8_t s2 = sgnf(set_speed2);

    // Motor 1
    if (s1 != 0 && s_prev_sign1 != 0 && s1 != s_prev_sign1) {
        motor1.set_duty(0);
        // coast (IN1=IN2=0)  
        digitalWrite(dir11, LOW);
        digitalWrite(dir12, LOW);

        // reset PID state + filter to prevent "burn-in"
        pid_ctr1._conf.error  = 0;
        pid_ctr1._conf.error1 = 0;
        pid_ctr1._conf.error2 = 0;
        pid_ctr1.output = 0;

        filtered_speed1 = 0;
        cur_speed1 = 0;

        // sync prev counter with current
        portENTER_CRITICAL(&mux);
        prev_cnt0 = encoder_get_count32_v44(PCNT_UNIT_0);
        delta_pulse1 = 0;
        portEXIT_CRITICAL(&mux);

        vTaskDelay(pdMS_TO_TICKS(50)); // dead-time 50ms
    }

    // Motor 2
    if (s2 != 0 && s_prev_sign2 != 0 && s2 != s_prev_sign2) {
        motor2.set_duty(0);
        digitalWrite(dir21, LOW);
        digitalWrite(dir22, LOW);

        pid_ctr2._conf.error  = 0;
        pid_ctr2._conf.error1 = 0;
        pid_ctr2._conf.error2 = 0;
        pid_ctr2.output = 0;

        filtered_speed2 = 0;
        cur_speed2 = 0;

        portENTER_CRITICAL(&mux);
        prev_cnt1 = encoder_get_count32_v44(PCNT_UNIT_1);
        delta_pulse2 = 0;
        portEXIT_CRITICAL(&mux);

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // set DIR pins
    if (s1 == 0) {
        digitalWrite(dir11, LOW);
        digitalWrite(dir12, LOW);
    } else if (s1 > 0) {
        digitalWrite(dir11, HIGH);
        digitalWrite(dir12, LOW);
    } else {
        digitalWrite(dir11, LOW);
        digitalWrite(dir12, HIGH);
    }

    if (s2 == 0) {
        digitalWrite(dir21, LOW);
        digitalWrite(dir22, LOW);
    } else if (s2 > 0) {
        digitalWrite(dir21, HIGH);
        digitalWrite(dir22, LOW);
    } else {
        digitalWrite(dir21, LOW);
        digitalWrite(dir22, HIGH);
    }

    s_prev_sign1 = s1;
    s_prev_sign2 = s2;
}

static inline bool reached_signed(float now, float target, float tol)
{
    return (target >= 0.0f) ? (now >= target - tol) : (now <= target + tol);
}

void drive_motion(float dist_mm, float angle_deg, float v_mm_s, float w_deg_s)
{
    // Tolerance
    const float DIST_TOL_MM = 2.0f;   // Distance tolerance in mm
    const float ANG_TOL_DEG = 2.0f;   // Angle tolerance in degrees

    if (dist_mm == 0.0f && angle_deg == 0.0f) return;

    // Reset progress 
    pid_stop();
    motor1.set_duty(0);
    motor2.set_duty(0);

    encoders_reset_v44();

    portENTER_CRITICAL(&mux);
    prev_cnt0 = 0;
    prev_cnt1 = 0;
    delta_pulse1 = 0;
    delta_pulse2 = 0;
    filtered_speed1 = 0;
    filtered_speed2 = 0;
    cur_speed1 = 0;
    cur_speed2 = 0;
    portEXIT_CRITICAL(&mux);

    pid_ctr1._conf.error = pid_ctr1._conf.error1 = pid_ctr1._conf.error2 = 0;
    pid_ctr2._conf.error = pid_ctr2._conf.error1 = pid_ctr2._conf.error2 = 0;
    pid_ctr1.output = 0;
    pid_ctr2.output = 0;

    pid_start();
    motor_speed_set(v_mm_s, w_deg_s);

    // Initial encoder counts
    const int32_t c0_0 = encoder_get_count32_v44(PCNT_UNIT_0);
    const int32_t c1_0 = encoder_get_count32_v44(PCNT_UNIT_1);

    while (1)
    {
        const int32_t c0 = encoder_get_count32_v44(PCNT_UNIT_0) - c0_0;
        const int32_t c1 = encoder_get_count32_v44(PCNT_UNIT_1) - c1_0;

        // Distance traveled by each wheel
        const float dL_mm = (float)c0 * DIST_PER_PULSE_MM;
        const float dR_mm = (float)c1 * DIST_PER_PULSE_MM;

        // Calculate traveled distance and angle
        const float s_mm = 0.5f * (dL_mm + dR_mm);
        const float theta_deg = ((dR_mm - dL_mm) / wheel_distance_mm) * (180.0f / PI);

        // Check done
        bool dist_done = (fabsf(dist_mm) <= DIST_TOL_MM) ? true : reached_signed(s_mm, dist_mm, DIST_TOL_MM);
        bool ang_done  = (fabsf(angle_deg) <= ANG_TOL_DEG) ? true : reached_signed(theta_deg, angle_deg, ANG_TOL_DEG);

        if (dist_done && ang_done) break;

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    motor_speed_set(0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}



// void move_straight(float distance_mm, float speed_mm_s)
// {
//     if (distance_mm == 0) return;

//     // Reset encoder
//     encoders_reset_v44();
//     prev_cnt0 = 0;
//     prev_cnt1 = 0;

//     // Go straight backward
//     float v = (distance_mm > 0) ? fabsf(speed_mm_s) : -fabsf(speed_mm_s);
//     motor_speed_set(v, 0.0f);

//     const float target_pulses = fabsf(distance_mm) / DIST_PER_PULSE_MM;

//     while (1) {
//         int32_t c0 = encoder_get_count32_v44(PCNT_UNIT_0);
//         int32_t c1 = encoder_get_count32_v44(PCNT_UNIT_1);

//         float avg = 0.5f * (fabsf((float)c0) + fabsf((float)c1));
//         if (avg >= target_pulses) break;

//         esp_task_wdt_reset();
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }

//     motor_speed_set(0, 0);
//     move_done = true;
//     Serial.println("Move done");
//     vTaskDelay(pdMS_TO_TICKS(100));
// }


// // ----------------------------------------------------
// // Quay một góc angle_deg (độ) với tốc độ góc w_rad (rad/s)
// // ----------------------------------------------------
// void turn_angle(float angle_deg, float angular_speed_deg_s)
// {
//     if (angle_deg == 0) return;

//     encoders_reset_v44();
//     prev_cnt0 = 0;
//     prev_cnt1 = 0;

//     // quay tại chỗ: v=0, w theo deg/s (dấu theo angle)
//     float w = (angle_deg > 0) ? fabsf(angular_speed_deg_s) : -fabsf(angular_speed_deg_s);
//     motor_speed_set(0.0f, w);

//     const float turn_pulses = fabsf(angle_deg) * PULSES_PER_DEGREE;

//     while (1) {
//         int32_t c0 = encoder_get_count32_v44(PCNT_UNIT_0);
//         int32_t c1 = encoder_get_count32_v44(PCNT_UNIT_1);

//         if (fabsf((float)c0) >= turn_pulses && fabsf((float)c1) >= turn_pulses) break;

//         esp_task_wdt_reset();
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }

//     motor_speed_set(0, 0);
//     turn_done = true;
//     Serial.println("Turn done");
//     vTaskDelay(pdMS_TO_TICKS(100));
// }