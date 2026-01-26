#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "pid.h"
#include "pwmgen.h"
#define dir11 23
#define dir12 5
#define dir21 2
#define dir22 4
#define wheel_distance 1.0f

extern pid_gen pid_ctr1;
extern pid_gen pid_ctr2;

static float filtered_speed1 = 0;
static float filtered_speed2 = 0;

extern volatile bool move_done;
extern volatile bool turn_done;

extern volatile int error1;
extern volatile int error2;

extern volatile float cur_speed1;
extern volatile float cur_speed2;

extern volatile float output1;
extern volatile float output2;

extern volatile long prev_signal_count_0;
extern volatile long prev_signal_count_1;

extern volatile long delta_pulse1;
extern volatile long delta_pulse2;

extern volatile float set_speed1;
extern volatile float set_speed2;

void motor_begin(void);
void motor_run(bool en);
void motor_speed_set(float v, float w);
void speed_control(void);
void move_straight(float distance_mm, float speed_mm_s);
void turn_angle(float angle_deg, float angular_speed_deg_s);

#endif