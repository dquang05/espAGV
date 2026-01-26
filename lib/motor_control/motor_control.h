#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "pid.h"
#include "pwmgen.h"
#define dir11 23
#define dir12 18
#define dir21 13
#define dir22 16
#define wheel_distance_mm 1.0f

extern pid_gen pid_ctr1;
extern pid_gen pid_ctr2;

extern float filtered_speed1;
extern float filtered_speed2;

extern volatile bool move_done;
extern volatile bool turn_done;

extern volatile int error1;
extern volatile int error2;

extern volatile float cur_speed1;
extern volatile float cur_speed2;

extern volatile float output1;
extern volatile float output2;

extern volatile int32_t prev_cnt0;
extern volatile int32_t prev_cnt1;

extern volatile long delta_pulse1;
extern volatile long delta_pulse2;

extern volatile float set_speed1;
extern volatile float set_speed2;

void motor_begin(void);
void motor_run(bool en);
void speed_control(void);
void motor_speed_set(float v_mm_s, float w_deg_s);
void move_straight(float distance_mm, float speed_mm_s);
void turn_angle(float angle_deg, float angular_speed_deg_s);
void drive_motion(float dist_mm, float angle_deg,
                  float v_mm_s, float w_deg_s);


#endif