#pragma once
#ifndef LIDAR_H
#define LIDAR_H
#include <RPLidar.h>
#include "pwmgen.h"
#include "thijs_rplidar.h"

typedef struct
{
    float distance;
    float angle;
    uint16_t quality;
} coordinate;

class lidar
{
public:
    void begin(void);
    void lidar_run(void);
    void lidar_stop(void);
    void speed_set(int duty, int freq);
    coordinate coordinate_take(void);
    coordinate coordinate_takev2_0(void);
    RPLidar lidar_device;
    
    pwm_gen lidar_motor;
};
coordinate simulateLidar();
extern lidar rlidar;
#endif