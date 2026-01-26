#pragma once
#include <Arduino.h>
#include "thijs_rplidar.h"
#include "pwmgen.h"

struct LidarPoint {
    float distance;   // mm
    float angleDeg;   // ° degrees
};

class LidarHandler {
public:
    LidarHandler();          // Constructor
    bool begin(uint8_t rxPin, uint8_t txPin, uint8_t pwmPin);
    void run();              // handleData loop
    bool coordinate_take(LidarPoint &outPoint); // return angle + distance

private:
    static void dataHandlerTrampoline(RPlidar* lidarPtr, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality);
    void dataHandler(uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality);

    RPlidar lidar;
    pwm_gen motorPWM;
    
    volatile float lastAngle = -1;
    volatile float lastDistance = -1;
};
