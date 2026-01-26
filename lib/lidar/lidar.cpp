#include <Arduino.h>
#include <RPLidar.h>
#include "pwmgen.h"
#include "lidar.h"
#include "thijs_rplidar.h"

lidar rlidar;

void lidar::begin()
{
    Serial1.begin(256000);
    lidar_device.begin(Serial1);
    // lidar_device.begin(Serial2);
    lidar_motor.begin(3, 5000, 95);
    
  vTaskDelay(pdMS_TO_TICKS(10));
  rlidar.speed_set(80, 10000); // Set LIDAR speed and frequency
  vTaskDelay(pdMS_TO_TICKS(10));
  rlidar.speed_set(80, 50000); // Set LIDAR speed and frequency

  
}

void lidar::lidar_run(void)
{
    lidar_motor.start_pwm();
}

void lidar::lidar_stop(void)
{
    lidar_motor.stop_pwm();
}

void lidar::speed_set(int duty, int freq)
{
    lidar_motor.set_duty(duty);
    lidar_motor.set_frequency(freq);
}

coordinate lidar::coordinate_take(void)
{
    coordinate value;
    value.distance = 0;
    value.angle = 0;
    value.quality = 0;

    if (IS_OK(lidar_device.waitPoint()))
    {
        if (lidar_device.getCurrentPoint().quality  > 0 )
        {
            value.distance = lidar_device.getCurrentPoint().distance; // distance value in mm unit
            value.angle = lidar_device.getCurrentPoint().angle;       // angle value in degree
            value.quality = lidar_device.getCurrentPoint().quality;   // quality value
        }
    }
    else
    {
        this->lidar_stop();
        rplidar_response_device_info_t info;
        if (IS_OK(lidar_device.getDeviceInfo(info, 100)))
        {
            lidar_device.startScan();
            this->lidar_run();
            vTaskDelay(pdMS_TO_TICKS(10)); // Wait for LIDAR to start
        }
    }

    return value;
}

coordinate simulateLidar()
{
    static float angle = 0.0f;
    coordinate pt;

    // Giả lập góc quét từ 0 -> 360
    pt.angle = angle;
    angle += 1.0f; // mỗi lần tăng 1 độ
    if (angle >= 360.0f)
        angle = 0.0f;

    pt.distance = random(300, 2500);

    return pt;
}

coordinate lidar::coordinate_takev2_0(void)
{
   

   
}