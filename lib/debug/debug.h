#pragma once
#include <Arduino.h>

#ifndef DEBUG_ENABLE
#define DEBUG_ENABLE 1
#endif

#if DEBUG_ENABLE
#define DBG_PRINT(...) Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)

extern int imuCount;
extern int lidarCount;
extern int ctrCount;

#define DEBUG_IMU_INC() imuCount++;
#define DEBUG_LIDAR_INC() lidarCount++;
#define DEBUG_CTR_INC() ctrCount++;

#else
#define DBG_PRINT(...) \
  do                   \
  {                    \
  } while (0)
#define DBG_PRINTLN(...) \
  do                     \
  {                      \
  } while (0)
#define DBG_PRINTF(...) \
  do                    \
  {                     \
  } while (0)
#define DEBUG_IMU_INC()   
#define DEBUG_LIDAR_INC()
#define DEBUG_CTR_INC()
#endif