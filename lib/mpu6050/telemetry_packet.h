#pragma once
#include <stdint.h>

#pragma pack(push,1)
struct TeleImuPktV1 {
  uint16_t magic;   // 0xA55A
  uint8_t  ver;     // 1
  uint8_t  type;    // 1=IMU
  uint32_t seq;
  uint32_t t_ms;

  int16_t roll_cdeg;
  int16_t pitch_cdeg;

  int16_t ax_mg, ay_mg, az_mg;
  int16_t gx_dps10, gy_dps10, gz_dps10;

  int16_t temp_c10;
  int16_t reserved;
};
#pragma pack(pop)

static_assert(sizeof(TeleImuPktV1) == 32, "TeleImuPktV1 must be 32 bytes");
