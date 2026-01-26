#pragma once
#include <stdint.h>
#include "driver/pcnt.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

void encoders_begin_v44(void);
void encoders_reset_v44(void);

int32_t encoder_get_count32_v44(pcnt_unit_t unit);

#ifdef __cplusplus
}
#endif
