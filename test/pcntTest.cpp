#include <Arduino.h>
#include "myPcnt.h"
#include "driver/pcnt.h"

static int32_t last0 = 0;
static int32_t last1 = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== PCNT ONLY TEST (ESP-IDF v4.4) ===");
  Serial.println("Rotate encoder by hand. Count should change (+/-).");

  encoders_begin_v44();

  // First read
  last0 = encoder_get_count32_v44(PCNT_UNIT_0);
  last1 = encoder_get_count32_v44(PCNT_UNIT_1);

  Serial.printf("Init count0=%ld, count1=%ld\n", (long)last0, (long)last1);
}

void loop() {
  int32_t c0 = encoder_get_count32_v44(PCNT_UNIT_0);
  int32_t c1 = encoder_get_count32_v44(PCNT_UNIT_1);

  int32_t d0 = c0 - last0;
  int32_t d1 = c1 - last1;

  last0 = c0;
  last1 = c1;

  // Print both total and delta counts to see how much the count changes with each rotation
  Serial.printf("U0: %ld (d=%ld) | U1: %ld (d=%ld)\n",
                (long)c0, (long)d0, (long)c1, (long)d1);

  delay(100);
}
