#include <Arduino.h>
#include "lidar.h"

#include "driver/uart.h"
#include "esp_timer.h"

LidarA1 lidar;

static void dump_hex(const uint8_t* b, int n) {
  for (int i = 0; i < n; i++) {
    Serial.printf("%02X ", b[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n=== LIDAR RAW TEST (Arduino entry) ===");

  // 1) Init UART + start motor PWM (case 3 = GPIO19) inside lidar.begin()
  esp_err_t err = lidar.begin(
      UART_NUM_2,
      GPIO_NUM_26,  // ESP TX -> Lidar RX
      GPIO_NUM_33,  // ESP RX <- Lidar TX
      115200,
      3,            // PWM case 3 -> GPIO19
      20000,        // 20 kHz
      90.0f         // duty %
  );
  Serial.printf("lidar.begin() = %d\n", (int)err);

  delay(600); // đợi motor ổn định

  // 2) Send minimal commands
  lidar.stop();  delay(100);
  //lidar.reset(); delay(300);
  lidar.get_info();  delay(200);
  lidar.get_health();delay(200);
  lidar.start_scan();
  Serial.println("Sent: STOP, RESET, START_SCAN");

  // 3) VERIFY: read bytes for 1.5s, dump first 32 bytes one time
  uint8_t buf[256];
  int total = 0;
  bool dumped = false;

  int64_t t_end = esp_timer_get_time() + 1500LL * 1000LL;

  while (esp_timer_get_time() < t_end) {
    int n = uart_read_bytes(UART_NUM_2, buf, sizeof(buf), pdMS_TO_TICKS(50));
    if (n > 0) {
      total += n;

      if (!dumped) {
        Serial.printf("First chunk (%d bytes), first 32 bytes:\n", n);
        dump_hex(buf, (n > 32) ? 32 : n);
        dumped = true;
      }
    }
  }

  Serial.printf("Total received in 1.5s: %d bytes\n", total);
  Serial.println("Now streaming: will print bytes/sec...\n");
}

void loop() {
  // Print bytes received per 1 second (đỡ spam log, tránh rớt byte)
  static uint32_t last_ms = 0;
  static int acc = 0;

  uint8_t buf[512];
  int n = uart_read_bytes(UART_NUM_2, buf, sizeof(buf), pdMS_TO_TICKS(20));
  if (n > 0) acc += n;

  uint32_t now = millis();
  if (now - last_ms >= 1000) {
    Serial.printf("RX rate: %d bytes/s\n", acc);
    acc = 0;
    last_ms = now;
  }
}

