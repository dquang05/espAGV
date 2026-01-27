#include <Arduino.h>
#include "lidar.h"
#include "wifiBridge.h"
#include "driver/uart.h"

static const char* SSID = "TP-Link_C718";
static const char* PASS = "20017231";

LidarA1 lidar;
WifiBridgeTCP bridge;

void setup() {
  Serial.begin(115200);
  delay(800);

  // Lidar: UART2 TX=26 RX=33, PWM case 3(GPIO19), 20kHz, 90% duty cho chắc
  lidar.begin(UART_NUM_2, UART_TX_PIN, UART_RX_PIN, 115200, 3, 20000, 90.0f);
  delay(600);

  lidar.stop(); delay(100);
  // lidar.reset(); delay(300);   // bạn có thể bỏ reset nếu không cần
  lidar.start_scan();

  // WiFi TCP server on port 9000
  if (!bridge.begin(SSID, PASS, 9000)) {
    Serial.println("WiFi bridge begin FAILED");
  } else {
    Serial.printf("Connect PC to: %s:9000\n", bridge.ipString());
  }
}

void loop() {
  bridge.poll(); // accept client if any

  // UART -> TCP (raw stream)
  if (bridge.isClientConnected()) {
    static uint8_t buf[1024];
    int n = uart_read_bytes(UART_NUM_2, buf, sizeof(buf), pdMS_TO_TICKS(10));
    if (n > 0) {
      bridge.sendBytes(buf, (size_t)n);
    }

    // (Optional) PC -> UART (để gửi lệnh)
    // static uint8_t inb[256];
    // int m = bridge.recvBytes(inb, sizeof(inb));
    // if (m > 0) uart_write_bytes(UART_NUM_2, (const char*)inb, m);
  }

  vTaskDelay(pdMS_TO_TICKS(1));
}
