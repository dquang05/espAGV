#include <Arduino.h>

#define HC12_RX 21   // ESP32 RX ← Arduino TX
#define HC12_TX 22   // ESP32 TX → Arduino RX

HardwareSerial HC12(1);  // Use UART1

void setup() {
  Serial.begin(115200);                              // For debug
  HC12.begin(9600, SERIAL_8N1, HC12_RX, HC12_TX);    // Match baud rate with Nano
  Serial.println("ESP32 UART Receiver Started");
}

void loop() {
  static uint8_t buffer[4];
  static uint8_t index = 0;

  while (HC12.available()) {
    uint8_t byteIn = HC12.read();

    if (index == 0 && byteIn != 0xAA) {
      // Wait until start byte (0xAA)
      continue;
    }

    buffer[index++] = byteIn;

    if (index == 4) {
      // Full 4-byte packet received
      if (buffer[0] == 0xAA && buffer[3] == 0x55) {
        Serial.print("Received packet: ");
        for (int i = 0; i < 4; i++) {
          Serial.print(buffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        // Extract v and w
        uint8_t v = buffer[1];
        uint8_t w = buffer[2];
        Serial.print("v = ");
        Serial.print(v);
        Serial.print(", w = ");
        Serial.println(w);
      } else {
        Serial.println("Invalid packet");
      }

      index = 0; // Reset for next packet
    }
  }
}
