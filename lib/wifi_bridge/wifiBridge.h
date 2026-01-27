#pragma once
#ifndef WIFI_BRIDGE_H
#define WIFI_BRIDGE_H

#include <stdint.h>
#include <stddef.h>

class WifiBridgeTCP {
public:
  WifiBridgeTCP() = default;

  // Start WiFi STA + TCP server
  // port ví dụ 9000
  bool begin(const char* ssid, const char* pass, uint16_t port);

  // Call repeatedly in loop() to accept client / maintain connection
  void poll();

  bool isClientConnected() const;

  // Send raw bytes to PC (returns bytes sent, -1 if no client)
  int sendBytes(const uint8_t* data, size_t len);

  // Receive bytes from PC (returns bytes received, 0 if none, -1 if disconnected/no client)
  int recvBytes(uint8_t* out, size_t maxLen);

  // Close client socket
  void disconnectClient();

  // Get ESP IP as String (Arduino)
  const char* ipString() const;

private:
  uint16_t port_ = 0;
  int serverFd_ = -1;
  int clientFd_ = -1;
  char ipStr_[16] = {0};

  bool startServer_();
  void closeFd_(int& fd);
};

#endif
