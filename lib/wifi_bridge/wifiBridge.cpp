#include "wifiBridge.h"
#include <Arduino.h>
#include <WiFi.h>

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

static const char* TAG = "WIFI_BRIDGE";

bool WifiBridgeTCP::begin(const char* ssid, const char* pass, uint16_t port) {
  port_ = port;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                 // giảm jitter
  WiFi.begin(ssid, pass);

  Serial.printf("[WIFI] Connecting to %s", ssid);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - t0 > 15000) {
      Serial.println("\n[WIFI] Connect timeout");
      return false;
    }
  }
  Serial.println("\n[WIFI] Connected!");
  IPAddress ip = WiFi.localIP();
  snprintf(ipStr_, sizeof(ipStr_), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  Serial.printf("[WIFI] IP: %s\n", ipStr_);

  return startServer_();
}

bool WifiBridgeTCP::startServer_() {
  // create server socket
  serverFd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (serverFd_ < 0) {
    Serial.println("[WIFI] socket() failed");
    return false;
  }

  int yes = 1;
  setsockopt(serverFd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(serverFd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
    Serial.println("[WIFI] bind() failed");
    closeFd_(serverFd_);
    return false;
  }

  if (listen(serverFd_, 1) < 0) {
    Serial.println("[WIFI] listen() failed");
    closeFd_(serverFd_);
    return false;
  }

  // non-block accept
  int flags = fcntl(serverFd_, F_GETFL, 0);
  fcntl(serverFd_, F_SETFL, flags | O_NONBLOCK);

  Serial.printf("[WIFI] TCP server listening on %u\n", port_);
  return true;
}

void WifiBridgeTCP::poll() {
  // if client already connected, nothing to do
  if (clientFd_ >= 0) return;

  if (serverFd_ < 0) return;

  sockaddr_in6 clientAddr{};
  socklen_t len = sizeof(clientAddr);
  int c = accept(serverFd_, (sockaddr*)&clientAddr, &len);
  if (c >= 0) {
    clientFd_ = c;

    // optional: non-block client recv
    // int flags = fcntl(clientFd_, F_GETFL, 0);
    // fcntl(clientFd_, F_SETFL, flags | O_NONBLOCK);

    // reduce latency
    int one = 1;
    setsockopt(clientFd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    Serial.println("[WIFI] Client connected");
  }
}

bool WifiBridgeTCP::isClientConnected() const {
  return clientFd_ >= 0;
}

int WifiBridgeTCP::sendBytes(const uint8_t* data, size_t len) {
  if (clientFd_ < 0) return -1;
  if (!data || len == 0) return 0;

  int sent = send(clientFd_, data, (int)len, 0);
  if (sent < 0) {
    // disconnected
    disconnectClient();
    return -1;
  }
  return sent;
}

int WifiBridgeTCP::recvBytes(uint8_t* out, size_t maxLen) {
  if (clientFd_ < 0) return -1;
  if (!out || maxLen == 0) return 0;

  int n = recv(clientFd_, out, (int)maxLen, 0);
  if (n > 0) return n;
  if (n == 0) { // graceful close
    disconnectClient();
    return -1;
  }
  // n < 0: EWOULDBLOCK is normal for non-block
  return 0;
}

void WifiBridgeTCP::disconnectClient() {
  if (clientFd_ >= 0) {
    Serial.println("[WIFI] Client disconnected");
  }
  closeFd_(clientFd_);
}

void WifiBridgeTCP::closeFd_(int& fd) {
  if (fd >= 0) {
    close(fd);
    fd = -1;
  }
}

const char* WifiBridgeTCP::ipString() const {
  return ipStr_;
}
