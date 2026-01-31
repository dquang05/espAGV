#include <Arduino.h>
#include "UdpBeacon.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/def.h"
#include <WiFi.h>

#include <string.h>

#ifdef INADDR_NONE
#undef INADDR_NONE
#endif

bool UdpBeacon::begin(uint16_t port)
{
    port_ = port;

    sockFd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockFd_ < 0)
        return false;

    int broadcastEnable = 1;
    setsockopt(sockFd_, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));

    return true;
}

void UdpBeacon::setPayload(const uint8_t *data, size_t len)
{
    payloadLen_ = (len > sizeof(payload_)) ? sizeof(payload_) : len;
    memcpy(payload_, data, payloadLen_);
}

void UdpBeacon::tick()
{
    if (sockFd_ < 0)
        return;

    uint32_t now = millis();
    if (now - lastTick_ >= interval_)
    {
        lastTick_ = now;

        sockaddr_in destAddr{};
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(port_);
        IPAddress ip = WiFi.localIP();
        IPAddress mask = WiFi.subnetMask();
        IPAddress bcast(
            (uint8_t)(ip[0] | ~mask[0]),
            (uint8_t)(ip[1] | ~mask[1]),
            (uint8_t)(ip[2] | ~mask[2]),
            (uint8_t)(ip[3] | ~mask[3]));

        // Chuyển IPAddress -> uint32_t (Arduino ESP32 core làm đúng endian cho lwIP)
        destAddr.sin_addr.s_addr = (uint32_t)bcast;

        sendto(sockFd_, payload_, payloadLen_, 0, (sockaddr *)&destAddr, sizeof(destAddr));
    }
}

void UdpBeacon::stop()
{
    if (sockFd_ >= 0)
    {
        close(sockFd_);
        sockFd_ = -1;
    }
}