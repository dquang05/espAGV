#pragma once
#ifndef UDPBEACON_H
#define UDPBEACON_H

#include <stdint.h>
#include <stddef.h>

class UdpBeacon {
public:
    UdpBeacon() : sockFd_(-1), port_(0), payloadLen_(0), lastTick_(0) {}
    
    bool begin(uint16_t port);
    void setPayload(const uint8_t* data, size_t len);
    void tick(); 
    void stop();

private:
    int sockFd_;
    uint16_t port_;
    uint8_t payload_[128];
    size_t payloadLen_;
    uint32_t lastTick_;
    const uint32_t interval_ = 2000; // 2 giây
};

#endif