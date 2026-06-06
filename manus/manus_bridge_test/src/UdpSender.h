#pragma once

#include <cstdint>
#include <string>

#include "ManusClient.h"

struct UdpSenderConfig
{
    bool enabled = true;
    std::string target_ip = "127.0.0.1";
    uint16_t target_port = 9000;
    double max_hz = 120.0;
};

class UdpSender
{
public:
    UdpSender();
    ~UdpSender();

    UdpSender(const UdpSender&) = delete;
    UdpSender& operator=(const UdpSender&) = delete;

    bool open(const UdpSenderConfig& config, std::string& error);
    void close();
    bool isEnabled() const;
    bool sendFrame(const ManusFrame& frame, std::string& error);

private:
    bool opened_ = false;
    bool enabled_ = false;
    void* socket_ = nullptr;
    std::string target_ip_;
    uint16_t target_port_ = 0;
};
