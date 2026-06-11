#pragma once

#include <cstdint>
#include <string>

#include "ManusClient.h"

struct TcpSenderConfig
{
    bool enabled = true;
    std::string target_ip = "127.0.0.1";
    uint16_t target_port = 9001;
    double reconnect_s = 1.0;
    double max_hz = 120.0;
};

class TcpSender
{
public:
    TcpSender();
    ~TcpSender();

    TcpSender(const TcpSender&) = delete;
    TcpSender& operator=(const TcpSender&) = delete;

    bool open(const TcpSenderConfig& config, std::string& error);
    void close();
    bool isEnabled() const;
    bool isConnected() const;
    bool sendFrame(const ManusFrame& frame, std::string& error);

private:
    bool tryConnect(std::string& error);

    bool opened_ = false;
    bool enabled_ = false;
    void* socket_ = nullptr;
    std::string target_ip_;
    uint16_t target_port_ = 0;
    double reconnect_s_ = 1.0;
    double last_connect_attempt_ = 0.0;
};
