#include "TcpSender.h"

#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>
#include <WS2tcpip.h>

#include <chrono>
#include <iomanip>
#include <sstream>

#pragma comment(lib, "Ws2_32.lib")

namespace
{
double monotonicSeconds()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string escapeJson(const std::string& value)
{
    std::ostringstream out;
    for (char ch : value)
    {
        switch (ch)
        {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default: out << ch; break;
        }
    }
    return out.str();
}

std::string frameToJsonLine(const ManusFrame& frame)
{
    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{";
    json << "\"type\":\"manus_frame\",";
    json << "\"version\":1,";
    json << "\"seq\":" << frame.seq << ",";
    json << "\"timestamp\":" << frame.timestamp << ",";
    json << "\"dt\":" << frame.dt << ",";
    json << "\"glove_id\":" << frame.glove_id << ",";
    json << "\"side\":\"" << escapeJson(frame.side) << "\",";
    json << "\"ergonomics_names\":[";
    for (size_t i = 0; i < frame.ergonomics_names.size(); ++i)
    {
        if (i > 0) json << ",";
        json << "\"" << escapeJson(frame.ergonomics_names[i]) << "\"";
    }
    json << "],\"ergonomics_values\":[";
    for (size_t i = 0; i < frame.ergonomics_values.size(); ++i)
    {
        if (i > 0) json << ",";
        json << frame.ergonomics_values[i];
    }
    json << "]}\n";
    return json.str();
}
}

TcpSender::TcpSender() = default;

TcpSender::~TcpSender()
{
    close();
}

bool TcpSender::open(const TcpSenderConfig& config, std::string& error)
{
    close();
    enabled_ = config.enabled;
    target_ip_ = config.target_ip;
    target_port_ = config.target_port;
    reconnect_s_ = config.reconnect_s;
    if (!enabled_)
    {
        return true;
    }

    WSADATA data;
    int result = WSAStartup(MAKEWORD(2, 2), &data);
    if (result != 0)
    {
        error = "WSAStartup failed for TCP: " + std::to_string(result);
        enabled_ = false;
        return false;
    }
    opened_ = true;
    return true;
}

void TcpSender::close()
{
    if (socket_)
    {
        closesocket(reinterpret_cast<SOCKET>(socket_));
        socket_ = nullptr;
    }
    if (opened_)
    {
        WSACleanup();
    }
    opened_ = false;
}

bool TcpSender::isEnabled() const
{
    return enabled_ && opened_;
}

bool TcpSender::isConnected() const
{
    return socket_ != nullptr;
}

bool TcpSender::tryConnect(std::string& error)
{
    const double now = monotonicSeconds();
    if (now - last_connect_attempt_ < reconnect_s_)
    {
        return false;
    }
    last_connect_attempt_ = now;

    SOCKET sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET)
    {
        error = "TCP socket creation failed: " + std::to_string(WSAGetLastError());
        return false;
    }

    sockaddr_in target;
    target.sin_family = AF_INET;
    target.sin_port = htons(target_port_);
    if (InetPtonA(AF_INET, target_ip_.c_str(), &target.sin_addr) != 1)
    {
        closesocket(sock);
        error = "Invalid TCP target IP: " + target_ip_;
        return false;
    }

    if (connect(sock, reinterpret_cast<const sockaddr*>(&target), sizeof(target)) == SOCKET_ERROR)
    {
        closesocket(sock);
        error = "TCP connect waiting for receiver " + target_ip_ + ":" + std::to_string(target_port_);
        return false;
    }

    socket_ = reinterpret_cast<void*>(sock);
    return true;
}

bool TcpSender::sendFrame(const ManusFrame& frame, std::string& error)
{
    if (!isEnabled() || !frame.valid)
    {
        return true;
    }
    if (!socket_ && !tryConnect(error))
    {
        return false;
    }
    if (!socket_)
    {
        return false;
    }

    const std::string payload = frameToJsonLine(frame);
    SOCKET sock = reinterpret_cast<SOCKET>(socket_);
    int sent = send(sock, payload.c_str(), static_cast<int>(payload.size()), 0);
    if (sent == SOCKET_ERROR)
    {
        closesocket(sock);
        socket_ = nullptr;
        error = "TCP send failed; will reconnect";
        return false;
    }
    return true;
}
