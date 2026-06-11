#include "UdpSender.h"

#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>
#include <WS2tcpip.h>

#include <iomanip>
#include <sstream>

#pragma comment(lib, "Ws2_32.lib")

namespace
{
std::string escapeJson(const std::string& value)
{
    std::ostringstream out;
    for (char ch : value)
    {
        switch (ch)
        {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\b': out << "\\b"; break;
        case '\f': out << "\\f"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (static_cast<unsigned char>(ch) < 0x20)
            {
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<int>(static_cast<unsigned char>(ch));
            }
            else
            {
                out << ch;
            }
            break;
        }
    }
    return out.str();
}

std::string frameToJson(const ManusFrame& frame)
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
    json << "],";

    json << "\"ergonomics_values\":[";
    for (size_t i = 0; i < frame.ergonomics_values.size(); ++i)
    {
        if (i > 0) json << ",";
        json << frame.ergonomics_values[i];
    }
    json << "]";
    json << "}";
    return json.str();
}
}

UdpSender::UdpSender() = default;

UdpSender::~UdpSender()
{
    close();
}

bool UdpSender::open(const UdpSenderConfig& config, std::string& error)
{
    close();
    enabled_ = config.enabled;
    target_ip_ = config.target_ip;
    target_port_ = config.target_port;

    if (!enabled_)
    {
        return true;
    }

    WSADATA data;
    int result = WSAStartup(MAKEWORD(2, 2), &data);
    if (result != 0)
    {
        error = "WSAStartup failed: " + std::to_string(result);
        enabled_ = false;
        return false;
    }

    SOCKET sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET)
    {
        error = "UDP socket creation failed: " + std::to_string(WSAGetLastError());
        WSACleanup();
        enabled_ = false;
        return false;
    }
    const int send_buffer_size = 1 << 20;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, reinterpret_cast<const char*>(&send_buffer_size), sizeof(send_buffer_size));

    socket_ = reinterpret_cast<void*>(sock);
    opened_ = true;
    return true;
}

void UdpSender::close()
{
    if (opened_ && socket_)
    {
        SOCKET sock = reinterpret_cast<SOCKET>(socket_);
        closesocket(sock);
        WSACleanup();
    }
    socket_ = nullptr;
    opened_ = false;
}

bool UdpSender::isEnabled() const
{
    return enabled_ && opened_;
}

bool UdpSender::sendFrame(const ManusFrame& frame, std::string& error)
{
    if (!isEnabled() || !frame.valid)
    {
        return true;
    }

    sockaddr_in target;
    target.sin_family = AF_INET;
    target.sin_port = htons(target_port_);
    if (InetPtonA(AF_INET, target_ip_.c_str(), &target.sin_addr) != 1)
    {
        error = "Invalid UDP target IP: " + target_ip_;
        return false;
    }

    const std::string payload = frameToJson(frame);
    SOCKET sock = reinterpret_cast<SOCKET>(socket_);
    const int sent = sendto(
        sock,
        payload.c_str(),
        static_cast<int>(payload.size()),
        0,
        reinterpret_cast<const sockaddr*>(&target),
        sizeof(target)
    );
    if (sent == SOCKET_ERROR)
    {
        error = "UDP send failed: " + std::to_string(WSAGetLastError());
        return false;
    }
    return true;
}
