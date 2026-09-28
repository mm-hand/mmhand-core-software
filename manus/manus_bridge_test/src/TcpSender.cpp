#include "TcpSender.h"

#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>
#include <WS2tcpip.h>

#include <chrono>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <vector>

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

std::string chainTypeName(int chain_type)
{
    switch (chain_type)
    {
    case 13: return "hand";
    case 5:  return "thumb";
    case 6:  return "index";
    case 7:  return "middle";
    case 8:  return "ring";
    case 9:  return "pinky";
    default: return "unknown";
    }
}

std::string jointTypeName(int joint_type)
{
    switch (joint_type)
    {
    case 1: return "metacarpal";
    case 2: return "proximal";
    case 3: return "intermediate";
    case 4: return "distal";
    case 5: return "tip";
    default: return "unknown";
    }
}

void writeNodeJson(
    std::ostringstream& out,
    const RawSkeletonNodeSample& node)
{
    out << "{";
    out << "\"node_id\":" << node.node_id << ",";
    out << "\"parent_id\":" << node.parent_id << ",";
    out << "\"pos\":[" << node.position.x << "," << node.position.y << "," << node.position.z << "],";
    out << "\"rot\":[" << node.rotation.w << "," << node.rotation.x << "," << node.rotation.y << "," << node.rotation.z << "]";
    out << "}";
}

void writeRawSkeletonNodesJson(
    std::ostringstream& out,
    const std::vector<RawSkeletonNodeSample>& nodes)
{
    out << "\"raw_skeleton_nodes\":[";
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        if (i) out << ",";
        const auto& node = nodes[i];
        out << "{";
        out << "\"node_id\":" << node.node_id << ",";
        out << "\"parent_id\":" << node.parent_id << ",";
        out << "\"chain_type\":" << node.chain_type << ",";
        out << "\"finger_joint_type\":" << node.finger_joint_type << ",";
        out << "\"chain\":\"" << chainTypeName(node.chain_type) << "\",";
        out << "\"joint\":\"" << jointTypeName(node.finger_joint_type) << "\",";
        out << "\"pos\":[" << node.position.x << "," << node.position.y << "," << node.position.z << "],";
        out << "\"rot\":[" << node.rotation.w << "," << node.rotation.x << "," << node.rotation.y << "," << node.rotation.z << "]";
        out << "}";
    }
    out << "]";
}

void writeSkeletonDictJson(
    std::ostringstream& out,
    const std::vector<RawSkeletonNodeSample>& nodes)
{
    std::map<std::string, std::map<std::string, RawSkeletonNodeSample>> skeleton_map;
    std::vector<std::string> chain_order;

    for (const auto& node : nodes)
    {
        const std::string chain = chainTypeName(node.chain_type);
        const std::string joint = jointTypeName(node.finger_joint_type);

        auto chain_it = skeleton_map.find(chain);
        if (chain_it == skeleton_map.end())
        {
            skeleton_map[chain] = {};
            chain_order.push_back(chain);
        }
        if (skeleton_map[chain].find(joint) == skeleton_map[chain].end())
        {
            skeleton_map[chain][joint] = node;
        }
    }

    out << "\"skeleton\":{";
    bool first_chain = true;
    for (const auto& chain_name : chain_order)
    {
        if (!first_chain) out << ",";
        first_chain = false;

        const auto& joints = skeleton_map[chain_name];
        out << "\"" << escapeJson(chain_name) << "\":";

        if (chain_name == "hand" && !joints.empty())
        {
            writeNodeJson(out, joints.begin()->second);
        }
        else
        {
            out << "{";
            bool first_joint = true;
            static const std::vector<std::string> joint_order = {
                "metacarpal", "proximal", "intermediate", "distal", "tip"
            };
            for (const auto& jname : joint_order)
            {
                auto it = joints.find(jname);
                if (it == joints.end()) continue;
                if (!first_joint) out << ",";
                first_joint = false;
                out << "\"" << jname << "\":";
                writeNodeJson(out, it->second);
            }
            out << "}";
        }
    }
    out << "}";
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
    json << "],";
    writeRawSkeletonNodesJson(json, frame.raw_skeleton_nodes);
    json << ",";
    writeSkeletonDictJson(json, frame.raw_skeleton_nodes);
    json << "}\n";
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
