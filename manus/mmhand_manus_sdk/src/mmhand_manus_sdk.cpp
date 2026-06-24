#include "mmhand_manus_sdk.h"

#include "ManusClient.h"

#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <iomanip>

namespace
{
std::mutex g_mutex;
std::unique_ptr<ManusClient> g_client;
std::string g_last_error;
std::string g_latest_json;

void setError(const std::string& text)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_last_error = text;
}

std::string escapeJson(const std::string& text)
{
    std::ostringstream out;
    for (char ch : text)
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
                out << "\\u00";
                const char* hex = "0123456789abcdef";
                out << hex[(ch >> 4) & 0xF] << hex[ch & 0xF];
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

std::string semanticKey(const std::string& value)
{
    if (value == "Hand") return "hand";
    if (value == "Thumb") return "thumb";
    if (value == "Index") return "index";
    if (value == "Middle") return "middle";
    if (value == "Ring") return "ring";
    if (value == "Pinky") return "pinky";
    if (value == "Metacarpal") return "metacarpal";
    if (value == "Proximal") return "proximal";
    if (value == "Intermediate") return "intermediate";
    if (value == "Distal") return "distal";
    if (value == "Tip") return "tip";
    return "unknown";
}

void writeNodeJson(std::ostringstream& out, const RawSkeletonNodeSample& node)
{
    out << "{";
    out << "\"node_id\":" << node.node_id << ",";
    out << "\"parent_id\":" << node.parent_id << ",";
    out << "\"side\":\"" << escapeJson(node.side) << "\",";
    out << "\"chain_type\":\"" << escapeJson(node.chain_type) << "\",";
    out << "\"finger_joint_type\":\"" << escapeJson(node.finger_joint_type) << "\",";
    out << "\"has_node_info\":" << (node.has_node_info ? "true" : "false") << ",";
    out << "\"pos\":[" << node.position.x << "," << node.position.y << "," << node.position.z << "],";
    out << "\"rot\":[" << node.rotation.w << "," << node.rotation.x << ","
        << node.rotation.y << "," << node.rotation.z << "]";
    out << "}";
}

void writeSemanticSkeletonJson(std::ostringstream& out, const ManusFrame& frame)
{
    out << "\"skeleton\":{";
    bool wrote_any = false;

    for (const RawSkeletonNodeSample& node : frame.raw_skeleton_nodes)
    {
        if (node.chain_type != "Hand")
        {
            continue;
        }
        out << "\"hand\":";
        writeNodeJson(out, node);
        wrote_any = true;
        break;
    }

    const char* fingers[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
    for (const char* finger : fingers)
    {
        bool wrote_finger_header = false;
        for (const RawSkeletonNodeSample& node : frame.raw_skeleton_nodes)
        {
            if (node.chain_type != finger)
            {
                continue;
            }
            if (!wrote_finger_header)
            {
                if (wrote_any) out << ",";
                out << "\"" << semanticKey(finger) << "\":{";
                wrote_finger_header = true;
                wrote_any = true;
            }
            else
            {
                out << ",";
            }
            out << "\"" << semanticKey(node.finger_joint_type) << "\":";
            writeNodeJson(out, node);
        }
        if (wrote_finger_header)
        {
            out << "}";
        }
    }

    out << "}";
}

std::string frameToJson(const ManusFrame& frame)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{";
    out << "\"type\":\"manus_frame\",";
    out << "\"version\":3,";
    out << "\"seq\":" << frame.seq << ",";
    out << "\"timestamp\":" << frame.timestamp << ",";
    out << "\"dt\":" << frame.dt << ",";
    out << "\"glove_id\":" << frame.glove_id << ",";
    out << "\"side\":\"" << escapeJson(frame.side) << "\",";

    out << "\"ergonomics_names\":[";
    for (size_t i = 0; i < frame.ergonomics_names.size(); ++i)
    {
        if (i) out << ",";
        out << "\"" << escapeJson(frame.ergonomics_names[i]) << "\"";
    }
    out << "],";

    out << "\"ergonomics_values\":[";
    for (size_t i = 0; i < frame.ergonomics_values.size(); ++i)
    {
        if (i) out << ",";
        out << frame.ergonomics_values[i];
    }
    out << "],";
    writeSemanticSkeletonJson(out, frame);
    out << "}";
    return out.str();
}

ManusClient* clientUnsafe()
{
    return g_client.get();
}
}

extern "C" {

int mmhand_manus_start(
    int connect_locally,
    unsigned int host_search_seconds,
    unsigned int max_raw_skeleton_nodes)
{
    std::unique_ptr<ManusClient> next(new ManusClient());
    ManusClientConfig config;
    config.connect_locally = connect_locally != 0;
    config.host_search_seconds = host_search_seconds ? host_search_seconds : 1;
    config.max_raw_skeleton_nodes = max_raw_skeleton_nodes ? max_raw_skeleton_nodes : 64;

    std::string error;
    if (!next->initialize(config, error))
    {
        setError(error);
        return 0;
    }
    if (!next->connect(error))
    {
        next->shutdown();
        setError(error);
        return 0;
    }

    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_client)
        {
            g_client->shutdown();
        }
        g_client = std::move(next);
        g_last_error.clear();
        g_latest_json.clear();
    }
    return 1;
}

void mmhand_manus_stop()
{
    std::unique_ptr<ManusClient> old;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        old = std::move(g_client);
        g_latest_json.clear();
    }
    if (old)
    {
        old->shutdown();
    }
}

int mmhand_manus_is_connected()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    ManusClient* client = clientUnsafe();
    return (client && client->isConnected()) ? 1 : 0;
}

int mmhand_manus_get_latest_frame_json(char* buffer, int buffer_size)
{
    std::vector<ManusFrame> frames;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        ManusClient* client = clientUnsafe();
        if (!client)
        {
            return 0;
        }
        frames = client->consumeFrames();
    }

    if (frames.empty())
    {
        return 0;
    }

    const std::string json = frameToJson(frames.back());
    const int required = static_cast<int>(json.size()) + 1;
    if (!buffer || buffer_size < required)
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_latest_json = json;
        return -required;
    }

    memcpy(buffer, json.c_str(), static_cast<size_t>(required));
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_latest_json = json;
    }
    return required - 1;
}

const char* mmhand_manus_last_error()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_last_error.c_str();
}

}
