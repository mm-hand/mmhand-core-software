#include "mmhand_manus_sdk.h"

#include "ManusClient.h"

#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

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

std::string chainTypeName(int chain_type)
{
    // ChainType enum: Invalid=0, Arm=1, Leg=2, Neck=3, Spine=4,
    //   FingerThumb=5, FingerIndex=6, FingerMiddle=7, FingerRing=8,
    //   FingerPinky=9, Pelvis=10, Head=11, Shoulder=12, Hand=13, Foot=14, Toe=15
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
    // FingerJointType: Invalid=0, Metacarpal=1, Proximal=2,
    //   Intermediate=3, Distal=4, Tip=5
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
    // 按 chain（手指名）分组，每个 chain 下按 joint 名分组
    // 使用 ordered map 保证输出顺序稳定
    std::map<std::string, std::map<std::string, RawSkeletonNodeSample>> skeleton_map;
    std::vector<std::string> chain_order; // 保持出现顺序

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
        // 同一 chain 同一 joint 只保留第一个节点
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
            // hand 是单节点，直接输出
            writeNodeJson(out, joints.begin()->second);
        }
        else
        {
            // 手指 chain，嵌套 joint 名
            out << "{";
            bool first_joint = true;
            // 按固定顺序输出关节
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

std::string frameToJson(const ManusFrame& frame)
{
    std::ostringstream out;
    out.precision(10);
    out << "{";
    out << "\"type\":\"manus_frame\",";
    out << "\"version\":1,";
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

    writeRawSkeletonNodesJson(out, frame.raw_skeleton_nodes);
    out << ",";
    writeSkeletonDictJson(out, frame.raw_skeleton_nodes);

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
    config.max_raw_skeleton_nodes = max_raw_skeleton_nodes ? max_raw_skeleton_nodes : 5;

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
    // 始终先检查是否有上次因 buffer 太小而未成功返回的缓存 JSON，
    // 避免重复调用 consumeFrames() 导致帧丢失。
    std::string json;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        ManusClient* client = clientUnsafe();
        if (!client)
        {
            return 0;
        }

        // 优先使用上次缓存的 JSON（处理 buffer resize 重试）
        if (!g_latest_json.empty())
        {
            json.swap(g_latest_json);
        }
        else
        {
            std::vector<ManusFrame> frames = client->consumeFrames();
            if (frames.empty())
            {
                return 0;
            }
            json = frameToJson(frames.back());
        }
    }

    const int required = static_cast<int>(json.size()) + 1;
    if (!buffer || buffer_size < required)
    {
        // buffer 太小：缓存 JSON 供下次调用，返回所需大小
        std::lock_guard<std::mutex> lock(g_mutex);
        g_latest_json = std::move(json);
        return -required;
    }

    memcpy(buffer, json.c_str(), static_cast<size_t>(required));
    return required - 1;
}

const char* mmhand_manus_last_error()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_last_error.c_str();
}

}
