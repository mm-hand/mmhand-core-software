#include "ManusClient.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>

#include "ManusSDKTypeInitializers.h"

ManusClient* ManusClient::instance_ = nullptr;

namespace
{
double nowSeconds()
{
    using clock = std::chrono::system_clock;
    const auto now = clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

std::string sideToString(Side side)
{
    switch (side)
    {
    case Side_Left:
        return "left";
    case Side_Right:
        return "right";
    case Side_Center:
        return "center";
    default:
        return "unknown";
    }
}

Vec3 toVec3(const ManusVec3& value)
{
    return Vec3{value.x, value.y, value.z};
}

Quat toQuat(const ManusQuaternion& value)
{
    return Quat{value.w, value.x, value.y, value.z};
}

std::string sdkCode(SDKReturnCode code)
{
    return std::to_string(static_cast<int>(code));
}
}

ManusClient::ManusClient()
{
    instance_ = this;
}

ManusClient::~ManusClient()
{
    shutdown();
    if (instance_ == this)
    {
        instance_ = nullptr;
    }
}

bool ManusClient::initialize(const ManusClientConfig& config, std::string& error)
{
    config_ = config;

    SDKReturnCode result = CoreSdk_InitializeCore();
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_InitializeCore failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    result = CoreSdk_RegisterCallbackForOnConnect(*ManusClient::onConnected);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_RegisterCallbackForOnConnect failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    result = CoreSdk_RegisterCallbackForOnDisconnect(*ManusClient::onDisconnected);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_RegisterCallbackForOnDisconnect failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    result = CoreSdk_RegisterCallbackForOnLog(*ManusClient::onLog);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_RegisterCallbackForOnLog failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    result = CoreSdk_RegisterCallbackForLandscapeStream(*ManusClient::onLandscape);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_RegisterCallbackForLandscapeStream failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    result = CoreSdk_RegisterCallbackForErgonomicsStream(*ManusClient::onErgonomics);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_RegisterCallbackForErgonomicsStream failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    result = CoreSdk_RegisterCallbackForRawSkeletonStream(*ManusClient::onRawSkeleton);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_RegisterCallbackForRawSkeletonStream failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    result = CoreSdk_RegisterCallbackForRawDeviceDataStream(*ManusClient::onRawDeviceData);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_RegisterCallbackForRawDeviceDataStream failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    CoordinateSystemVUH coordinate_system;
    CoordinateSystemVUH_Init(&coordinate_system);
    coordinate_system.handedness = Side_Right;
    coordinate_system.up = AxisPolarity_PositiveZ;
    coordinate_system.view = AxisView_XFromViewer;
    coordinate_system.unitScale = 1.0f;

    result = CoreSdk_InitializeCoordinateSystemWithVUH(coordinate_system, true);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_InitializeCoordinateSystemWithVUH failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    initialized_ = true;
    return true;
}

bool ManusClient::connect(std::string& error)
{
    if (!initialized_)
    {
        error = "SDK is not initialized.";
        return false;
    }

    SDKReturnCode result = CoreSdk_LookForHosts(config_.host_search_seconds, config_.connect_locally);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_LookForHosts failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    uint32_t host_count = 0;
    result = CoreSdk_GetNumberOfAvailableHostsFound(&host_count);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_GetNumberOfAvailableHostsFound failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    if (host_count == 0)
    {
        error = config_.connect_locally
            ? "No local MANUS Core host found. Is MANUS Core running on this Windows machine?"
            : "No MANUS Core host found on the network. Check firewall and network discovery.";
        return false;
    }

    std::unique_ptr<ManusHost[]> hosts(new ManusHost[host_count]);
    result = CoreSdk_GetAvailableHostsFound(hosts.get(), host_count);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_GetAvailableHostsFound failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    result = CoreSdk_ConnectToHost(hosts[0]);
    if (result != SDKReturnCode_Success)
    {
        error = "CoreSdk_ConnectToHost failed with SDKReturnCode " + sdkCode(result);
        return false;
    }

    result = CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto);
    if (result != SDKReturnCode_Success)
    {
        std::cerr << "Warning: CoreSdk_SetRawSkeletonHandMotion failed with SDKReturnCode "
                  << sdkCode(result) << "\n";
    }

    connected_ = true;
    return true;
}

void ManusClient::shutdown()
{
    if (!initialized_.exchange(false))
    {
        return;
    }

    CoreSdk_ShutDown();
    connected_ = false;
}

std::vector<ManusFrame> ManusClient::consumeFrames()
{
    std::vector<ManusFrame> frames;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        frames.swap(pending_frames_);
    }
    // 锁已释放，SDK 回调可以继续工作；
    // 在主循环上下文中安全地查询并补全层级信息。

    for (auto& frame : frames)
    {
        if (!frame.valid || frame.raw_skeleton_nodes.empty())
        {
            continue;
        }
        const uint32_t glove_id = static_cast<uint32_t>(frame.glove_id);
        ensureNodeInfoCache(glove_id);
        populateNodeHierarchy(frame.raw_skeleton_nodes, glove_id);
    }

    return frames;
}

bool ManusClient::isConnected() const
{
    return connected_.load();
}

const std::vector<std::string>& ManusClient::ergonomicsNames()
{
    static const std::vector<std::string> names = {
        "LeftFingerThumbMCPSpread",
        "LeftFingerThumbMCPStretch",
        "LeftFingerThumbPIPStretch",
        "LeftFingerThumbDIPStretch",
        "LeftFingerIndexMCPSpread",
        "LeftFingerIndexMCPStretch",
        "LeftFingerIndexPIPStretch",
        "LeftFingerIndexDIPStretch",
        "LeftFingerMiddleMCPSpread",
        "LeftFingerMiddleMCPStretch",
        "LeftFingerMiddlePIPStretch",
        "LeftFingerMiddleDIPStretch",
        "LeftFingerRingMCPSpread",
        "LeftFingerRingMCPStretch",
        "LeftFingerRingPIPStretch",
        "LeftFingerRingDIPStretch",
        "LeftFingerPinkyMCPSpread",
        "LeftFingerPinkyMCPStretch",
        "LeftFingerPinkyPIPStretch",
        "LeftFingerPinkyDIPStretch",
        "RightFingerThumbMCPSpread",
        "RightFingerThumbMCPStretch",
        "RightFingerThumbPIPStretch",
        "RightFingerThumbDIPStretch",
        "RightFingerIndexMCPSpread",
        "RightFingerIndexMCPStretch",
        "RightFingerIndexPIPStretch",
        "RightFingerIndexDIPStretch",
        "RightFingerMiddleMCPSpread",
        "RightFingerMiddleMCPStretch",
        "RightFingerMiddlePIPStretch",
        "RightFingerMiddleDIPStretch",
        "RightFingerRingMCPSpread",
        "RightFingerRingMCPStretch",
        "RightFingerRingPIPStretch",
        "RightFingerRingDIPStretch",
        "RightFingerPinkyMCPSpread",
        "RightFingerPinkyMCPStretch",
        "RightFingerPinkyPIPStretch",
        "RightFingerPinkyDIPStretch"
    };
    return names;
}

void ManusClient::onConnected(const ManusHost* const)
{
    if (instance_)
    {
        instance_->connected_ = true;
    }
}

void ManusClient::onDisconnected(const ManusHost* const)
{
    if (instance_)
    {
        instance_->connected_ = false;
    }
}

void ManusClient::onLog(LogSeverity, const char* const log, uint32_t length)
{
    if (log && length > 0)
    {
        std::cerr << "[MANUS SDK] " << std::string(log, log + length) << "\n";
    }
}

void ManusClient::onLandscape(const Landscape* const landscape)
{
    if (instance_)
    {
        instance_->handleLandscape(landscape);
    }
}

void ManusClient::onErgonomics(const ErgonomicsStream* const ergonomics)
{
    if (instance_)
    {
        instance_->handleErgonomics(ergonomics);
    }
}

void ManusClient::onRawSkeleton(const SkeletonStreamInfo* const raw_skeleton_info)
{
    if (instance_)
    {
        instance_->handleRawSkeleton(raw_skeleton_info);
    }
}

void ManusClient::onRawDeviceData(const RawDeviceDataInfo* const raw_device_info)
{
    if (instance_)
    {
        instance_->handleRawDeviceData(raw_device_info);
    }
}

void ManusClient::handleLandscape(const Landscape* landscape)
{
    if (!landscape)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    glove_sides_.clear();
    for (uint32_t i = 0; i < landscape->gloveDevices.gloveCount; ++i)
    {
        const GloveLandscapeData& glove = landscape->gloveDevices.gloves[i];
        glove_sides_[glove.id] = sideToString(glove.side);
    }
}

void ManusClient::handleErgonomics(const ErgonomicsStream* ergonomics)
{
    if (!ergonomics)
    {
        return;
    }

    const double timestamp = nowSeconds();
    const std::vector<std::string>& names = ergonomicsNames();

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (uint32_t i = 0; i < ergonomics->dataCount; ++i)
    {
        const ErgonomicsData& data = ergonomics->data[i];

        ManusFrame frame;
        frame.seq = next_seq_++;
        frame.timestamp = timestamp;
        frame.dt = last_frame_timestamp_ > 0.0 ? timestamp - last_frame_timestamp_ : 0.0;
        last_frame_timestamp_ = timestamp;
        frame.glove_id = static_cast<int>(data.id);
        frame.is_user_id = data.isUserID;
        frame.side = data.isUserID ? "user" : "unknown";

        const auto side_it = glove_sides_.find(data.id);
        if (side_it != glove_sides_.end())
        {
            frame.side = side_it->second;
        }

        const int count = static_cast<int>(ErgonomicsDataType_MAX_SIZE);
        frame.ergonomics_values.reserve(count);
        frame.ergonomics_names.reserve(count);
        for (int index = 0; index < count; ++index)
        {
            frame.ergonomics_values.push_back(data.data[index]);
            if (index < static_cast<int>(names.size()))
            {
                frame.ergonomics_names.push_back(names[index]);
            }
            else
            {
                frame.ergonomics_names.push_back("ErgonomicsIndex" + std::to_string(index));
            }
        }

        const auto raw_it = raw_skeleton_by_glove_.find(data.id);
        if (raw_it != raw_skeleton_by_glove_.end())
        {
            frame.raw_skeleton_nodes = raw_it->second.nodes;
        }
        frame.raw_device_count = raw_device_count_;
        frame.valid = true;
        pending_frames_.push_back(std::move(frame));
    }
}

void ManusClient::handleRawSkeleton(const SkeletonStreamInfo* raw_skeleton_info)
{
    if (!raw_skeleton_info)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (uint32_t i = 0; i < raw_skeleton_info->skeletonsCount; ++i)
    {
        RawSkeletonInfo info;
        SDKReturnCode result = CoreSdk_GetRawSkeletonInfo(i, &info);
        if (result != SDKReturnCode_Success || info.nodesCount == 0)
        {
            continue;
        }

        std::vector<SkeletonNode> nodes(info.nodesCount);
        result = CoreSdk_GetRawSkeletonData(i, nodes.data(), info.nodesCount);
        if (result != SDKReturnCode_Success)
        {
            continue;
        }

        RawSkeletonSnapshot snapshot;
        const uint32_t sample_count = std::min(info.nodesCount, config_.max_raw_skeleton_nodes);
        snapshot.nodes.reserve(sample_count);
        for (uint32_t node_index = 0; node_index < sample_count; ++node_index)
        {
            const SkeletonNode& node = nodes[node_index];
            RawSkeletonNodeSample sample;
            sample.node_id = node.id;
            sample.position = toVec3(node.transform.position);
            sample.rotation = toQuat(node.transform.rotation);
            // hierarchy info (parent_id, chain_type, finger_joint_type)
            // will be populated later in consumeFrames() via cache.
            snapshot.nodes.push_back(sample);
        }
        raw_skeleton_by_glove_[info.gloveId] = std::move(snapshot);
    }
}

void ManusClient::handleRawDeviceData(const RawDeviceDataInfo* raw_device_info)
{
    if (!raw_device_info)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    raw_device_count_ = raw_device_info->rawDeviceDataCount;
}

void ManusClient::ensureNodeInfoCache(uint32_t glove_id)
{
    if (node_info_cache_.find(glove_id) != node_info_cache_.end())
    {
        return;
    }

    uint32_t node_count = 0;
    SDKReturnCode result = CoreSdk_GetRawSkeletonNodeCount(glove_id, node_count);
    if (result != SDKReturnCode_Success || node_count == 0)
    {
        return;
    }

    std::vector<NodeInfo> node_infos(node_count);
    result = CoreSdk_GetRawSkeletonNodeInfoArray(glove_id, node_infos.data(), node_count);
    if (result != SDKReturnCode_Success)
    {
        return;
    }

    std::unordered_map<uint32_t, NodeInfo> info_map;
    for (uint32_t i = 0; i < node_count; ++i)
    {
        info_map[node_infos[i].nodeId] = node_infos[i];
    }
    node_info_cache_[glove_id] = std::move(info_map);
}

void ManusClient::populateNodeHierarchy(
    std::vector<RawSkeletonNodeSample>& nodes,
    uint32_t glove_id)
{
    const auto cache_it = node_info_cache_.find(glove_id);
    if (cache_it == node_info_cache_.end())
    {
        return;
    }

    const auto& info_map = cache_it->second;
    for (auto& sample : nodes)
    {
        const auto it = info_map.find(sample.node_id);
        if (it != info_map.end())
        {
            sample.parent_id = it->second.parentId;
            sample.chain_type = static_cast<int>(it->second.chainType);
            sample.finger_joint_type = static_cast<int>(it->second.fingerJointType);
        }
    }
}
