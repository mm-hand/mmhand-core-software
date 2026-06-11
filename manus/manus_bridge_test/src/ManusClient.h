#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "ManusSDK.h"

struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Quat
{
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct RawSkeletonNodeSample
{
    uint32_t node_id = 0;
    Vec3 position;
    Quat rotation;
};

struct ManusFrame
{
    uint64_t seq = 0;
    double timestamp = 0.0;
    double dt = 0.0;

    int glove_id = 0;
    std::string side = "unknown";

    std::vector<double> ergonomics_values;
    std::vector<std::string> ergonomics_names;
    std::vector<RawSkeletonNodeSample> raw_skeleton_nodes;

    uint32_t raw_device_count = 0;
    bool is_user_id = false;
    bool valid = false;
};

struct ManusClientConfig
{
    bool connect_locally = true;
    uint32_t host_search_seconds = 1;
    uint32_t max_raw_skeleton_nodes = 5;
};

class ManusClient
{
public:
    ManusClient();
    ~ManusClient();

    ManusClient(const ManusClient&) = delete;
    ManusClient& operator=(const ManusClient&) = delete;

    bool initialize(const ManusClientConfig& config, std::string& error);
    bool connect(std::string& error);
    void shutdown();

    std::vector<ManusFrame> consumeFrames();
    bool isConnected() const;

    static const std::vector<std::string>& ergonomicsNames();

private:
    struct RawSkeletonSnapshot
    {
        std::vector<RawSkeletonNodeSample> nodes;
    };

    static void onConnected(const ManusHost* const host);
    static void onDisconnected(const ManusHost* const host);
    static void onLog(LogSeverity severity, const char* const log, uint32_t length);
    static void onLandscape(const Landscape* const landscape);
    static void onErgonomics(const ErgonomicsStream* const ergonomics);
    static void onRawSkeleton(const SkeletonStreamInfo* const raw_skeleton_info);
    static void onRawDeviceData(const RawDeviceDataInfo* const raw_device_info);

    void handleLandscape(const Landscape* landscape);
    void handleErgonomics(const ErgonomicsStream* ergonomics);
    void handleRawSkeleton(const SkeletonStreamInfo* raw_skeleton_info);
    void handleRawDeviceData(const RawDeviceDataInfo* raw_device_info);

    static ManusClient* instance_;

    ManusClientConfig config_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> connected_{false};
    std::atomic<uint64_t> next_seq_{1};

    std::mutex data_mutex_;
    std::vector<ManusFrame> pending_frames_;
    std::unordered_map<uint32_t, std::string> glove_sides_;
    std::unordered_map<uint32_t, RawSkeletonSnapshot> raw_skeleton_by_glove_;
    uint32_t raw_device_count_ = 0;
    double last_frame_timestamp_ = 0.0;
};
