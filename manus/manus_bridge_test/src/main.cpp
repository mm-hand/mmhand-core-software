#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include "CsvRecorder.h"
#include "DataPrinter.h"
#include "ManusClient.h"
#include "TcpSender.h"

namespace
{
std::atomic<bool> g_running{true};

void handleSignal(int)
{
    g_running = false;
}

struct AppOptions
{
    std::string mode = "debug";
    std::string output = "manus_log.csv";
    bool connect_locally = true;
    double print_hz = 10.0;
    uint32_t raw_nodes = 5;
    std::string config_path = "config/manus_bridge_config.json";
};

struct BridgeConfig
{
    std::string connection = "local";
    double print_hz = 10.0;
    uint32_t host_search_seconds = 1;
    uint32_t raw_skeleton_nodes_to_print = 5;
    std::string default_csv_output = "manus_log.csv";
    TcpSenderConfig tcp;
};

void printUsage()
{
    std::cout
        << "Usage:\n"
        << "  manus_bridge_test.exe --mode debug [--config config/manus_bridge_config.json] [--connection local|remote] [--print-hz 10]\n"
        << "  manus_bridge_test.exe --mode record --output manus_log.csv [--connection local|remote]\n";
}

std::string readTextFile(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
    {
        return {};
    }
    return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

std::string jsonStringValue(const std::string& text, const std::string& key, const std::string& fallback)
{
    const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch match;
    return std::regex_search(text, match, pattern) ? match[1].str() : fallback;
}

double jsonDoubleValue(const std::string& text, const std::string& key, double fallback)
{
    const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
    std::smatch match;
    return std::regex_search(text, match, pattern) ? std::stod(match[1].str()) : fallback;
}

uint32_t jsonUintValue(const std::string& text, const std::string& key, uint32_t fallback)
{
    const std::regex pattern("\"" + key + "\"\\s*:\\s*([0-9]+)");
    std::smatch match;
    return std::regex_search(text, match, pattern) ? static_cast<uint32_t>(std::stoul(match[1].str())) : fallback;
}

bool jsonBoolValue(const std::string& text, const std::string& key, bool fallback)
{
    const std::regex pattern("\"" + key + "\"\\s*:\\s*(true|false)");
    std::smatch match;
    if (!std::regex_search(text, match, pattern))
    {
        return fallback;
    }
    return match[1].str() == "true";
}

std::string jsonObjectValue(const std::string& text, const std::string& key)
{
    const std::string marker = "\"" + key + "\"";
    const size_t key_pos = text.find(marker);
    if (key_pos == std::string::npos)
    {
        return {};
    }
    const size_t brace_start = text.find('{', key_pos + marker.size());
    if (brace_start == std::string::npos)
    {
        return {};
    }
    int depth = 0;
    for (size_t i = brace_start; i < text.size(); ++i)
    {
        if (text[i] == '{')
        {
            ++depth;
        }
        else if (text[i] == '}')
        {
            --depth;
            if (depth == 0)
            {
                return text.substr(brace_start, i - brace_start + 1);
            }
        }
    }
    return {};
}

BridgeConfig loadBridgeConfig(const std::string& path)
{
    BridgeConfig config;
    const std::string text = readTextFile(path);
    if (text.empty())
    {
        return config;
    }

    config.connection = jsonStringValue(text, "connection", config.connection);
    config.print_hz = jsonDoubleValue(text, "print_hz", config.print_hz);
    config.host_search_seconds = jsonUintValue(text, "host_search_seconds", config.host_search_seconds);
    config.raw_skeleton_nodes_to_print = jsonUintValue(
        text,
        "raw_skeleton_nodes_to_print",
        config.raw_skeleton_nodes_to_print
    );
    config.default_csv_output = jsonStringValue(text, "default_csv_output", config.default_csv_output);
    const std::string tcp_text = jsonObjectValue(text, "tcp");
    config.tcp.enabled = jsonBoolValue(tcp_text, "enabled", config.tcp.enabled);
    config.tcp.target_ip = jsonStringValue(tcp_text, "target_ip", config.tcp.target_ip);
    config.tcp.target_port = static_cast<uint16_t>(jsonUintValue(tcp_text, "target_port", config.tcp.target_port));
    config.tcp.reconnect_s = jsonDoubleValue(tcp_text, "reconnect_s", config.tcp.reconnect_s);
    config.tcp.max_hz = jsonDoubleValue(tcp_text, "max_hz", config.tcp.max_hz);
    return config;
}

bool parseArgs(int argc, char** argv, AppOptions& options)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto requireValue = [&](const std::string& name) -> const char* {
            if (i + 1 >= argc)
            {
                std::cerr << "Missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--mode")
        {
            const char* value = requireValue(arg);
            if (!value) return false;
            options.mode = value;
        }
        else if (arg == "--output")
        {
            const char* value = requireValue(arg);
            if (!value) return false;
            options.output = value;
        }
        else if (arg == "--connection")
        {
            const char* value = requireValue(arg);
            if (!value) return false;
            const std::string connection = value;
            if (connection == "local")
            {
                options.connect_locally = true;
            }
            else if (connection == "remote")
            {
                options.connect_locally = false;
            }
            else
            {
                std::cerr << "--connection must be local or remote\n";
                return false;
            }
        }
        else if (arg == "--print-hz")
        {
            const char* value = requireValue(arg);
            if (!value) return false;
            options.print_hz = std::stod(value);
        }
        else if (arg == "--raw-nodes")
        {
            const char* value = requireValue(arg);
            if (!value) return false;
            options.raw_nodes = static_cast<uint32_t>(std::stoul(value));
        }
        else if (arg == "--config")
        {
            const char* value = requireValue(arg);
            if (!value) return false;
            options.config_path = value;
        }
        else if (arg == "--help" || arg == "-h")
        {
            printUsage();
            return false;
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << "\n";
            return false;
        }
    }

    if (options.mode != "debug" && options.mode != "record")
    {
        std::cerr << "--mode must be debug or record\n";
        return false;
    }

    return true;
}
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    AppOptions options;
    if (!parseArgs(argc, argv, options))
    {
        printUsage();
        return 1;
    }

    const BridgeConfig bridge_config = loadBridgeConfig(options.config_path);
    if (options.output == "manus_log.csv")
    {
        options.output = bridge_config.default_csv_output;
    }
    if (argc == 1)
    {
        options.print_hz = bridge_config.print_hz;
        options.raw_nodes = bridge_config.raw_skeleton_nodes_to_print;
        options.connect_locally = bridge_config.connection != "remote";
    }

    ManusClientConfig client_config;
    client_config.connect_locally = options.connect_locally;
    client_config.host_search_seconds = bridge_config.host_search_seconds;
    client_config.max_raw_skeleton_nodes = options.raw_nodes;

    ManusClient client;
    std::string error;
    if (!client.initialize(client_config, error))
    {
        std::cerr << "Failed to initialize MANUS SDK: " << error << "\n";
        return 2;
    }

    std::cout << "Looking for MANUS Core using "
              << (options.connect_locally ? "local" : "remote/network")
              << " mode...\n";
    if (!client.connect(error))
    {
        std::cerr << "Failed to connect to MANUS Core: " << error << "\n";
        return 3;
    }

    CsvRecorder recorder;
    if (options.mode == "record")
    {
        if (!recorder.open(options.output, error))
        {
            std::cerr << error << "\n";
            return 4;
        }
        std::cout << "Recording CSV to " << options.output << "\n";
    }

    TcpSender tcp_sender;
    if (!tcp_sender.open(bridge_config.tcp, error))
    {
        std::cerr << "Warning: " << error << "\n";
    }
    std::cout << "TCP JSON "
              << (tcp_sender.isEnabled() ? "enabled" : "disabled")
              << " target=" << bridge_config.tcp.target_ip << ":" << bridge_config.tcp.target_port
              << " max_hz=" << bridge_config.tcp.max_hz
              << "\n";
    std::cout << "Running in " << options.mode << " mode. Press Ctrl+C to stop.\n";
    uint64_t tcp_sent_total = 0;
    uint64_t tcp_sent_last_report = 0;
    uint64_t tcp_failed_total = 0;
    uint64_t manus_frame_total = 0;
    uint64_t manus_frame_last_report = 0;
    auto tcp_last_report = std::chrono::steady_clock::now();
    auto tcp_last_send = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    bool tcp_last_connected = false;

    while (g_running)
    {
        std::vector<ManusFrame> frames = client.consumeFrames();
        manus_frame_total += frames.size();
        if (!frames.empty())
        {
            const auto send_now = std::chrono::steady_clock::now();
            const double min_interval_s = bridge_config.tcp.max_hz > 0.0 ? 1.0 / bridge_config.tcp.max_hz : 0.0;
            const bool can_send = min_interval_s <= 0.0
                || std::chrono::duration<double>(send_now - tcp_last_send).count() >= min_interval_s;
            const ManusFrame& frame = frames.back();
            if (can_send && !tcp_sender.sendFrame(frame, error))
            {
                ++tcp_failed_total;
            }
            else if (can_send && tcp_sender.isEnabled() && frame.valid)
            {
                ++tcp_sent_total;
                tcp_last_send = send_now;
            }
        }

        if (tcp_sender.isEnabled())
        {
            const auto send_now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(send_now - tcp_last_send).count() >= 0.5)
            {
                ManusFrame heartbeat;
                heartbeat.seq = 0;
                heartbeat.timestamp = std::chrono::duration<double>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count();
                heartbeat.dt = 0.0;
                heartbeat.glove_id = 0;
                heartbeat.side = "heartbeat";
                heartbeat.valid = true;
                if (!tcp_sender.sendFrame(heartbeat, error))
                {
                    ++tcp_failed_total;
                }
                else
                {
                    ++tcp_sent_total;
                    tcp_last_send = send_now;
                }
            }
        }
        const auto now = std::chrono::steady_clock::now();
        const bool tcp_connected = tcp_sender.isConnected();
        if (tcp_connected != tcp_last_connected)
        {
            std::cout << "[TCP] " << (tcp_connected ? "connected" : "disconnected/waiting") << "\n";
            tcp_last_connected = tcp_connected;
        }
        if (tcp_sender.isEnabled() && now - tcp_last_report >= std::chrono::seconds(1))
        {
            const double elapsed = std::chrono::duration<double>(now - tcp_last_report).count();
            const uint64_t delta = tcp_sent_total - tcp_sent_last_report;
            const uint64_t manus_delta = manus_frame_total - manus_frame_last_report;
            std::cout << "[STATUS] manus=" << (client.isConnected() ? "connected" : "disconnected")
                      << " manus_hz=" << (elapsed > 0.0 ? static_cast<double>(manus_delta) / elapsed : 0.0)
                      << " tcp=" << (tcp_sender.isConnected() ? "connected" : "waiting")
                      << " sent_total=" << tcp_sent_total
                      << " sent_hz=" << (elapsed > 0.0 ? static_cast<double>(delta) / elapsed : 0.0)
                      << " failed_total=" << tcp_failed_total
                      << "\n";
            tcp_sent_last_report = tcp_sent_total;
            manus_frame_last_report = manus_frame_total;
            tcp_last_report = now;
        }
        if (recorder.isOpen())
        {
            recorder.writeFrames(frames);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    recorder.close();
    tcp_sender.close();
    client.shutdown();
    std::cout << "Stopped manus_bridge_test.\n";
    return 0;
}
