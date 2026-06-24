#include "DataPrinter.h"

#include <iomanip>
#include <iostream>

DataPrinter::DataPrinter(double print_hz)
    : last_print_(std::chrono::steady_clock::now()),
      interval_(print_hz > 0.0 ? 1.0 / print_hz : 0.1)
{
}

bool DataPrinter::shouldPrint()
{
    const auto now = std::chrono::steady_clock::now();
    if (now - last_print_ >= interval_)
    {
        last_print_ = now;
        return true;
    }
    return false;
}

void DataPrinter::print(const std::vector<ManusFrame>& frames)
{
    if (frames.empty())
    {
        return;
    }

    std::cout << "\n=== MANUS frames: " << frames.size() << " ===\n";
    for (const ManusFrame& frame : frames)
    {
        if (!frame.valid)
        {
            continue;
        }

        std::cout << "seq=" << frame.seq
                  << " timestamp=" << std::fixed << std::setprecision(6) << frame.timestamp
                  << " dt=" << std::setprecision(4) << frame.dt
                  << " glove_id=" << frame.glove_id
                  << " side=" << frame.side
                  << " ergonomics_count=" << frame.ergonomics_values.size()
                  << " raw_nodes=" << frame.raw_skeleton_nodes.size()
                  << " raw_devices=" << frame.raw_device_count
                  << "\n";

        for (size_t i = 0; i < frame.ergonomics_values.size(); ++i)
        {
            const std::string& name = i < frame.ergonomics_names.size()
                ? frame.ergonomics_names[i]
                : std::string("unknown");
            std::cout << "  ergo[" << std::setw(2) << i << "] "
                      << std::setw(34) << std::left << name << std::right
                      << " = " << std::fixed << std::setprecision(3)
                      << frame.ergonomics_values[i] << "\n";
        }

        if (!frame.raw_skeleton_nodes.empty())
        {
            std::cout << "  raw skeleton samples:\n";
            for (const RawSkeletonNodeSample& node : frame.raw_skeleton_nodes)
            {
                std::cout << "    node_id=" << node.node_id
                          << " parent_id=" << node.parent_id
                          << " side=" << node.side
                          << " chain=" << node.chain_type
                          << " joint=" << node.finger_joint_type
                          << " info=" << (node.has_node_info ? "yes" : "no")
                          << " pos=(" << node.position.x << ", " << node.position.y << ", " << node.position.z << ")"
                          << " rot=(" << node.rotation.w << ", " << node.rotation.x << ", "
                          << node.rotation.y << ", " << node.rotation.z << ")\n";
            }
        }
    }
}
