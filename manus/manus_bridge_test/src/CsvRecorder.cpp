#include "CsvRecorder.h"

#include <sstream>

bool CsvRecorder::open(const std::string& path, std::string& error)
{
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_)
    {
        error = "Could not open CSV output file: " + path;
        return false;
    }
    header_written_ = false;
    return true;
}

void CsvRecorder::writeFrames(const std::vector<ManusFrame>& frames)
{
    if (!file_)
    {
        return;
    }

    for (const ManusFrame& frame : frames)
    {
        if (!frame.valid)
        {
            continue;
        }

        if (!header_written_)
        {
            writeHeader(frame);
        }

        file_ << frame.seq << ','
              << frame.timestamp << ','
              << frame.dt << ','
              << frame.glove_id << ','
              << escape(frame.side) << ','
              << frame.raw_device_count;

        for (double value : frame.ergonomics_values)
        {
            file_ << ',' << value;
        }

        const size_t raw_count = frame.raw_skeleton_nodes.size();
        for (size_t i = 0; i < raw_count; ++i)
        {
            const RawSkeletonNodeSample& node = frame.raw_skeleton_nodes[i];
            file_ << ',' << node.node_id
                  << ',' << node.position.x
                  << ',' << node.position.y
                  << ',' << node.position.z
                  << ',' << node.rotation.w
                  << ',' << node.rotation.x
                  << ',' << node.rotation.y
                  << ',' << node.rotation.z;
        }

        file_ << '\n';
    }
    file_.flush();
}

void CsvRecorder::close()
{
    if (file_)
    {
        file_.flush();
        file_.close();
    }
}

bool CsvRecorder::isOpen() const
{
    return file_.is_open();
}

std::string CsvRecorder::escape(const std::string& value)
{
    if (value.find_first_of(",\"\n") == std::string::npos)
    {
        return value;
    }

    std::ostringstream out;
    out << '"';
    for (char c : value)
    {
        if (c == '"')
        {
            out << "\"\"";
        }
        else
        {
            out << c;
        }
    }
    out << '"';
    return out.str();
}

void CsvRecorder::writeHeader(const ManusFrame& frame)
{
    file_ << "seq,timestamp,dt,glove_id,side,raw_device_count";
    for (size_t i = 0; i < frame.ergonomics_names.size(); ++i)
    {
        file_ << ",ergo_" << i << '_' << escape(frame.ergonomics_names[i]);
    }
    for (size_t i = 0; i < frame.raw_skeleton_nodes.size(); ++i)
    {
        file_ << ",raw_node" << i << "_id"
              << ",raw_node" << i << "_pos_x"
              << ",raw_node" << i << "_pos_y"
              << ",raw_node" << i << "_pos_z"
              << ",raw_node" << i << "_rot_w"
              << ",raw_node" << i << "_rot_x"
              << ",raw_node" << i << "_rot_y"
              << ",raw_node" << i << "_rot_z";
    }
    file_ << '\n';
    header_written_ = true;
}
