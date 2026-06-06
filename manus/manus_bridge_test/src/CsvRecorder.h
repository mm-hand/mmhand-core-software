#pragma once

#include <fstream>
#include <string>

#include "ManusClient.h"

class CsvRecorder
{
public:
    bool open(const std::string& path, std::string& error);
    void writeFrames(const std::vector<ManusFrame>& frames);
    void close();
    bool isOpen() const;

private:
    static std::string escape(const std::string& value);
    void writeHeader(const ManusFrame& frame);

    std::ofstream file_;
    bool header_written_ = false;
};
