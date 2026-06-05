#pragma once

#include <chrono>

#include "ManusClient.h"

class DataPrinter
{
public:
    explicit DataPrinter(double print_hz);

    bool shouldPrint();
    void print(const std::vector<ManusFrame>& frames);

private:
    std::chrono::steady_clock::time_point last_print_;
    std::chrono::duration<double> interval_;
};
