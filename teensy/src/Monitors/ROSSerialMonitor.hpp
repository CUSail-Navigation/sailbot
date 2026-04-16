#pragma once

#include "sfr.hpp"
#include "constants.hpp"

class ROSSerialMonitor
{
public:
    ROSSerialMonitor();
    void execute();

private:
    uint8_t buffer_index;
    bool packet_started;
    uint32_t packet_start_time;
};
