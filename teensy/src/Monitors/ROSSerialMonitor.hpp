#pragma once
#include "sfr.hpp"

class ROSSerialMonitor {
public:
    ROSSerialMonitor();
    void execute();

private:
    uint8_t buffer_index;
    bool packet_started;
    uint32_t packet_start_time;

    void drop_packet();
};
