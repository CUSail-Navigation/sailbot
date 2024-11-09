#pragma once

#include "sfr.hpp"
#include "constants.hpp"

class SerialMonitor
{
public:
    SerialMonitor();
    void execute();

private:
    uint8_t buffer_index;
    bool packet_started;
};