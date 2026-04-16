#pragma once

#include <Arduino.h>
#include "sfr.hpp"
#include "constants.hpp"

class RadioSerialMonitor
{
public:
    RadioSerialMonitor();
    void execute();

private:
    uint8_t buffer_index;
    bool packet_started;
    uint32_t packet_start_time;
};
