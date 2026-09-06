#pragma once
#include "sfr.hpp"

/** An abstract base class to store shared implementation for serial monitors. */
class SerialMonitorBase {
public:
    virtual void execute() = 0;

protected:
    uint8_t buffer_index;
    bool packet_started;
    uint32_t packet_start_time;

    SerialMonitorBase();
    virtual ~SerialMonitorBase() = default;

    void drop_packet();
    bool packet_timed_out() const;
};
