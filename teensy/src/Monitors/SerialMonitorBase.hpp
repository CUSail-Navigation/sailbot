#pragma once
#include "sfr.hpp"
#include <algorithm>

/** An abstract base class: stores shared functionality for serial monitors that assemble packets. */
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
    [[nodiscard]] bool packet_timed_out() const;
};
