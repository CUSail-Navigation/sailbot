#pragma once
#include "SerialMonitorBase.hpp"

class ROSSerialMonitor : public SerialMonitorBase {
public:
    void execute() override;

private:
    uint8_t temp_buffer[constants::serial::BUFFER_LEN] = {};
};
