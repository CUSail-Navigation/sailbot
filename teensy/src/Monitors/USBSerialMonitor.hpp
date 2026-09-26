#pragma once
#include "SerialMonitorBase.hpp"

class USBSerialMonitor : public SerialMonitorBase {
public:
    void execute() override;

private:
    uint8_t temp_buffer[constants::serial::USB_BUFFER_LEN] = {};
};
