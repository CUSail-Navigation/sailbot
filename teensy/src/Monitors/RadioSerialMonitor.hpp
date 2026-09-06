#pragma once
#include "SerialMonitorBase.hpp"

class RadioSerialMonitor : public SerialMonitorBase {
public:
    void execute() override;
};
